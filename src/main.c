#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

/* =========================
 *  CONFIG - AJUSTE PINOS
 * ========================= */
#define UART_PORT           UART_NUM_2
#define UART_TX_PIN         GPIO_NUM_17
#define UART_RX_PIN         GPIO_NUM_16
#define RS485_EN_PIN        GPIO_NUM_4   // DE e /RE amarrados: 1=TX, 0=RX

#define UART_BAUD           115200

/* =========================
 *  PROTOCOLO
 * ========================= */
#define SOF                 0x7E
#define ESC                 0x7D
#define ESC_XOR             0x20

#define ADDR_STM32          0x01
#define ADDR_MASTER         0xF0

#define TYPE_PING           0x01
#define TYPE_SET_OUTPUT     0x02
#define TYPE_SET_PWM        0x03
#define TYPE_READ_STATUS    0x04

#define TYPE_ACK            0x80
#define TYPE_NACK           0x81

#define MAX_PAYLOAD         128
#define MAX_FRAME_RAW       (1 + 4 + MAX_PAYLOAD + 2)
#define MAX_FRAME_ESC       (MAX_FRAME_RAW * 2)

#define FRAME_QUEUE_LEN     8

static const char *TAG = "RS485_MASTER";

/* =========================
 *  CRC16 (IBM / Modbus) poly 0xA001
 * ========================= */
static uint16_t crc16_ibm(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else         crc >>= 1;
        }
    }
    return crc;
}

/* =========================
 *  RS485 EN (DE=/RE)
 * ========================= */
static inline void rs485_set_tx(bool tx_enable)
{
    gpio_set_level(RS485_EN_PIN, tx_enable ? 1 : 0);
}

/* =========================
 *  Escape helper
 * ========================= */
static size_t slip_escape(const uint8_t *in, size_t in_len, uint8_t *out, size_t out_max)
{
    size_t j = 0;
    for (size_t i = 0; i < in_len; i++) {
        uint8_t b = in[i];
        if (b == SOF || b == ESC) {
            if (j + 2 > out_max) return 0;
            out[j++] = ESC;
            out[j++] = (uint8_t)(b ^ ESC_XOR);
        } else {
            if (j + 1 > out_max) return 0;
            out[j++] = b;
        }
    }
    return j;
}

/* =========================
 *  Frame builder:
 *  [SOF][ADDR][TYPE][SEQ][LEN][PAYLOAD...][CRC16 L][CRC16 H]
 *  Escape em tudo EXCETO o SOF inicial
 * ========================= */
static size_t build_frame(uint8_t addr, uint8_t type, uint8_t seq,
                          const uint8_t *payload, uint8_t len,
                          uint8_t *out_escaped, size_t out_max)
{
    if (len > MAX_PAYLOAD) return 0;

    uint8_t raw[MAX_FRAME_RAW];
    size_t idx = 0;

    raw[idx++] = SOF;
    raw[idx++] = addr;
    raw[idx++] = type;
    raw[idx++] = seq;
    raw[idx++] = len;

    if (len && payload) {
        memcpy(&raw[idx], payload, len);
        idx += len;
    }

    // CRC cobre ADDR..PAYLOAD (não inclui SOF)
    uint16_t crc = crc16_ibm(&raw[1], idx - 1);
    raw[idx++] = (uint8_t)(crc & 0xFF);
    raw[idx++] = (uint8_t)((crc >> 8) & 0xFF);

    if (out_max < 1) return 0;
    out_escaped[0] = SOF;

    size_t esc_len = slip_escape(&raw[1], idx - 1, &out_escaped[1], out_max - 1);
    if (esc_len == 0) return 0;

    return 1 + esc_len;
}

/* =========================
 *  Parser (byte-a-byte)
 * ========================= */
typedef enum {
    PS_WAIT_SOF = 0,
    PS_HDR_ADDR,
    PS_HDR_TYPE,
    PS_HDR_SEQ,
    PS_HDR_LEN,
    PS_PAYLOAD,
    PS_CRC_L,
    PS_CRC_H
} parse_state_t;

typedef struct {
    parse_state_t st;
    bool esc_next;

    uint8_t addr, type, seq, len;
    uint8_t payload[MAX_PAYLOAD];
    uint8_t pay_i;

    uint8_t crc_l, crc_h;

    int64_t last_byte_us;
} frame_parser_t;

typedef struct {
    uint8_t addr, type, seq, len;
    uint8_t payload[MAX_PAYLOAD];
} frame_t;

static void parser_reset(frame_parser_t *p)
{
    p->st = PS_WAIT_SOF;
    p->esc_next = false;
    p->addr = p->type = p->seq = p->len = 0;
    p->pay_i = 0;
    p->crc_l = p->crc_h = 0;
    p->last_byte_us = 0;
}

static bool parser_feed(frame_parser_t *p, uint8_t byte, frame_t *out_frame)
{
    int64_t now = esp_timer_get_time();

    // timeout inter-byte (5ms)
    if (p->st != PS_WAIT_SOF && p->last_byte_us != 0 && (now - p->last_byte_us) > 5000) {
        parser_reset(p);
    }
    p->last_byte_us = now;

    if (byte == SOF) {
        p->st = PS_HDR_ADDR;
        p->esc_next = false;
        p->pay_i = 0;
        return false;
    }

    if (p->st == PS_WAIT_SOF) return false;

    if (p->esc_next) {
        byte ^= ESC_XOR;
        p->esc_next = false;
    } else if (byte == ESC) {
        p->esc_next = true;
        return false;
    }

    switch (p->st) {
        case PS_HDR_ADDR: p->addr = byte; p->st = PS_HDR_TYPE; break;
        case PS_HDR_TYPE: p->type = byte; p->st = PS_HDR_SEQ; break;
        case PS_HDR_SEQ:  p->seq  = byte; p->st = PS_HDR_LEN; break;

        case PS_HDR_LEN:
            p->len = byte;
            if (p->len > MAX_PAYLOAD) { parser_reset(p); return false; }
            p->pay_i = 0;
            p->st = (p->len == 0) ? PS_CRC_L : PS_PAYLOAD;
            break;

        case PS_PAYLOAD:
            p->payload[p->pay_i++] = byte;
            if (p->pay_i >= p->len) p->st = PS_CRC_L;
            break;

        case PS_CRC_L: p->crc_l = byte; p->st = PS_CRC_H; break;

        case PS_CRC_H: {
            p->crc_h = byte;

            uint8_t tmp[4 + MAX_PAYLOAD];
            tmp[0] = p->addr;
            tmp[1] = p->type;
            tmp[2] = p->seq;
            tmp[3] = p->len;
            if (p->len) memcpy(&tmp[4], p->payload, p->len);

            uint16_t crc_calc = crc16_ibm(tmp, 4 + p->len);
            uint16_t crc_rx   = (uint16_t)p->crc_l | ((uint16_t)p->crc_h << 8);

            if (crc_calc == crc_rx) {
                out_frame->addr = p->addr;
                out_frame->type = p->type;
                out_frame->seq  = p->seq;
                out_frame->len  = p->len;
                if (p->len) memcpy(out_frame->payload, p->payload, p->len);

                parser_reset(p);
                return true;
            } else {
                ESP_LOGW(TAG, "CRC fail calc=0x%04X rx=0x%04X", crc_calc, crc_rx);
                parser_reset(p);
                return false;
            }
        } break;

        default:
            parser_reset(p);
            break;
    }

    return false;
}

/* =========================
 *  RX Task -> Queue
 * ========================= */
static QueueHandle_t g_frame_q;

static void rx_task(void *arg)
{
    (void)arg;
    uint8_t buf[64];
    frame_parser_t parser;
    parser_reset(&parser);

    while (1) {
        int n = uart_read_bytes(UART_PORT, buf, sizeof(buf), pdMS_TO_TICKS(50));
        for (int i = 0; i < n; i++) {
            frame_t fr;
            if (parser_feed(&parser, buf[i], &fr)) {
                // Se fila cheia, descarta o mais antigo e insere o novo
                if (uxQueueSpacesAvailable(g_frame_q) == 0) {
                    frame_t dump;
                    xQueueReceive(g_frame_q, &dump, 0);
                }
                xQueueSend(g_frame_q, &fr, 0);
            }
        }
    }
}

/* =========================
 *  Send frame (TX->RX) correto
 * ========================= */
static esp_err_t rs485_send_frame(uint8_t addr, uint8_t type, uint8_t seq,
                                 const uint8_t *payload, uint8_t len)
{
    uint8_t txbuf[MAX_FRAME_ESC];
    size_t txlen = build_frame(addr, type, seq, payload, len, txbuf, sizeof(txbuf));
    if (txlen == 0) return ESP_FAIL;

    // limpa buffer RX para evitar lixo/frames velhos
   ///uart_flush_input(UART_PORT);

    rs485_set_tx(true);
    esp_rom_delay_us(10);

    int w = uart_write_bytes(UART_PORT, (const char*)txbuf, txlen);
    if (w < 0) {
        rs485_set_tx(false);
        return ESP_FAIL;
    }

    esp_err_t e = uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(50));

    // turnaround ~300us
    rs485_set_tx(false);
    esp_rom_delay_us(20);
    
    return e;
}

/* =========================
 *  Helper: limpa frames pendentes na fila
 * ========================= */
static void purge_frame_queue(void)
{
    frame_t dump;
    while (xQueueReceive(g_frame_q, &dump, 0) == pdTRUE) {}
}

/* =========================
 *  Wait reply (filtra por addr+seq e aceita apenas ACK/NACK)
 * ========================= */
static bool wait_reply(uint8_t expected_seq, frame_t *reply, uint32_t timeout_ms)
{
    int64_t t0 = esp_timer_get_time();
    while (1) {
        int64_t now = esp_timer_get_time();
        int64_t elapsed_ms = (now - t0) / 1000;
        if (elapsed_ms >= (int64_t)timeout_ms) return false;

        uint32_t remain_ms = (uint32_t)(timeout_ms - elapsed_ms);
        TickType_t wait_ticks = pdMS_TO_TICKS(remain_ms);
        if (wait_ticks == 0) wait_ticks = 1;

        frame_t fr;
        if (xQueueReceive(g_frame_q, &fr, wait_ticks) == pdTRUE) {
            // valida addr + seq
            if (fr.addr == ADDR_MASTER && fr.seq == expected_seq) {
                // valida tipo
                if (fr.type == TYPE_ACK || fr.type == TYPE_NACK) {
                    if (reply) *reply = fr;
                    return true;
                }
            }
            // frame não era o esperado -> ignora e continua esperando dentro do timeout
        }
    }
}

/* =========================
 *  Request/Response com retry
 * ========================= */
static bool rs485_request(uint8_t type, const uint8_t *payload, uint8_t len,
                          frame_t *reply, uint32_t timeout_ms)
{
    static uint8_t seq = 0;
    seq++;

    purge_frame_queue();

    const int retry_max = 3;
    for (int r = 0; r < retry_max; r++) {
        if (rs485_send_frame(ADDR_STM32, type, seq, payload, len) != ESP_OK) {
            ESP_LOGW(TAG, "send fail retry=%d", r);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        frame_t fr;
        if (wait_reply(seq, &fr, timeout_ms)) {
            if (reply) *reply = fr;
            return true;
        }

        ESP_LOGW(TAG, "timeout waiting reply (retry=%d)", r);
    }

    return false;
}

/* =========================
 *  Demo task
 * ========================= */
static void demo_loop_task(void *arg)
{
    (void)arg;

    frame_t rep;

    if (rs485_request(TYPE_PING, NULL, 0, &rep, 80)) {
        ESP_LOGI(TAG, "PING -> reply type=0x%02X len=%u", rep.type, rep.len);
    } else {
        ESP_LOGE(TAG, "PING failed");
    }

    uint8_t duty = 49;
    uint8_t pl_pwm[1] = { duty };
    if (rs485_request(TYPE_SET_PWM, pl_pwm, 1, &rep, 120)) {
        if (rep.type == TYPE_ACK) ESP_LOGI(TAG, "SET_PWM ACK");
        else ESP_LOGW(TAG, "SET_PWM NACK err=%u", rep.len ? rep.payload[0] : 0);
    } else {
        ESP_LOGE(TAG, "SET_PWM failed");
    }

    while (1) {
        if (rs485_request(TYPE_READ_STATUS, NULL, 0, &rep, 120)) {
            if (rep.type == TYPE_ACK && rep.len >= 2) {
                ESP_LOGI(TAG, "STATUS state=%u duty=%u", rep.payload[0], rep.payload[1]);
            } else if (rep.type == TYPE_NACK) {
                ESP_LOGW(TAG, "STATUS NACK err=%u", rep.len ? rep.payload[0] : 0);
            } else {
                ESP_LOGW(TAG, "STATUS unexpected reply type=0x%02X len=%u", rep.type, rep.len);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << RS485_EN_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    rs485_set_tx(false);

    uart_config_t cfg = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    g_frame_q = xQueueCreate(FRAME_QUEUE_LEN, sizeof(frame_t));
    if (!g_frame_q) {
        ESP_LOGE(TAG, "Failed to create frame queue");
        return;
    }

    xTaskCreate(rx_task, "rs485_rx", 4096, NULL, 10, NULL);
    xTaskCreate(demo_loop_task, "demo", 4096, NULL, 9, NULL);

    ESP_LOGI(TAG, "RS485 Master started (115200 8E1)");
}
