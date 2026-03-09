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
 * CONFIG - AJUSTE PINOS
 * ========================= */
#define UART_PORT           UART_NUM_2
#define UART_TX_PIN         GPIO_NUM_17
#define UART_RX_PIN         GPIO_NUM_16
#define RS485_EN_PIN        GPIO_NUM_4   

#define UART_BAUD           115200

/* =========================
 * PROTOCOLO
 * ========================= */
#define SOF                 0x7E
#define ESC                 0x7D
#define ESC_XOR             0x20

#define ADDR_STM32          0x01
#define ADDR_MASTER         0xF0

#define TYPE_READ_STATUS    0x04 // Usado para Sincronismo IHM <-> MI
#define TYPE_ACK            0x80
#define TYPE_NACK           0x81

#define MAX_PAYLOAD         128
#define MAX_FRAME_RAW       (1 + 4 + MAX_PAYLOAD + 2)
#define MAX_FRAME_ESC       (MAX_FRAME_RAW * 2)

#define FRAME_QUEUE_LEN     8

static const char *TAG = "IHM_MASTER";

/* =========================
 * ESTRUTURAS DA APLICAÇÃO (MI)
 * ========================= */
typedef struct {
    uint8_t  buttons;      // Bit 0: Start, Bit 1: Stop, Bit 2: Reset
    uint16_t target_freq;  // 0 - 600 (Equivale a 0.0 - 60.0 Hz)
    uint16_t ramp_time;    // Tempo em ms
    uint8_t  brake;        // 0 ou 1
} ihm_cmd_t;

typedef struct {
    uint16_t current_speed; // RPM
    uint16_t motor_current; // mA
    uint16_t bus_voltage;   // V
    uint8_t  temp;          // °C
} mi_sensors_t;

// Variáveis globais para interface
static ihm_cmd_t g_ihm_cmd = { .buttons = 0, .target_freq = 300, .ramp_time = 1000, .brake = 0 };
static mi_sensors_t g_mi_status = { 0 };

/* =========================
 * CRC16 (IBM / Modbus)
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

static inline void rs485_set_tx(bool tx_enable)
{
    gpio_set_level(RS485_EN_PIN, tx_enable ? 1 : 0);
}

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
    uint16_t crc = crc16_ibm(&raw[1], idx - 1);
    raw[idx++] = (uint8_t)(crc & 0xFF);
    raw[idx++] = (uint8_t)((crc >> 8) & 0xFF);
    if (out_max < 1) return 0;
    out_escaped[0] = SOF;
    size_t esc_len = slip_escape(&raw[1], idx - 1, &out_escaped[1], out_max - 1);
    return 1 + esc_len;
}

/* =========================
 * PARSER & QUEUE
 * ========================= */
typedef enum { PS_WAIT_SOF = 0, PS_HDR_ADDR, PS_HDR_TYPE, PS_HDR_SEQ, PS_HDR_LEN, PS_PAYLOAD, PS_CRC_L, PS_CRC_H } parse_state_t;

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

static void parser_reset(frame_parser_t *p) {
    p->st = PS_WAIT_SOF; p->esc_next = false; p->pay_i = 0; p->last_byte_us = 0;
}

static bool parser_feed(frame_parser_t *p, uint8_t byte, frame_t *out_frame)
{
    int64_t now = esp_timer_get_time();
    if (p->st != PS_WAIT_SOF && p->last_byte_us != 0 && (now - p->last_byte_us) > 5000) parser_reset(p);
    p->last_byte_us = now;

    if (byte == SOF) { p->st = PS_HDR_ADDR; p->esc_next = false; p->pay_i = 0; return false; }
    if (p->st == PS_WAIT_SOF) return false;
    if (p->esc_next) { byte ^= ESC_XOR; p->esc_next = false; }
    else if (byte == ESC) { p->esc_next = true; return false; }

    switch (p->st) {
        case PS_HDR_ADDR: p->addr = byte; p->st = PS_HDR_TYPE; break;
        case PS_HDR_TYPE: p->type = byte; p->st = PS_HDR_SEQ; break;
        case PS_HDR_SEQ:  p->seq  = byte; p->st = PS_HDR_LEN; break;
        case PS_HDR_LEN:
            p->len = byte;
            if (p->len > MAX_PAYLOAD) { parser_reset(p); return false; }
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
            tmp[0] = p->addr; tmp[1] = p->type; tmp[2] = p->seq; tmp[3] = p->len;
            if (p->len) memcpy(&tmp[4], p->payload, p->len);
            uint16_t crc_calc = crc16_ibm(tmp, 4 + p->len);
            uint16_t crc_rx   = (uint16_t)p->crc_l | ((uint16_t)p->crc_h << 8);
            if (crc_calc == crc_rx) {
                out_frame->addr = p->addr; out_frame->type = p->type; out_frame->seq = p->seq; out_frame->len = p->len;
                if (p->len) memcpy(out_frame->payload, p->payload, p->len);
                parser_reset(p); return true;
            }
            parser_reset(p); return false;
        } break;
        default: parser_reset(p); break;
    }
    return false;
}

static QueueHandle_t g_frame_q;
static void rx_task(void *arg) {
    uint8_t buf[64];
    frame_parser_t parser;
    parser_reset(&parser);
    while (1) {
        int n = uart_read_bytes(UART_PORT, buf, sizeof(buf), pdMS_TO_TICKS(10));
        for (int i = 0; i < n; i++) {
            frame_t fr;
            if (parser_feed(&parser, buf[i], &fr)) {
                if (uxQueueSpacesAvailable(g_frame_q) == 0) { frame_t dump; xQueueReceive(g_frame_q, &dump, 0); }
                xQueueSend(g_frame_q, &fr, 0);
            }
        }
    }
}

/* =========================
 * COMMUNICATION CORE
 * ========================= */
static esp_err_t rs485_send_frame(uint8_t addr, uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t len) {
    uint8_t txbuf[MAX_FRAME_ESC];
    size_t txlen = build_frame(addr, type, seq, payload, len, txbuf, sizeof(txbuf));
    if (txlen == 0) return ESP_FAIL;
    rs485_set_tx(true);
    esp_rom_delay_us(10);
    uart_write_bytes(UART_PORT, (const char*)txbuf, txlen);
    uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(50));
    rs485_set_tx(false);
    esp_rom_delay_us(20);
    return ESP_OK;
}

static bool rs485_request(uint8_t type, const uint8_t *payload, uint8_t len, frame_t *reply, uint32_t timeout_ms) {
    static uint8_t seq = 0;
    seq++;
    frame_t dump;
    while (xQueueReceive(g_frame_q, &dump, 0) == pdTRUE) {}

    for (int r = 0; r < 2; r++) { // Reduzido para 2 retries para manter tempo real
        rs485_send_frame(ADDR_STM32, type, seq, payload, len);
        int64_t t0 = esp_timer_get_time();
        while (((esp_timer_get_time() - t0)/1000) < timeout_ms) {
            frame_t fr;
            if (xQueueReceive(g_frame_q, &fr, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
                if (fr.addr == ADDR_MASTER && fr.seq == seq) {
                    if (reply) *reply = fr;
                    return true;
                }
            }
        }
    }
    return false;
}

/* =========================
 * IHM MAIN TASK (MI SYNC)
 * ========================= */
static void ihm_sync_task(void *arg)
{
    frame_t rep;
    uint8_t tx_payload[6];

    while (1) {
        // 1. Empacotar Comandos da IHM para o MI
        tx_payload[0] = g_ihm_cmd.buttons;
        tx_payload[1] = (uint8_t)(g_ihm_cmd.target_freq >> 8);
        tx_payload[2] = (uint8_t)(g_ihm_cmd.target_freq & 0xFF);
        tx_payload[3] = (uint8_t)(g_ihm_cmd.ramp_time >> 8);
        tx_payload[4] = (uint8_t)(g_ihm_cmd.ramp_time & 0xFF);
        tx_payload[5] = g_ihm_cmd.brake;

        // 2. Solicitar sincronismo (Polling 50ms)
        if (rs485_request(TYPE_READ_STATUS, tx_payload, 6, &rep, 40)) {
            if (rep.type == TYPE_ACK && rep.len == 7) {
                // 3. Desempacotar Sensores recebidos do MI
                g_mi_status.current_speed = (rep.payload[0] << 8) | rep.payload[1];
                g_mi_status.motor_current = (rep.payload[2] << 8) | rep.payload[3];
                g_mi_status.bus_voltage   = (rep.payload[4] << 8) | rep.payload[5];
                g_mi_status.temp          = rep.payload[6];

                ESP_LOGI(TAG, "Sync OK | RPM: %u | Curr: %u mA | Vbus: %u V | MI Temp: %d C", 
                         g_mi_status.current_speed, g_mi_status.motor_current, 
                         g_mi_status.bus_voltage, g_mi_status.temp);
            }
        } else {
            ESP_LOGW(TAG, "MI Communication Lost!");
        }

        vTaskDelay(pdMS_TO_TICKS(3000)); // Ciclo de 50ms para controle em tempo real
    }
}

void app_main(void)
{
    // GPIO Init
    gpio_config_t io = { .pin_bit_mask = 1ULL << RS485_EN_PIN, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&io);
    rs485_set_tx(false);

    // UART Init
    uart_config_t cfg = {
        .baud_rate = UART_BAUD, .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN, .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT
    };
    uart_driver_install(UART_PORT, 2048, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &cfg);
    uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    g_frame_q = xQueueCreate(FRAME_QUEUE_LEN, sizeof(frame_t));

    xTaskCreate(rx_task, "rs485_rx", 4096, NULL, 10, NULL);
    xTaskCreate(ihm_sync_task, "ihm_sync", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "IHM Master Interface Online");
}