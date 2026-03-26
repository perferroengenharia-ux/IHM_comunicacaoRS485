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
#include "nvs_flash.h"
#include "nvs.h"

/* =========================
 * CONFIG - HARDWARE & UART
 * ========================= */
#define UART_PORT           UART_NUM_2
#define UART_TX_PIN         GPIO_NUM_17
#define UART_RX_PIN         GPIO_NUM_16
#define RS485_EN_PIN        GPIO_NUM_4   
#define UART_BAUD           115200

/* =========================
 * PROTOCOLO RS485
 * ========================= */
#define SOF                 0x7E
#define ESC                 0x7D
#define ESC_XOR             0x20

#define ADDR_STM32          0x01
#define TYPE_READ_STATUS    0x04 
#define TYPE_WRITE_PARAM    0x05

#define MAX_PAYLOAD         128
#define MAX_FRAME_RAW       (1 + 4 + MAX_PAYLOAD + 2)
#define MAX_FRAME_ESC       (MAX_FRAME_RAW * 2)

/* =========================
 * ESTRUTURAS DE DADOS E TELEMETRIA
 * ========================= */
typedef struct {
    uint8_t id;
    uint16_t value;
    const char* name;
    bool pending_sync; 
} param_t;

param_t params[] = {
    {10, 10, "P10", false}, {11, 7,  "P11", false},
    {20, 1,  "P20", false}, {21, 60, "P21", false},
    {35, 0,  "P35", false}, {42, 5,  "P42", false},
    {43, 5,  "P43", false}, {44, 0,  "P44", false},
    {45, 180,"P45", false}, {85, 1,  "P85", false},
};
#define ACTIVE_PARAMS_COUNT (sizeof(params)/sizeof(params[0]))

// Estrutura para os dados que o IHM envia ao MI
typedef struct {
    uint8_t  buttons;       // Bitmask: Bit0=Start, Bit1=Stop, Bit2=Up, Bit3=Down
    uint16_t target_freq;   // Frequência alvo (ex: 6000 para 60.00Hz)
    uint8_t  direction;     // 0 = Horário (FWD), 1 = Anti-horário (REV)
} ihm_control_t;

// Estrutura para a Telemetria que o MI envia ao IHM
typedef struct {
    uint16_t current_freq;  // Freq. Atual
    uint16_t v_bus;         // Tensão Barramento CC
    uint16_t i_out;         // Corrente de Saída (mA)
    uint16_t v_out;         // Tensão de Saída (V)
    uint8_t  temp_igbt;     // Temperatura IGBTs
} mi_telemetry_t;

static ihm_control_t g_control = { .buttons = 0, .target_freq = 2000, .direction = 0 };
static mi_telemetry_t g_telemetry = { 0 };

static uint8_t  P90 = 0;      
static uint8_t  P91 = 15; 
static bool     E08 = false;  
static QueueHandle_t g_frame_q = NULL;
static bool     g_show_logs = false; 

typedef struct {
    uint8_t addr, type, seq, len, payload[MAX_PAYLOAD];
} frame_t;

/* =========================
 * UTILITÁRIOS RS485 & CRC
 * ========================= */
static uint16_t crc16_ibm(const uint8_t *data, size_t len) {
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

static inline void rs485_set_tx(bool tx_enable) {
    gpio_set_level(RS485_EN_PIN, tx_enable ? 1 : 0);
}

static size_t build_frame(uint8_t addr, uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t len, uint8_t *out_esc, size_t out_max) {
    uint8_t raw[MAX_FRAME_RAW];
    size_t r_idx = 0;
    raw[r_idx++] = SOF;
    raw[r_idx++] = addr;
    raw[r_idx++] = type;
    raw[r_idx++] = seq;
    raw[r_idx++] = len;
    if (len && payload) { memcpy(&raw[r_idx], payload, len); r_idx += len; }
    uint16_t crc = crc16_ibm(&raw[1], r_idx - 1);
    raw[r_idx++] = (uint8_t)(crc & 0xFF);
    raw[r_idx++] = (uint8_t)((crc >> 8) & 0xFF);

    size_t e_idx = 0;
    out_esc[e_idx++] = SOF;
    for (size_t i = 1; i < r_idx; i++) {
        if (raw[i] == SOF || raw[i] == ESC) {
            out_esc[e_idx++] = ESC;
            out_esc[e_idx++] = raw[i] ^ ESC_XOR;
        } else {
            out_esc[e_idx++] = raw[i];
        }
    }
    return e_idx;
}

/* =========================
 * PARSER & RX TASK
 * ========================= */
typedef enum { PS_WAIT_SOF, PS_HDR_ADDR, PS_HDR_TYPE, PS_HDR_SEQ, PS_HDR_LEN, PS_PAYLOAD, PS_CRC_L, PS_CRC_H } parse_state_t;
typedef struct {
    parse_state_t st;
    bool esc_next;
    uint8_t addr, type, seq, len, payload[MAX_PAYLOAD], pay_i, crc_l, crc_h;
} frame_parser_t;

static void parser_reset(frame_parser_t *p) { p->st = PS_WAIT_SOF; p->esc_next = false; p->pay_i = 0; }

static bool parser_feed(frame_parser_t *p, uint8_t byte, frame_t *out_frame) {
    if (byte == SOF) { parser_reset(p); p->st = PS_HDR_ADDR; return false; }
    if (p->st == PS_WAIT_SOF) return false;
    if (p->esc_next) { byte ^= ESC_XOR; p->esc_next = false; }
    else if (byte == ESC) { p->esc_next = true; return false; }

    switch (p->st) {
        case PS_HDR_ADDR: p->addr = byte; p->st = PS_HDR_TYPE; break;
        case PS_HDR_TYPE: p->type = byte; p->st = PS_HDR_SEQ; break;
        case PS_HDR_SEQ:  p->seq  = byte; p->st = PS_HDR_LEN; break;
        case PS_HDR_LEN:  
            if (byte > MAX_PAYLOAD) { parser_reset(p); return false; }
            p->len = byte; p->st = (p->len == 0) ? PS_CRC_L : PS_PAYLOAD; 
            break;
        case PS_PAYLOAD:  
            p->payload[p->pay_i++] = byte; 
            if (p->pay_i >= p->len) p->st = PS_CRC_L; 
            break;
        case PS_CRC_L:    p->crc_l = byte; p->st = PS_CRC_H; break;
        case PS_CRC_H: {
            p->crc_h = byte;
            uint8_t tmp[4 + MAX_PAYLOAD];
            tmp[0] = p->addr; tmp[1] = p->type; tmp[2] = p->seq; tmp[3] = p->len;
            if (p->len) memcpy(&tmp[4], p->payload, p->len);
            uint16_t calc = crc16_ibm(tmp, 4 + p->len);
            if (calc == ((uint16_t)p->crc_l | (p->crc_h << 8))) {
                out_frame->addr = p->addr; out_frame->type = p->type; 
                out_frame->seq = p->seq; out_frame->len = p->len;
                if (p->len) memcpy(out_frame->payload, p->payload, p->len);
                parser_reset(p); return true;
            }
            parser_reset(p);
        } break;
        default: parser_reset(p); break;
    }
    return false;
}

static void rx_task(void *arg) {
    uint8_t buf[128];
    frame_parser_t parser;
    parser_reset(&parser);
    while (1) {
        int n = uart_read_bytes(UART_PORT, buf, sizeof(buf), pdMS_TO_TICKS(10));
        if (n > 0) {
            for (int i = 0; i < n; i++) {
                frame_t fr;
                if (parser_feed(&parser, buf[i], &fr)) {
                    if (g_frame_q) xQueueSend(g_frame_q, &fr, 0);
                }
            }
        }
    }
}

static bool rs485_request(uint8_t type, const uint8_t *payload, uint8_t len, frame_t *reply, uint32_t timeout_ms) {
    static uint8_t seq = 0;
    seq++;
    uint8_t txbuf[MAX_FRAME_ESC];
    size_t txlen = build_frame(ADDR_STM32, type, seq, payload, len, txbuf, sizeof(txbuf));
    
    frame_t dump;
    while (xQueueReceive(g_frame_q, &dump, 0) == pdTRUE);

    rs485_set_tx(true);
    esp_rom_delay_us(50);
    uart_write_bytes(UART_PORT, (const char*)txbuf, txlen);
    uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(100));
    rs485_set_tx(false);

    frame_t fr;
    if (xQueueReceive(g_frame_q, &fr, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        if (fr.seq == seq) {
            if (reply) *reply = fr;
            return true;
        }
    }
    return false;
}

/* =========================
 * LÓGICA DE SINCRONIZAÇÃO
 * ========================= */

void user_change_param(uint8_t id, uint16_t new_val) {
    for (int i = 0; i < ACTIVE_PARAMS_COUNT; i++) {
        if (params[i].id == id) {
            params[i].value = new_val;
            params[i].pending_sync = true;
            printf("\n[MENU] %s alterado para %d. Pendente sync na parada.\n", params[i].name, new_val);
            return;
        }
    }
    printf("\n[ERRO] Parametro P%d nao encontrado.\n", id);
}

bool sync_params_to_mi(bool force_all) {
    uint8_t tx_payload[3];
    frame_t rep;
    for (int i = 0; i < ACTIVE_PARAMS_COUNT; i++) {
        if (force_all || params[i].pending_sync) {
            tx_payload[0] = params[i].id;
            tx_payload[1] = (uint8_t)(params[i].value >> 8);
            tx_payload[2] = (uint8_t)(params[i].value & 0xFF);
            if (!rs485_request(TYPE_WRITE_PARAM, tx_payload, 3, &rep, 200)) return false;
            params[i].pending_sync = false;
            printf("[OK] Sincronizado %s: %d\n", params[i].name, params[i].value);
        }
    }
    return true;
}

/* =========================
 * TASK DE COMUNICAÇÃO PRINCIPAL (LOOP REAL)
 * ========================= */
static void ihm_sync_task(void *arg) {
    frame_t rep;
    uint8_t tx_payload[9];
    vTaskDelay(pdMS_TO_TICKS(2000));

    printf("Iniciando Handshake...\n");
    while (!sync_params_to_mi(true)) vTaskDelay(pdMS_TO_TICKS(1000));
    printf("Handshake OK! Logs silenciados. 'MON' para ver.\n");

    while (1) {
        // --- MONTAR PAYLOAD (IHM -> MI) ---
        tx_payload[0] = g_control.buttons;
        tx_payload[1] = (uint8_t)(g_control.target_freq >> 8);
        tx_payload[2] = (uint8_t)(g_control.target_freq & 0xFF);
        tx_payload[3] = g_control.direction;
        tx_payload[4] = 0; // Reservado
        tx_payload[5] = 0; // Reservado
        tx_payload[6] = P90;
        tx_payload[7] = P91;
        tx_payload[8] = (uint8_t)E08;

        if (g_show_logs) {
            printf(">> [TX] BTN: 0x%02X | Alvo: %d | Dir: %d\n", 
                    g_control.buttons, g_control.target_freq, g_control.direction);
        }

        if (rs485_request(TYPE_READ_STATUS, tx_payload, 9, &rep, 100)) {
            if (P90 > 0) P90--;
            if (P90 <= P91) E08 = false;

            // --- PROCESSAR RESPOSTA (MI -> IHM) ---
            if (rep.len >= 9) {
                g_telemetry.current_freq = (rep.payload[0] << 8) | rep.payload[1];
                g_telemetry.i_out        = (rep.payload[2] << 8) | rep.payload[3];
                g_telemetry.v_bus        = (rep.payload[4] << 8) | rep.payload[5];
                g_telemetry.v_out        = (rep.payload[6] << 8) | rep.payload[7];
                g_telemetry.temp_igbt    = rep.payload[8];

                if (g_show_logs) {
                    printf("<< [RX] Freq: %d | Cur: %d | Bus: %d | Vout: %d | Temp: %d\n",
                           g_telemetry.current_freq, g_telemetry.i_out, 
                           g_telemetry.v_bus, g_telemetry.v_out, g_telemetry.temp_igbt);
                }
                
                // Handshake de parada se houver pendência
                bool needs_sync = false;
                for (int i = 0; i < ACTIVE_PARAMS_COUNT; i++) {
                    if (params[i].pending_sync) { needs_sync = true; break; }
                }
                if (needs_sync && g_telemetry.current_freq == 0) {
                    printf("\n*** Motor Parado (0 Hz). Sincronizando Pendencias... ***\n");
                    sync_params_to_mi(false);
                }
            }
        } else {
            if (P90 < 100) P90++;
            if (g_show_logs) printf("!! [ERRO] Timeout MI\n");
        }

        if (P90 > P91) E08 = true;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* =========================
 * TASK DE CONSOLE (SIMULAÇÃO)
 * ========================= */
static void serial_console_task(void *arg) {
    char line[48];
    int rx_pos = 0;
    setvbuf(stdin, NULL, _IONBF, 0);

    while (1) {
        int c = fgetc(stdin);
        if (c != EOF) {
            if (c == '\n' || c == '\r') {
                line[rx_pos] = '\0';
                if (rx_pos > 0) {
                    if (strcmp(line, "MON") == 0) { g_show_logs = true; printf("\n[LOGS ON]\n"); }
                    else if (strcmp(line, "SIL") == 0) { g_show_logs = false; printf("\n[LOGS OFF]\n"); }
                    else if (strncmp(line, "BT=", 3) == 0) {
                        g_control.buttons = (uint8_t)atoi(&line[3]);
                        printf("\n[CONTROLE] Botao definido para: 0x%02X\n", g_control.buttons);
                    }
                    else if (strncmp(line, "DIR=", 4) == 0) {
                        g_control.direction = (uint8_t)atoi(&line[4]);
                        printf("\n[CONTROLE] Sentido: %s\n", g_control.direction ? "REV" : "FWD");
                    }
                    else if (strncmp(line, "FR=", 3) == 0) {
                        g_control.target_freq = (uint16_t)atoi(&line[3]);
                        printf("\n[CONTROLE] Freq Alvo: %d\n", g_control.target_freq);
                    }
                    else {
                        uint8_t id; uint16_t val;
                        if (sscanf(line, "P%hhu=%hu", &id, &val) == 2) user_change_param(id, val);
                    }
                }
                rx_pos = 0;
            } else if (rx_pos < sizeof(line) - 1) {
                line[rx_pos++] = (char)c;
                fputc(c, stdout);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {
    nvs_flash_init();
    g_frame_q = xQueueCreate(10, sizeof(frame_t));
    gpio_config_t io = { .pin_bit_mask = 1ULL << RS485_EN_PIN, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&io);
    
    uart_config_t cfg = {
        .baud_rate = 115200, .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN, .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT
    };
    uart_driver_install(UART_PORT, 1024, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &cfg);
    uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    xTaskCreate(rx_task, "rs485_rx", 4096, NULL, 10, NULL);
    xTaskCreate(ihm_sync_task, "ihm_sync", 4096, NULL, 5, NULL);
    xTaskCreate(serial_console_task, "console", 4096, NULL, 3, NULL);
}
