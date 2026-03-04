#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define UART_PORT       UART_NUM_1
#define UART_TX_PIN     17
#define UART_RX_PIN     16
#define RS485_DE_PIN    4

#define BAUDRATE        115200
#define RX_BUF_SIZE     1024

#define SOF1 0xAA
#define SOF2 0x55
#define VER  0x01

#define SRC_ID 0x01
#define DST_ID 0x10

#define ACK_TIMEOUT_MS  150
#define N_RETRY         3

#define WIN_SIZE        100
#define LOSS_TOL        0.02f   // 2%

typedef struct {
    uint32_t tx_total;
    uint32_t ack_ok;
    uint32_t ack_timeout;
    uint32_t rx_crc_err;
    uint32_t rx_frame_err;

    uint32_t win_tx;
    uint32_t win_lost;
    float tol_loss;
    uint32_t win_size;

    bool e08;
} comm_stats_t;

static comm_stats_t st = {
    .tol_loss = LOSS_TOL,
    .win_size = WIN_SIZE,
    .e08 = false
};

static uint8_t g_seq = 0;

// CRC16-IBM/Modbus (polinômio 0xA001)
static uint16_t crc16_ibm(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
        }
    }
    return crc;
}

static inline void rs485_set_tx(bool tx) {
    gpio_set_level(RS485_DE_PIN, tx ? 1 : 0);
}

// Frame: [AA 55][VER SRC DST TYPE SEQ LEN][PAYLOAD][CRC_L CRC_H]
static size_t build_frame(uint8_t *out, uint8_t src, uint8_t dst, uint8_t type, uint8_t seq,
                          const uint8_t *payload, uint8_t len) {
    out[0] = SOF1;
    out[1] = SOF2;
    out[2] = VER;
    out[3] = src;
    out[4] = dst;
    out[5] = type;
    out[6] = seq;
    out[7] = len;
    if (len && payload) memcpy(&out[8], payload, len);

    // CRC em VER..LEN + payload => (6 bytes header sem SOF) + len
    uint16_t crc = crc16_ibm(&out[2], (size_t)(6 + len));
    out[8 + len]     = (uint8_t)(crc & 0xFF);
    out[8 + len + 1] = (uint8_t)(crc >> 8);

    return (size_t)(8 + len + 2);
}

typedef struct {
    uint8_t ver, src, dst, type, seq, len;
    uint8_t payload[255];
} parsed_t;

// Parser “procura SOF” + valida CRC; retorna 1 frame por vez
static bool try_parse_one(uint8_t *buf, size_t n, parsed_t *p, size_t *consumed) {
    *consumed = 0;
    if (n < 10) return false;

    size_t i = 0;
    while (i + 1 < n) {
        if (buf[i] == SOF1 && buf[i+1] == SOF2) break;
        i++;
    }
    if (i + 1 >= n) { *consumed = n; return false; }
    if (n - i < 10) { *consumed = i; return false; }

    uint8_t len = buf[i + 7];
    size_t total = 8 + len + 2;
    if (n - i < total) { *consumed = i; return false; }

    uint16_t crc_rx = (uint16_t)buf[i + 8 + len] | ((uint16_t)buf[i + 8 + len + 1] << 8);
    uint16_t crc_ok = crc16_ibm(&buf[i + 2], (size_t)(6 + len));
    if (crc_rx != crc_ok) {
        st.rx_crc_err++;
        *consumed = i + total;
        return false;
    }

    p->ver  = buf[i+2];
    p->src  = buf[i+3];
    p->dst  = buf[i+4];
    p->type = buf[i+5];
    p->seq  = buf[i+6];
    p->len  = len;
    if (len) memcpy(p->payload, &buf[i+8], len);

    *consumed = i + total;
    return true;
}

static bool wait_ack(uint8_t expect_seq, uint8_t expect_type) {
    int64_t t0 = esp_timer_get_time();

    uint8_t tmp[256];
    uint8_t acc[512];
    size_t acc_n = 0;

    while ((esp_timer_get_time() - t0) < (int64_t)ACK_TIMEOUT_MS * 1000) {
        int rd = uart_read_bytes(UART_PORT, tmp, sizeof(tmp), pdMS_TO_TICKS(10));
        if (rd > 0) {
            if (acc_n + (size_t)rd > sizeof(acc)) acc_n = 0; // simples
            memcpy(&acc[acc_n], tmp, (size_t)rd);
            acc_n += (size_t)rd;

            while (acc_n) {
                parsed_t f;
                size_t consumed = 0;
                bool ok = try_parse_one(acc, acc_n, &f, &consumed);
                if (consumed == 0) break;

                memmove(acc, &acc[consumed], acc_n - consumed);
                acc_n -= consumed;

                if (!ok) { st.rx_frame_err++; continue; }

                if (f.dst == SRC_ID && f.src == DST_ID &&
                    f.seq == expect_seq && f.type == (uint8_t)(0x80 | expect_type)) {
                    return true;
                }
            }
        }
    }
    return false;
}

static void update_window_and_e08(void) {
    if (st.win_tx >= st.win_size) {
        float loss = (st.win_tx > 0) ? ((float)st.win_lost / (float)st.win_tx) : 0.0f;
        st.win_tx = 0;
        st.win_lost = 0;
        if (loss > st.tol_loss) st.e08 = true;
    }
}

// API: envia comando e retorna sucesso. st.e08 indica erro de comunicação.
bool rs485_send_cmd(uint8_t type, const uint8_t *payload, uint8_t len) {
    uint8_t seq = g_seq++;
    uint8_t pkt[8 + 255 + 2];
    size_t pkt_n = build_frame(pkt, SRC_ID, DST_ID, type, seq, payload, len);

    st.tx_total++;
    st.win_tx++;

    for (int attempt = 0; attempt < N_RETRY; attempt++) {
        rs485_set_tx(true);
        uart_write_bytes(UART_PORT, (const char*)pkt, pkt_n);
        uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(50));
        rs485_set_tx(false);

        if (wait_ack(seq, type)) {
            st.ack_ok++;
            update_window_and_e08();
            return true;
        }
    }

    st.ack_timeout++;
    st.win_lost++;
    st.e08 = true;           // falha imediata também ativa E08
    update_window_and_e08();
    return false;
}

void rs485_init(void) {
    uart_config_t cfg = {
        .baud_rate = BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(UART_PORT, RX_BUF_SIZE, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &cfg);
    uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << RS485_DE_PIN,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    rs485_set_tx(false);
}
