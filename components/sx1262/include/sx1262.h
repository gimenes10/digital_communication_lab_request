/**
 * @file sx1262.h
 * @brief Minimal SX1262 LoRa driver for Heltec WiFi LoRa 32 V3
 *
 * Pin mapping (Heltec V3 board):
 *   MOSI=GPIO10, MISO=GPIO11, SCK=GPIO9, CS=GPIO8
 *   RST=GPIO12, BUSY=GPIO13, DIO1=GPIO14
 *
 * TCXO controlled via DIO3 (1.8V), RF switch via DIO2.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"

/* ── Heltec WiFi LoRa 32 V3 pin definitions ─────────────────────── */
#define SX1262_PIN_MOSI   10
#define SX1262_PIN_MISO   11
#define SX1262_PIN_SCK     9
#define SX1262_PIN_CS      8
#define SX1262_PIN_RST    12
#define SX1262_PIN_BUSY   13
#define SX1262_PIN_DIO1   14

/* ── LoRa default parameters ─────────────────────────────────────── */
#define LORA_FREQUENCY_HZ   915000000UL   /* 915 MHz */
#define LORA_SF             7
#define LORA_BW_125K        0x04
#define LORA_CR_4_5         0x01
#define LORA_PREAMBLE_LEN   8
#define LORA_TX_POWER_DBM   22
#define LORA_MAX_PAYLOAD     255

/* ── SX1262 opcode commands ──────────────────────────────────────── */
#define SX_CMD_SET_SLEEP              0x84
#define SX_CMD_SET_STANDBY            0x80
#define SX_CMD_SET_TX                 0x83
#define SX_CMD_SET_RX                 0x82
#define SX_CMD_SET_PACKET_TYPE        0x8A
#define SX_CMD_SET_RF_FREQUENCY       0x86
#define SX_CMD_SET_PA_CONFIG          0x95
#define SX_CMD_SET_TX_PARAMS          0x8E
#define SX_CMD_SET_BUFFER_BASE_ADDR   0x8F
#define SX_CMD_SET_MOD_PARAMS         0x8B
#define SX_CMD_SET_PACKET_PARAMS      0x8C
#define SX_CMD_SET_DIO_IRQ_PARAMS     0x08
#define SX_CMD_GET_IRQ_STATUS         0x12
#define SX_CMD_CLR_IRQ_STATUS         0x02
#define SX_CMD_WRITE_BUFFER           0x0E
#define SX_CMD_READ_BUFFER            0x1E
#define SX_CMD_GET_RX_BUFFER_STATUS   0x13
#define SX_CMD_GET_PACKET_STATUS      0x14
#define SX_CMD_SET_DIO3_AS_TCXO       0x97
#define SX_CMD_CALIBRATE              0x89
#define SX_CMD_SET_REGULATOR_MODE     0x96
#define SX_CMD_SET_DIO2_AS_RF_SW      0x9D
#define SX_CMD_SET_RX_TX_FALLBACK     0x93

/* ── IRQ flags ───────────────────────────────────────────────────── */
#define SX_IRQ_TX_DONE      (1 << 0)
#define SX_IRQ_RX_DONE      (1 << 1)
#define SX_IRQ_PREAMBLE     (1 << 2)
#define SX_IRQ_SYNC_WORD    (1 << 3)
#define SX_IRQ_HEADER_VALID (1 << 4)
#define SX_IRQ_HEADER_ERR   (1 << 5)
#define SX_IRQ_CRC_ERR      (1 << 6)
#define SX_IRQ_CAD_DONE     (1 << 7)
#define SX_IRQ_CAD_DETECTED (1 << 8)
#define SX_IRQ_TIMEOUT      (1 << 9)

/* ── Device handle ───────────────────────────────────────────────── */
typedef struct {
    spi_device_handle_t spi;
} sx1262_t;

/**
 * @brief Initialize SX1262 for LoRa operation on 915 MHz.
 *        Configures SPI, resets chip, sets up TCXO, PA, modulation, etc.
 */
esp_err_t sx1262_init(sx1262_t *dev);

/**
 * @brief Send a LoRa packet (blocking until TX done or timeout).
 * @param data  Payload bytes
 * @param len   Payload length (max 255)
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if TX did not complete
 */
esp_err_t sx1262_send_packet(sx1262_t *dev, const uint8_t *data, uint8_t len);

/**
 * @brief Enter RX mode and block until a packet is received or timeout.
 * @param timeout_ms  RX timeout in milliseconds (0 = continuous/no timeout)
 * @return ESP_OK if packet received, ESP_ERR_TIMEOUT on timeout
 */
esp_err_t sx1262_receive_packet(sx1262_t *dev, uint32_t timeout_ms);

/**
 * @brief Read the last received packet from the SX1262 buffer.
 * @param[out] data  Buffer to store payload
 * @param[out] len   Received payload length
 * @param[out] rssi  RSSI of received packet (can be NULL)
 */
esp_err_t sx1262_read_packet(sx1262_t *dev, uint8_t *data, uint8_t *len, int16_t *rssi);

/**
 * @brief Start RX mode (non-blocking). Use sx1262_get_irq() to poll.
 */
esp_err_t sx1262_start_rx(sx1262_t *dev, uint32_t timeout_ms);

/**
 * @brief Get current IRQ flags.
 */
uint16_t sx1262_get_irq(sx1262_t *dev);

/**
 * @brief Clear all IRQ flags.
 */
void sx1262_clear_irq(sx1262_t *dev);

/**
 * @brief Set SX1262 to standby mode.
 */
esp_err_t sx1262_set_standby(sx1262_t *dev);

