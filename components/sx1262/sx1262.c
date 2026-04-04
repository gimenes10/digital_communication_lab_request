/**
 * @file sx1262.c
 * @brief Minimal SX1262 LoRa driver for Heltec WiFi LoRa 32 V3
 */

#include "sx1262.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "sx1262";

/* ── Internal helpers ────────────────────────────────────────────── */

static void sx1262_wait_busy(void)
{
    while (gpio_get_level(SX1262_PIN_BUSY)) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static esp_err_t sx1262_spi_cmd(sx1262_t *dev, const uint8_t *tx, uint8_t *rx, size_t len)
{
    spi_transaction_t t = {
        .length    = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    sx1262_wait_busy();
    return spi_device_transmit(dev->spi, &t);
}

static esp_err_t sx1262_write_command(sx1262_t *dev, uint8_t cmd,
                                       const uint8_t *params, uint8_t param_len)
{
    uint8_t tx[16] = {0};
    uint8_t rx[16] = {0};

    tx[0] = cmd;
    if (params && param_len > 0) {
        memcpy(&tx[1], params, param_len);
    }

    return sx1262_spi_cmd(dev, tx, rx, 1 + param_len);
}

static esp_err_t sx1262_read_command(sx1262_t *dev, uint8_t cmd,
                                      uint8_t *result, uint8_t result_len)
{
    /* cmd + NOP (status) + result bytes */
    uint8_t total = 2 + result_len;
    uint8_t tx[16] = {0};
    uint8_t rx[16] = {0};

    tx[0] = cmd;
    /* rest are NOP (0x00) */

    esp_err_t ret = sx1262_spi_cmd(dev, tx, rx, total);
    if (ret != ESP_OK) {
        return ret;
    }

    if (result && result_len > 0) {
        memcpy(result, &rx[2], result_len);
    }

    return ESP_OK;
}

static void sx1262_hw_reset(void)
{
    gpio_set_level(SX1262_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(SX1262_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
}

/* ── Public API ──────────────────────────────────────────────────── */

esp_err_t sx1262_init(sx1262_t *dev)
{
    esp_err_t ret;

    /* ── GPIO setup ──────────────────────────────────────────────── */
    gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << SX1262_PIN_RST) | (1ULL << SX1262_PIN_CS),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&out_cfg);
    gpio_set_level(SX1262_PIN_CS, 1);
    gpio_set_level(SX1262_PIN_RST, 1);

    gpio_config_t in_cfg = {
        .pin_bit_mask = (1ULL << SX1262_PIN_BUSY) | (1ULL << SX1262_PIN_DIO1),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&in_cfg);

    /* ── SPI bus init ────────────────────────────────────────────── */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num   = SX1262_PIN_MOSI,
        .miso_io_num   = SX1262_PIN_MISO,
        .sclk_io_num   = SX1262_PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 256 + 8,
    };

    ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 8 * 1000 * 1000,  /* 8 MHz */
        .mode           = 0,                  /* CPOL=0, CPHA=0 */
        .spics_io_num   = SX1262_PIN_CS,
        .queue_size     = 1,
    };

    ret = spi_bus_add_device(SPI2_HOST, &dev_cfg, &dev->spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI add device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* ── Hardware reset ──────────────────────────────────────────── */
    sx1262_hw_reset();
    sx1262_wait_busy();

    /* ── SetStandby(STDBY_RC) ────────────────────────────────────── */
    uint8_t standby_rc = 0x00;
    ret = sx1262_write_command(dev, SX_CMD_SET_STANDBY, &standby_rc, 1);
    if (ret != ESP_OK) goto err;

    /* ── SetDio3AsTcxoCtrl(1.8V, 5ms) ───────────────────────────── */
    /* Heltec V3 TCXO controlled by DIO3 at 1.8V */
    uint8_t tcxo_params[] = {
        0x02,       /* tcxoVoltage: 1.8V */
        0x00, 0x00, 0xC8  /* timeout: 200 * 15.625µs ≈ 3.125ms → round up */
    };
    ret = sx1262_write_command(dev, SX_CMD_SET_DIO3_AS_TCXO, tcxo_params, 4);
    if (ret != ESP_OK) goto err;
    vTaskDelay(pdMS_TO_TICKS(5));

    /* ── Calibrate all blocks ────────────────────────────────────── */
    uint8_t calib_param = 0x7F;
    ret = sx1262_write_command(dev, SX_CMD_CALIBRATE, &calib_param, 1);
    if (ret != ESP_OK) goto err;
    vTaskDelay(pdMS_TO_TICKS(10));

    /* ── SetStandby(STDBY_XOSC) ─────────────────────────────────── */
    uint8_t standby_xosc = 0x01;
    ret = sx1262_write_command(dev, SX_CMD_SET_STANDBY, &standby_xosc, 1);
    if (ret != ESP_OK) goto err;

    /* ── SetRegulatorMode(DC-DC) ─────────────────────────────────── */
    uint8_t regulator = 0x01;  /* DC-DC */
    ret = sx1262_write_command(dev, SX_CMD_SET_REGULATOR_MODE, &regulator, 1);
    if (ret != ESP_OK) goto err;

    /* ── SetDio2AsRfSwitchCtrl(enable) ───────────────────────────── */
    uint8_t dio2_enable = 0x01;
    ret = sx1262_write_command(dev, SX_CMD_SET_DIO2_AS_RF_SW, &dio2_enable, 1);
    if (ret != ESP_OK) goto err;

    /* ── SetPacketType(LoRa) ─────────────────────────────────────── */
    uint8_t pkt_type = 0x01;  /* LoRa */
    ret = sx1262_write_command(dev, SX_CMD_SET_PACKET_TYPE, &pkt_type, 1);
    if (ret != ESP_OK) goto err;

    /* ── SetRfFrequency(915 MHz) ─────────────────────────────────── */
    /* freq_reg = freq_hz * 2^25 / 32e6 = 959447040 = 0x39300000 */
    uint8_t freq_params[] = { 0x39, 0x30, 0x00, 0x00 };
    ret = sx1262_write_command(dev, SX_CMD_SET_RF_FREQUENCY, freq_params, 4);
    if (ret != ESP_OK) goto err;

    /* ── SetPaConfig (SX1262: +22 dBm capable) ───────────────────── */
    uint8_t pa_params[] = {
        0x04,  /* paDutyCycle */
        0x07,  /* hpMax */
        0x00,  /* deviceSel: SX1262 */
        0x01,  /* paLut */
    };
    ret = sx1262_write_command(dev, SX_CMD_SET_PA_CONFIG, pa_params, 4);
    if (ret != ESP_OK) goto err;

    /* ── SetTxParams(+22 dBm, 200µs ramp) ────────────────────────── */
    uint8_t tx_params[] = {
        0x16,  /* power: 22 dBm */
        0x04,  /* rampTime: 200µs */
    };
    ret = sx1262_write_command(dev, SX_CMD_SET_TX_PARAMS, tx_params, 2);
    if (ret != ESP_OK) goto err;

    /* ── SetBufferBaseAddress(TX=0x00, RX=0x80) ──────────────────── */
    uint8_t buf_base[] = { 0x00, 0x80 };
    ret = sx1262_write_command(dev, SX_CMD_SET_BUFFER_BASE_ADDR, buf_base, 2);
    if (ret != ESP_OK) goto err;

    /* ── SetModulationParams(SF7, BW125, CR4/5, LDRO off) ────────── */
    uint8_t mod_params[] = {
        LORA_SF,      /* spreadingFactor */
        LORA_BW_125K, /* bandwidth: 125 kHz */
        LORA_CR_4_5,  /* codingRate: 4/5 */
        0x00,         /* lowDataRateOptimize: off (SF7+BW125 doesn't need it) */
    };
    ret = sx1262_write_command(dev, SX_CMD_SET_MOD_PARAMS, mod_params, 4);
    if (ret != ESP_OK) goto err;

    /* ── SetPacketParams(preamble=8, explicit, len=4, CRC on, std IQ) */
    uint8_t pkt_params[] = {
        0x00, LORA_PREAMBLE_LEN,  /* preambleLength: 8 symbols */
        0x00,                      /* headerType: explicit */
        0xFF,                      /* payloadLength: max (updated per TX) */
        0x01,                      /* crcType: on */
        0x00,                      /* invertIQ: standard */
    };
    ret = sx1262_write_command(dev, SX_CMD_SET_PACKET_PARAMS, pkt_params, 6);
    if (ret != ESP_OK) goto err;

    /* ── SetDioIrqParams: TxDone+RxDone+Timeout on DIO1 ─────────── */
    uint16_t irq_mask = SX_IRQ_TX_DONE | SX_IRQ_RX_DONE | SX_IRQ_TIMEOUT | SX_IRQ_CRC_ERR;
    uint8_t irq_params[] = {
        (uint8_t)(irq_mask >> 8), (uint8_t)(irq_mask & 0xFF),  /* irqMask */
        (uint8_t)(irq_mask >> 8), (uint8_t)(irq_mask & 0xFF),  /* dio1Mask */
        0x00, 0x00,  /* dio2Mask: none */
        0x00, 0x00,  /* dio3Mask: none */
    };
    ret = sx1262_write_command(dev, SX_CMD_SET_DIO_IRQ_PARAMS, irq_params, 8);
    if (ret != ESP_OK) goto err;

    /* ── SetRxTxFallbackMode(STDBY_XOSC) ────────────────────────── */
    uint8_t fallback = 0x30;  /* STDBY_XOSC */
    ret = sx1262_write_command(dev, SX_CMD_SET_RX_TX_FALLBACK, &fallback, 1);
    if (ret != ESP_OK) goto err;

    /* ── Clear any pending IRQs ──────────────────────────────────── */
    sx1262_clear_irq(dev);

    ESP_LOGI(TAG, "SX1262 initialized: 915 MHz, SF%d, BW125, CR4/5, +%d dBm",
             LORA_SF, LORA_TX_POWER_DBM);
    return ESP_OK;

err:
    ESP_LOGE(TAG, "SX1262 init failed at step: %s", esp_err_to_name(ret));
    return ret;
}

esp_err_t sx1262_send_packet(sx1262_t *dev, const uint8_t *data, uint8_t len)
{
    esp_err_t ret;

    /* Update payload length in packet params */
    uint8_t pkt_params[] = {
        0x00, LORA_PREAMBLE_LEN,
        0x00,  /* explicit header */
        len,   /* payloadLength */
        0x01,  /* CRC on */
        0x00,  /* standard IQ */
    };
    ret = sx1262_write_command(dev, SX_CMD_SET_PACKET_PARAMS, pkt_params, 6);
    if (ret != ESP_OK) return ret;

    /* Write payload to TX buffer at offset 0x00 */
    uint8_t tx_buf[258] = {0};
    uint8_t rx_buf[258] = {0};
    tx_buf[0] = SX_CMD_WRITE_BUFFER;
    tx_buf[1] = 0x00;  /* offset */
    memcpy(&tx_buf[2], data, len);

    ret = sx1262_spi_cmd(dev, tx_buf, rx_buf, 2 + len);
    if (ret != ESP_OK) return ret;

    /* Clear IRQ and start TX */
    sx1262_clear_irq(dev);

    uint8_t tx_timeout[] = { 0x00, 0x00, 0x00 };  /* no timeout (single TX) */
    ret = sx1262_write_command(dev, SX_CMD_SET_TX, tx_timeout, 3);
    if (ret != ESP_OK) return ret;

    /* Poll DIO1 for TX done (max 5 seconds) */
    int64_t start = esp_timer_get_time();
    while (!gpio_get_level(SX1262_PIN_DIO1)) {
        if ((esp_timer_get_time() - start) > 5000000LL) {
            ESP_LOGE(TAG, "TX timeout");
            sx1262_set_standby(dev);
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    uint16_t irq = sx1262_get_irq(dev);
    sx1262_clear_irq(dev);

    if (irq & SX_IRQ_TX_DONE) {
        ESP_LOGD(TAG, "TX done (%d bytes)", len);
        return ESP_OK;
    }

    ESP_LOGE(TAG, "TX failed, IRQ=0x%04X", irq);
    return ESP_FAIL;
}

esp_err_t sx1262_start_rx(sx1262_t *dev, uint32_t timeout_ms)
{
    sx1262_clear_irq(dev);

    uint32_t timeout_ticks;
    if (timeout_ms == 0) {
        /* Continuous RX */
        timeout_ticks = 0x00FFFFFF;
    } else {
        /* Each tick = 15.625 µs → ticks = ms * 1000 / 15.625 = ms * 64 */
        timeout_ticks = timeout_ms * 64;
        if (timeout_ticks > 0x00FFFFFE) {
            timeout_ticks = 0x00FFFFFE;
        }
    }

    uint8_t rx_params[] = {
        (uint8_t)((timeout_ticks >> 16) & 0xFF),
        (uint8_t)((timeout_ticks >> 8) & 0xFF),
        (uint8_t)(timeout_ticks & 0xFF),
    };

    return sx1262_write_command(dev, SX_CMD_SET_RX, rx_params, 3);
}

esp_err_t sx1262_receive_packet(sx1262_t *dev, uint32_t timeout_ms)
{
    esp_err_t ret = sx1262_start_rx(dev, timeout_ms);
    if (ret != ESP_OK) return ret;

    /* Poll DIO1 for RX done or timeout */
    uint32_t poll_limit_ms = (timeout_ms > 0) ? (timeout_ms + 1000) : 60000;
    int64_t start = esp_timer_get_time();

    while (!gpio_get_level(SX1262_PIN_DIO1)) {
        if ((esp_timer_get_time() - start) > (int64_t)poll_limit_ms * 1000LL) {
            sx1262_set_standby(dev);
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    uint16_t irq = sx1262_get_irq(dev);
    sx1262_clear_irq(dev);

    if (irq & SX_IRQ_CRC_ERR) {
        ESP_LOGW(TAG, "RX CRC error");
        return ESP_ERR_INVALID_CRC;
    }

    if (irq & SX_IRQ_RX_DONE) {
        return ESP_OK;
    }

    if (irq & SX_IRQ_TIMEOUT) {
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGW(TAG, "RX unexpected IRQ: 0x%04X", irq);
    return ESP_FAIL;
}

esp_err_t sx1262_read_packet(sx1262_t *dev, uint8_t *data, uint8_t *len, int16_t *rssi)
{
    esp_err_t ret;

    /* GetRxBufferStatus → payloadLength, rxStartBufferPointer */
    uint8_t rx_status[2] = {0};
    ret = sx1262_read_command(dev, SX_CMD_GET_RX_BUFFER_STATUS, rx_status, 2);
    if (ret != ESP_OK) return ret;

    uint8_t payload_len = rx_status[0];
    uint8_t rx_offset   = rx_status[1];

    if (payload_len == 0) {
        *len = 0;
        return ESP_ERR_INVALID_SIZE;
    }

    /* ReadBuffer at offset */
    uint8_t tx_buf[260] = {0};
    uint8_t rx_buf[260] = {0};
    tx_buf[0] = SX_CMD_READ_BUFFER;
    tx_buf[1] = rx_offset;
    tx_buf[2] = 0x00;  /* NOP */

    ret = sx1262_spi_cmd(dev, tx_buf, rx_buf, 3 + payload_len);
    if (ret != ESP_OK) return ret;

    memcpy(data, &rx_buf[3], payload_len);
    *len = payload_len;

    /* GetPacketStatus → rssi */
    if (rssi) {
        uint8_t pkt_status[3] = {0};
        ret = sx1262_read_command(dev, SX_CMD_GET_PACKET_STATUS, pkt_status, 3);
        if (ret == ESP_OK) {
            /* rssiPkt = -pkt_status[0] / 2 */
            *rssi = -(int16_t)pkt_status[0] / 2;
        }
    }

    return ESP_OK;
}

uint16_t sx1262_get_irq(sx1262_t *dev)
{
    uint8_t irq_buf[2] = {0};
    sx1262_read_command(dev, SX_CMD_GET_IRQ_STATUS, irq_buf, 2);
    return ((uint16_t)irq_buf[0] << 8) | irq_buf[1];
}

void sx1262_clear_irq(sx1262_t *dev)
{
    uint8_t clear[] = { 0xFF, 0xFF };
    sx1262_write_command(dev, SX_CMD_CLR_IRQ_STATUS, clear, 2);
}

esp_err_t sx1262_set_standby(sx1262_t *dev)
{
    uint8_t mode = 0x01;  /* STDBY_XOSC */
    return sx1262_write_command(dev, SX_CMD_SET_STANDBY, &mode, 1);
}
