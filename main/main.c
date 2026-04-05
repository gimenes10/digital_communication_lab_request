/**
 * @file main.c
 * @brief LoRa Gateway (Heltec #2) — Requester
 *
 * Simula o papel do FPGA: ao pressionar o botão PRG (GPIO0),
 * envia um pacote LoRa de requisição (0xBB 0x01) e aguarda
 * a resposta do nó sensor com o valor do LDR.
 *
 * Protocolo (conforme especificação do projeto):
 *   Request: [0xBB, 0x01]
 *   Response: [0xAA, DATA_HIGH, DATA_LOW, XOR(HIGH,LOW)]
 *
 * Plataforma: Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262)
 * Framework:  ESP-IDF v6.0
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_random.h"
#include "sx1262.h"
#include "ssd1306.h"

static const char *TAG = "gateway";

/* ── Protocolo ───────────────────────────────────────────────────── */
#define HEADER_REQUEST   0xBB
#define CMD_READ_SENSOR  0x01
#define HEADER_RESPONSE  0xAA
#define RESPONSE_LEN     4

/* ── Hardware ────────────────────────────────────────────────────── */
#define BUTTON_GPIO      GPIO_NUM_0   /* PRG button no Heltec V3 */
#define RX_TIMEOUT_MS    5000         /* Timeout para resposta do sensor */

/* ── Semáforo para ISR do botão ──────────────────────────────────── */
static SemaphoreHandle_t s_btn_sem;

static void IRAM_ATTR button_isr_handler(void *arg)
{
    BaseType_t higher_prio_woken = pdFALSE;
    xSemaphoreGiveFromISR(s_btn_sem, &higher_prio_woken);
    portYIELD_FROM_ISR(higher_prio_woken);
}

static esp_err_t button_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,  /* Ativo-baixo (pressionado = LOW) */
    };

    esp_err_t ret = gpio_config(&cfg);
    if (ret != ESP_OK) return ret;

    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret;

    return gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL);
}

/* ── Task principal ──────────────────────────────────────────────── */
static void gateway_task(void *arg)
{
    sx1262_t radio;
    ssd1306_t *oled = NULL;
    esp_err_t ret;

    /* Inicializa OLED */
    ret = ssd1306_init(&oled);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "OLED init falhou: %s (continuando sem display)", esp_err_to_name(ret));
    } else {
        ssd1306_clear(oled);
        ssd1306_draw_string(oled, 0, 0, "LoRa Gateway");
        ssd1306_draw_string(oled, 2, 0, "Aguardando...");
        ssd1306_update(oled);
    }

    ret = sx1262_init(&radio);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao inicializar SX1262");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "=== LoRa Gateway (Requester) ===");
    ESP_LOGI(TAG, "Pressione o botao PRG (GPIO0) para solicitar leitura");

    if (oled) {
        ssd1306_clear(oled);
        ssd1306_draw_string(oled, 0, 0, "LoRa Gateway");
        ssd1306_draw_string(oled, 2, 0, "Pressione PRG");
        ssd1306_draw_string(oled, 3, 0, "para ler sensor");
        ssd1306_update(oled);
    }

    while (1) {
        /* Aguarda pressionamento do botão */
        if (xSemaphoreTake(s_btn_sem, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        /* Debounce simples */
        vTaskDelay(pdMS_TO_TICKS(200));
        xSemaphoreTake(s_btn_sem, 0);  /* Limpa presses extras */

        ESP_LOGI(TAG, "---------------------------------------");
        ESP_LOGI(TAG, "Botao pressionado! Enviando request...");

        /* ── Monta e envia pacote de requisição ──────────────────── */
        uint8_t request[] = { HEADER_REQUEST, CMD_READ_SENSOR };

        ret = sx1262_send_packet(&radio, request, sizeof(request));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Falha ao enviar request: %s", esp_err_to_name(ret));
            continue;
        }
        ESP_LOGI(TAG, "Request enviado [0x%02X 0x%02X]", request[0], request[1]);

        /* ── Aguarda resposta do sensor ──────────────────────────── */
        ESP_LOGI(TAG, "Aguardando resposta (timeout %d ms)...", RX_TIMEOUT_MS);

        ret = sx1262_receive_packet(&radio, RX_TIMEOUT_MS);
        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "Timeout: sensor nao respondeu");
            continue;
        }
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Erro na recepcao: %s", esp_err_to_name(ret));
            continue;
        }

        /* ── Lê e valida pacote de resposta ──────────────────────── */
        uint8_t rx_data[LORA_MAX_PAYLOAD];
        uint8_t rx_len = 0;
        int16_t rssi = 0;

        ret = sx1262_read_packet(&radio, rx_data, &rx_len, &rssi);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Erro ao ler pacote: %s", esp_err_to_name(ret));
            continue;
        }

        ESP_LOGI(TAG, "Pacote recebido (%d bytes, RSSI: %d dBm)", rx_len, rssi);

        /* Validação do formato */
        if (rx_len != RESPONSE_LEN) {
            ESP_LOGW(TAG, "Tamanho invalido: esperado %d, recebido %d",
                     RESPONSE_LEN, rx_len);
            continue;
        }

        if (rx_data[0] != HEADER_RESPONSE) {
            ESP_LOGW(TAG, "Header invalido: 0x%02X (esperado 0x%02X)",
                     rx_data[0], HEADER_RESPONSE);
            continue;
        }

        /* Validação do checksum XOR */
        uint8_t data_high = rx_data[1];
        uint8_t data_low  = rx_data[2];
        uint8_t checksum  = rx_data[3];
        uint8_t calc_xor  = data_high ^ data_low;

        if (checksum != calc_xor) {
            ESP_LOGW(TAG, "Checksum invalido: recebido=0x%02X, calculado=0x%02X",
                     checksum, calc_xor);
            continue;
        }

        /* Extrai valor ADC de 12 bits */
        uint16_t adc_value = ((uint16_t)data_high << 8) | data_low;

        ESP_LOGI(TAG, ">>> Valor do sensor (LDR): %u  (0x%04X)", adc_value, adc_value);
        ESP_LOGI(TAG, "    Pacote: [0x%02X 0x%02X 0x%02X 0x%02X] | RSSI: %d dBm",
                 rx_data[0], rx_data[1], rx_data[2], rx_data[3], rssi);

        /* Atualiza OLED com dados recebidos */
        if (oled) {
            char line[22];
            ssd1306_clear(oled);
            ssd1306_draw_string(oled, 0, 0, "LoRa Gateway");
            snprintf(line, sizeof(line), "LDR: %u", adc_value);
            ssd1306_draw_string(oled, 2, 0, line);
            snprintf(line, sizeof(line), "RSSI: %d dBm", rssi);
            ssd1306_draw_string(oled, 3, 0, line);
            snprintf(line, sizeof(line), "[%02X %02X %02X %02X]",
                     rx_data[0], rx_data[1], rx_data[2], rx_data[3]);
            ssd1306_draw_string(oled, 5, 0, line);
            ssd1306_update(oled);
        }

        /*
         * No sistema final, aqui o gateway repassaria via UART para o FPGA:
         *   UART TX → [0xAA, DATA_HIGH, DATA_LOW, CHECKSUM]
         */
    }
}

void app_main(void)
{
    s_btn_sem = xSemaphoreCreateBinary();

    esp_err_t ret = button_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao configurar botao: %s", esp_err_to_name(ret));
        return;
    }

    xTaskCreatePinnedToCore(gateway_task, "gateway", 8192, NULL, 5, NULL, 1);
}
