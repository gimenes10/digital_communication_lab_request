/**
 * @file  main.c
 * @brief LoRa Gateway (Heltec #2) — Requester
 *
 * NÓ GATEWAY DO PIPELINE END-TO-END:
 *
 *   ┌──────────────┐  LoRa req   ┌──────────────┐  LoRa resp  ┌──────────────┐  UART  ┌──────┐
 *   │  ESTE NÓ     │ ──────────> │  Sensor Node │ ──────────> │   ESTE NÓ    │ ─────> │ FPGA │
 *   │ (requester)  │             │  + BH1750    │             │ (re-recv)    │        │ MIPS │
 *   └──────────────┘             └──────────────┘             └──────────────┘        └──────┘
 *           ▲                                                          │
 *           │                                                          │
 *           └─── botão PRG dispara request ────────────────────────────┘
 *                (futuramente substituído por trigger automático
 *                 em intervalo regular ou comando do FPGA via UART)
 *
 * Comportamento:
 *   1. Inicializa OLED e SX1262
 *   2. Configura GPIO0 (botão PRG) como interrupção por borda de descida
 *   3. Aguarda pressionamento via semáforo binário (ISR → task)
 *   4. Ao pressionar:
 *        a. Aplica debounce de 200ms
 *        b. Transmite [0xBB, 0x01] via LoRa
 *        c. Entra em RX com timeout de 5s aguardando resposta
 *        d. Valida tamanho, header e checksum XOR
 *        e. Extrai lux (uint16) e mostra no OLED + log serial
 *   5. Volta a aguardar próximo pressionamento
 *
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
#include "sx1262.h"
#include "ssd1306.h"

static const char *TAG = "gateway";

/* ── Protocolo Request/Response ──────────────────────────────────── */
/* Formato idêntico ao do nó sensor — protocolo agnóstico ao sensor   */
/* físico do outro lado (LDR, BH1750, qualquer um cabe em uint16).    */
#define HEADER_REQUEST   0xBB    /* Byte 0 do request                  */
#define CMD_READ_SENSOR  0x01    /* Byte 1: comando "ler sensor"       */
#define HEADER_RESPONSE  0xAA    /* Byte 0 esperado na resposta        */
#define RESPONSE_LEN     4       /* [HEADER, HIGH, LOW, XOR]           */

/* ── Hardware ────────────────────────────────────────────────────── */
#define BUTTON_GPIO      GPIO_NUM_0   /* Botão PRG do Heltec V3        */
#define RX_TIMEOUT_MS    5000         /* Sensor deve responder em <5s  */

/* ── Sincronização ISR → Task ────────────────────────────────────── */
/* Semáforo binário: a ISR dá give() a cada borda, a task espera      */
/* indefinidamente em take(). Evita polling do GPIO no loop principal. */
static SemaphoreHandle_t s_btn_sem;

/**
 * @brief ISR do botão PRG (GPIO0).
 *
 * Atributo IRAM_ATTR é obrigatório: ISRs são executadas com cache de
 * flash potencialmente desativado, então precisam estar em IRAM.
 *
 * O semáforo é dado de dentro da ISR via xSemaphoreGiveFromISR para
 * acordar a task. Se o give() resultou em uma task de prioridade
 * maior que a interrompida, portYIELD_FROM_ISR força um context
 * switch ao retornar, ao invés de esperar o próximo tick.
 */
static void IRAM_ATTR button_isr_handler(void *arg)
{
    BaseType_t higher_prio_woken = pdFALSE;
    xSemaphoreGiveFromISR(s_btn_sem, &higher_prio_woken);
    portYIELD_FROM_ISR(higher_prio_woken);
}

/**
 * @brief Configura GPIO0 como entrada com pull-up + IRQ neg-edge.
 *
 * O botão PRG é ativo-baixo: pull-up interno mantém HIGH em idle,
 * e o pressionamento força LOW. Por isso a interrupção é por borda
 * de descida (NEGEDGE).
 *
 * gpio_install_isr_service(0):
 *   - O argumento 0 indica que não queremos flags especiais (ESP_INTR_FLAG_*)
 *   - Pode retornar ESP_ERR_INVALID_STATE se outra parte do código já
 *     instalou o serviço — não é erro, ignoramos.
 */
static esp_err_t button_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };

    esp_err_t ret = gpio_config(&cfg);
    if (ret != ESP_OK) return ret;

    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret;

    return gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL);
}

/**
 * @brief Task principal do gateway LoRa.
 *
 * Roda no Core 1 (deixa Core 0 para o stack WiFi/BT, se ativados).
 * Stack de 8KB acomoda o framebuffer do OLED (1KB), buffers SPI do
 * SX1262 e variáveis locais com folga.
 *
 * O loop é totalmente event-driven: bloqueia em xSemaphoreTake até
 * o botão ser pressionado. Sem polling = sem CPU desperdiçada.
 */
static void gateway_task(void *arg)
{
    sx1262_t   radio;
    ssd1306_t *oled = NULL;
    esp_err_t  ret;

    /* ── Inicialização do display ────────────────────────────────── */
    /* OLED é opcional — se falhar, continua só com log serial */
    ret = ssd1306_init(&oled);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "OLED init falhou: %s (continuando sem display)",
                 esp_err_to_name(ret));
    } else {
        ssd1306_clear(oled);
        ssd1306_draw_string(oled, 0, 0, "LoRa Gateway");
        ssd1306_draw_string(oled, 2, 0, "Aguardando...");
        ssd1306_update(oled);
    }

    /* ── Inicialização do rádio LoRa ─────────────────────────────── */
    ret = sx1262_init(&radio);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao inicializar SX1262");
        vTaskDelete(NULL);   /* Sem rádio não há projeto */
        return;
    }

    ESP_LOGI(TAG, "=== LoRa Gateway (Requester) ===");
    ESP_LOGI(TAG, "Pressione PRG (GPIO0) para solicitar leitura do sensor");

    if (oled) {
        ssd1306_clear(oled);
        ssd1306_draw_string(oled, 0, 0, "LoRa Gateway");
        ssd1306_draw_string(oled, 2, 0, "Pressione PRG");
        ssd1306_draw_string(oled, 3, 0, "para ler sensor");
        ssd1306_update(oled);
    }

    /* ── Loop principal: aguarda botão → request → response → display ── */
    while (1) {

        /* Bloqueia até a ISR sinalizar que o botão foi pressionado.
           portMAX_DELAY = espera infinita; sem CPU consumida. */
        if (xSemaphoreTake(s_btn_sem, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        /* ── Debounce ────────────────────────────────────────────── */
        /* Botões mecânicos geram múltiplas bordas em poucos ms.
           Esperamos 200ms e descartamos qualquer give() acumulado
           durante esse intervalo, garantindo "1 press = 1 request". */
        vTaskDelay(pdMS_TO_TICKS(200));
        xSemaphoreTake(s_btn_sem, 0);

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

        /* ── Aguarda resposta com timeout ────────────────────────── */
        /* Após o TX, o SX1262 muda para RX. O nó sensor tem um delay
           de 50ms antes de responder, então damos margem confortável. */
        ESP_LOGI(TAG, "Aguardando resposta (timeout %d ms)...", RX_TIMEOUT_MS);

        ret = sx1262_receive_packet(&radio, RX_TIMEOUT_MS);
        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "Timeout: sensor nao respondeu");
            if (oled) {
                ssd1306_clear(oled);
                ssd1306_draw_string(oled, 0, 0, "LoRa Gateway");
                ssd1306_draw_string(oled, 2, 0, "TIMEOUT");
                ssd1306_draw_string(oled, 3, 0, "sem resposta");
                ssd1306_update(oled);
            }
            continue;
        }
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Erro na recepcao: %s", esp_err_to_name(ret));
            continue;
        }

        /* ── Lê o pacote e captura RSSI ──────────────────────────── */
        uint8_t rx_data[LORA_MAX_PAYLOAD];
        uint8_t rx_len = 0;
        int16_t rssi   = 0;

        ret = sx1262_read_packet(&radio, rx_data, &rx_len, &rssi);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Erro ao ler pacote: %s", esp_err_to_name(ret));
            continue;
        }

        ESP_LOGI(TAG, "Pacote recebido (%d bytes, RSSI: %d dBm)", rx_len, rssi);

        /* ── Validação tripla: tamanho, header, checksum ─────────── */
        /* Validações em sequência protegem contra: pacotes corrompidos
           que passaram do CRC LoRa, ruído de RF que coincida com algum
           pacote válido, ou outros devices LoRa na mesma frequência. */

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

        uint8_t data_high = rx_data[1];
        uint8_t data_low  = rx_data[2];
        uint8_t checksum  = rx_data[3];
        uint8_t calc_xor  = data_high ^ data_low;

        if (checksum != calc_xor) {
            ESP_LOGW(TAG, "Checksum invalido: recebido=0x%02X, calculado=0x%02X",
                     checksum, calc_xor);
            continue;
        }

        /* ── Reconstrói o valor de lux do sensor BH1750 ──────────── */
        /* O nó sensor já fez a conversão raw / 1.2 antes de transmitir,
           então o valor aqui é diretamente em lux (0 ~ 54612). */
        uint16_t lux = ((uint16_t)data_high << 8) | data_low;

        ESP_LOGI(TAG, ">>> Luminosidade (BH1750): %u lux  (0x%04X)", lux, lux);
        ESP_LOGI(TAG, "    Pacote: [0x%02X 0x%02X 0x%02X 0x%02X] | RSSI: %d dBm",
                 rx_data[0], rx_data[1], rx_data[2], rx_data[3], rssi);

        /* ── Atualiza display com leitura recebida ───────────────── */
        if (oled) {
            char line[22];
            ssd1306_clear(oled);
            ssd1306_draw_string(oled, 0, 0, "LoRa Gateway");
            snprintf(line, sizeof(line), "Lux: %u", lux);
            ssd1306_draw_string(oled, 2, 0, line);
            snprintf(line, sizeof(line), "RSSI: %d dBm", rssi);
            ssd1306_draw_string(oled, 3, 0, line);
            snprintf(line, sizeof(line), "[%02X %02X %02X %02X]",
                     rx_data[0], rx_data[1], rx_data[2], rx_data[3]);
            ssd1306_draw_string(oled, 5, 0, line);
            ssd1306_update(oled);
        }

        /*
         * TODO (próxima fase do roadmap):
         * Encaminhar via UART para o FPGA o pacote validado:
         *   uart_write_bytes(UART_NUM, rx_data, RESPONSE_LEN);
         * O programa MIPS no FPGA, rodando sobre o SO preemptivo,
         * vai ler dos endereços 1018 (status) e 1019 (data) já
         * mapeados pelo UARTController.v.
         */
    }
}

/**
 * @brief Entry point da aplicação.
 *
 * Ordem de inicialização:
 *  1. Cria o semáforo binário (não pode ser usado antes de criado)
 *  2. Configura o botão e a ISR (já podemos receber presses)
 *  3. Lança a task que consome do semáforo
 *
 * Stack de 8KB é generoso para:
 *  - Framebuffer OLED (~1KB)
 *  - Buffers SPI do SX1262 (~512B)
 *  - Strings de log e printf (~512B)
 *  - Margem para chamadas aninhadas (~6KB)
 */
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
