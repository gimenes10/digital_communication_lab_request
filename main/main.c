/**
 * @file  main.c
 * @brief LoRa Gateway (Heltec #2) — Requester com forwarding UART para FPGA
 *
 *   ┌──────────────┐  LoRa req   ┌──────────────┐  LoRa resp  ┌──────────────┐  UART  ┌──────┐
 *   │  ESTE NÓ     │ ──────────> │  Sensor Node │ ──────────> │   ESTE NÓ    │ ─────> │ FPGA │
 *   │ (requester)  │             │  + BH1750    │             │ (re-recv)    │        │ MIPS │
 *   └──────────────┘             └──────────────┘             └──────────────┘        └──────┘
 *           ▲                                                          │
 *           └─── botão PRG dispara request ────────────────────────────┘
 *
 * Comportamento:
 *   1. Inicializa OLED, SX1262 e UART para o FPGA
 *   2. Configura GPIO0 (botão PRG) como interrupção por borda de descida
 *   3. Aguarda pressionamento via semáforo binário (ISR → task)
 *   4. Ao pressionar:
 *        a. Aplica debounce de 200ms
 *        b. Transmite [0xBB, 0x01] via LoRa
 *        c. Entra em RX com timeout de 5s aguardando resposta
 *        d. Valida tamanho, header e checksum XOR
 *        e. Mostra lux no OLED + log serial
 *        f. Encaminha o pacote completo [0xAA HIGH LOW XOR] via UART para o FPGA
 *   5. Volta a aguardar próximo pressionamento
 *
 * UART → FPGA:
 *   - Baud: 9600 (compatível com UARTController.v no FPGA)
 *   - Pinos: TX=GPIO 19, RX=GPIO 20 (43/44 são console UART0 do S3)
 *   - O pacote enviado é EXATAMENTE o que veio por LoRa, byte por byte:
 *     o FPGA tem seu próprio parser que valida o XOR de novo.
 *
 * GATILHO REMOTO PELO FPGA (KEY2):
 *   - Quando KEY2 é pressionado na DE2-115, o FPGA envia o byte 0xCC
 *     pela UART. Uma task aqui (fpga_listener_task) escuta o RX,
 *     detecta esse byte e dispara o mesmo semáforo que o ISR do PRG
 *     dispara — acionando o ciclo de request LoRa exatamente igual
 *     a um aperto manual.
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
#include "driver/uart.h"
#include "esp_log.h"
#include "sx1262.h"
#include "ssd1306.h"

static const char *TAG = "gateway";

/* ── Protocolo Request/Response ──────────────────────────────────── */
#define HEADER_REQUEST   0xBB
#define CMD_READ_SENSOR  0x01
#define HEADER_RESPONSE  0xAA
#define RESPONSE_LEN     4       /* [HEADER, HIGH, LOW, XOR]           */

/* ── Botão de trigger ────────────────────────────────────────────── */
#define BUTTON_GPIO      GPIO_NUM_0   /* Botão PRG do Heltec V3        */
#define RX_TIMEOUT_MS    5000         /* Sensor deve responder em <5s  */

/* ── UART para o FPGA ────────────────────────────────────────────── */
/* UART_NUM_1 a 9600 baud, 8N1.
 *
 * Pinos: usei GPIO 19 e 20. Evitei 43/44 porque no ESP32-S3 esses
 * pinos são o console UART0 — conflito direto, ESP crashava no boot. */
#define FPGA_UART_NUM    UART_NUM_1
#define FPGA_UART_TX     GPIO_NUM_19
#define FPGA_UART_RX     GPIO_NUM_20
#define FPGA_UART_BAUD   9600
#define FPGA_UART_BUF    256

/* Byte que o FPGA manda pelo KEY2. Distinto dos bytes do protocolo LoRa
 * (0xAA = response, 0xBB = request) pra não ter ambiguidade. */

 #define FPGA_UART_CMD_TRIGGER  0xCC

/* ── Sincronização ISR → Task ────────────────────────────────────── */
static SemaphoreHandle_t s_btn_sem;

/**
 * @brief ISR do botão PRG (GPIO0).
 * IRAM_ATTR é obrigatório pois ISRs rodam mesmo com cache de flash off.
 */
static void IRAM_ATTR button_isr_handler(void *arg)
{
    BaseType_t higher_prio_woken = pdFALSE;
    xSemaphoreGiveFromISR(s_btn_sem, &higher_prio_woken);
    portYIELD_FROM_ISR(higher_prio_woken);
}

/**
 * @brief Configura GPIO0 como entrada com pull-up + IRQ neg-edge.
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
 * Inicializa a UART_NUM_1 a 9600 8N1 nos pinos FPGA_UART_TX/RX.
 *
 * Ordem dos passos importa na ESP-IDF v6.0:
 *   1º param_config
 *   2º set_pin
 *   3º driver_install
 *
 * E atenção aos parâmetros do install: a ordem é (port, RX_buf, TX_buf,
 * queue_size, queue, flags). Inverter dá erro no driver.
 */
static esp_err_t fpga_uart_init(void)
{
    esp_err_t ret;

    uart_config_t cfg = {
        .baud_rate = FPGA_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    /* 1. Configura parâmetros */
    ret = uart_param_config(FPGA_UART_NUM, &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config falhou: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 2. Atribui pinos (RTS/CTS sem mudança) */
    ret = uart_set_pin(FPGA_UART_NUM, FPGA_UART_TX, FPGA_UART_RX,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin falhou: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 3. Instala o driver.
       ORDEM CORRETA dos parâmetros: (port, RX_buf, TX_buf, queue_size, queue, flags)
       - RX_buf = 256 (>= UART_HW_FIFO_LEN=128)
       - TX_buf = 0   (escritas bloqueiam até sair, sem buffer extra; OK para 4 bytes)
       - queue_size = 0 e queue = NULL: não queremos eventos. */
    ret = uart_driver_install(FPGA_UART_NUM,
                              FPGA_UART_BUF,    /* RX */
                              0,                /* TX */
                              0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install falhou: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "UART para FPGA: TX=%d, RX=%d, %d baud, 8N1",
             FPGA_UART_TX, FPGA_UART_RX, FPGA_UART_BAUD);
    return ESP_OK;
}

/**
 * @brief Task principal do gateway LoRa.
 *
 * Roda no Core 1 (deixa Core 0 para o stack WiFi/BT, se ativados).
 */
static void gateway_task(void *arg)
{
    sx1262_t   radio;
    ssd1306_t *oled = NULL;
    esp_err_t  ret;

    /* ── Inicialização do display ────────────────────────────────── */
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
    /* OBS: a UART para o FPGA já foi inicializada no app_main, antes
       das tasks serem criadas — caso contrário a fpga_listener_task
       começaria chamando uart_read_bytes em um driver não instalado. */
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

    /* ── Loop principal: aguarda botão → request → response → forward ── */
    while (1) {

        /* Bloqueia até a ISR sinalizar que o botão foi pressionado. */
        if (xSemaphoreTake(s_btn_sem, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        /* ── Debounce ────────────────────────────────────────────── */
        vTaskDelay(pdMS_TO_TICKS(200));
        xSemaphoreTake(s_btn_sem, 0);

        ESP_LOGI(TAG, "---------------------------------------");
        ESP_LOGI(TAG, "Botao pressionado! Enviando request...");

        /* ── Monta e envia pacote de requisição via LoRa ─────────── */
        uint8_t request[] = { HEADER_REQUEST, CMD_READ_SENSOR };

        ret = sx1262_send_packet(&radio, request, sizeof(request));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Falha ao enviar request: %s", esp_err_to_name(ret));
            continue;
        }
        ESP_LOGI(TAG, "Request enviado [0x%02X 0x%02X]", request[0], request[1]);

        /* ── Aguarda resposta com timeout ────────────────────────── */
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

        /* ── Reconstrói o valor de lux (BH1750) ──────────────────── */
        uint16_t lux = ((uint16_t)data_high << 8) | data_low;

        ESP_LOGI(TAG, ">>> Luminosidade (BH1750): %u lux  (0x%04X)", lux, lux);
        ESP_LOGI(TAG, "    Pacote: [0x%02X 0x%02X 0x%02X 0x%02X] | RSSI: %d dBm",
                 rx_data[0], rx_data[1], rx_data[2], rx_data[3], rssi);

        /* ── Encaminha pacote para o FPGA via UART ──────────────── */
        /* Repassa os 4 bytes EXATAMENTE como vieram. O FPGA tem seu
           próprio parser que valida o XOR de novo, então mesmo se este
           gateway fosse bypassed, o FPGA ainda detectaria erros. */
        int written = uart_write_bytes(FPGA_UART_NUM,
                                       (const char *)rx_data, RESPONSE_LEN);
        if (written == RESPONSE_LEN) {
            ESP_LOGI(TAG, "    Encaminhado para FPGA via UART (%d bytes)", written);
        } else {
            ESP_LOGW(TAG, "    UART write incompleto: %d/%d bytes", written, RESPONSE_LEN);
        }

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
            ssd1306_draw_string(oled, 7, 0, "-> FPGA OK");
            ssd1306_update(oled);
        }
    }
}

/**
 * Task que escuta a UART do FPGA. Ao receber 0xCC, dá give no semáforo
 * (mesmo que o ISR do PRG faria), então o ciclo LoRa dispara igual a
 * um aperto manual.
 *
 * Uso timeout curto no uart_read_bytes em vez de portMAX_DELAY: com
 * espera infinita o driver da v6.0 dava erro em loop, com 100ms não.
 */
static void fpga_listener_task(void *arg)
{
    uint8_t rx_byte;

    ESP_LOGI(TAG, "FPGA listener iniciado: aguardando 0xCC no RX");

    while (1) {
        /* Lê com timeout curto. uart_read_bytes retorna 0 quando não tem
           dados (não é erro). A task fica eficientemente bloqueada no
           driver durante esses 100ms. */
        int n = uart_read_bytes(FPGA_UART_NUM, &rx_byte, 1, pdMS_TO_TICKS(100));
        if (n <= 0) {
            continue;  /* timeout normal, sem byte — volta a esperar */
        }

        if (rx_byte == FPGA_UART_CMD_TRIGGER) {
            ESP_LOGI(TAG, "<<< KEY2 do FPGA detectado (0xCC) — disparando request");
            /* Mesmo efeito do ISR do botão PRG: acorda a gateway_task. */
            xSemaphoreGive(s_btn_sem);
        } else {
            ESP_LOGW(TAG, "Byte UART desconhecido: 0x%02X (ignorado)", rx_byte);
        }
    }
}

/**
 * @brief Entry point da aplicação.
 */
void app_main(void)
{
    s_btn_sem = xSemaphoreCreateBinary();

    esp_err_t ret = button_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao configurar botao: %s", esp_err_to_name(ret));
        return;
    }

    /* IMPORTANTE: inicializar a UART ANTES de criar as tasks, para que
       a fpga_listener_task não comece chamando uart_read_bytes em um
       driver ainda não instalado (causa erros de driver em loop). */
    ret = fpga_uart_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "UART FPGA falhou — operando sem forwarding");
        /* Não cria a fpga_listener_task se a UART falhou. */
        xTaskCreatePinnedToCore(gateway_task, "gateway", 8192, NULL, 5, NULL, 1);
        return;
    }

    /* gateway_task: a tarefa principal (request LoRa + forwarding) */
    xTaskCreatePinnedToCore(gateway_task, "gateway", 8192, NULL, 5, NULL, 1);

    /* fpga_listener_task: escuta a UART vinda do FPGA.
       Stack menor pois só faz uart_read + give de semáforo.
       Prioridade igual à da gateway_task: o semáforo dá a sincronização. */
    xTaskCreatePinnedToCore(fpga_listener_task, "fpga_lst", 3072, NULL, 5, NULL, 1);
}
