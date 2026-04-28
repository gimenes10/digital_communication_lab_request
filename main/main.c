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
 *   - Pinos: TX=GPIO 43, RX=GPIO 44
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
/* UART_NUM_1 (não conflita com UART_NUM_0 que é o console USB).
 * Pinos escolhidos: GPIO 43 e 44. Estão livres no Heltec V3 e não
 * conflitam com SX1262 (8-14), OLED (17/18/21) ou Vext (36).      */
#define FPGA_UART_NUM    UART_NUM_1
#define FPGA_UART_TX     GPIO_NUM_43
#define FPGA_UART_RX     GPIO_NUM_44
#define FPGA_UART_BAUD   9600
#define FPGA_UART_BUF    256   /* Buffer interno do driver UART      */

/* Comando vindo do FPGA pelo RX da UART:
 * 0xCC é enviado pelo top.v quando KEY2 é pressionado.
 * Distinto de 0xAA/0xBB do protocolo LoRa para evitar ambiguidade. */
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
 * @brief Inicializa UART para forwarding ao FPGA.
 *
 * Configuração: 9600 baud, 8 bits, sem paridade, 1 stop bit (8N1).
 * Esses parâmetros precisam casar com o UARTController.v do FPGA:
 * o módulo Verilog assume 8N1 e o baud rate é parametrizado lá em
 * 9600 também (vide top.v: BAUD_RATE(9600)).
 *
 * RX é ligado mas não usado nesta fase — fica preparado pra quando
 * o FPGA mandar comandos de volta no futuro.
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

    ret = uart_driver_install(FPGA_UART_NUM, FPGA_UART_BUF, FPGA_UART_BUF,
                              0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install falhou: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_param_config(FPGA_UART_NUM, &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config falhou: %s", esp_err_to_name(ret));
        return ret;
    }

    /* UART_PIN_NO_CHANGE para os pinos não usados (RTS/CTS) */
    ret = uart_set_pin(FPGA_UART_NUM, FPGA_UART_TX, FPGA_UART_RX,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin falhou: %s", esp_err_to_name(ret));
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

    /* ── Inicialização do UART para FPGA ─────────────────────────── */
    /* Não-fatal se falhar: o gateway continua funcionando, só não
       encaminha pro FPGA. Loga warning e segue. */
    ret = fpga_uart_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "UART FPGA falhou — operando sem forwarding");
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
 * @brief Task que escuta a UART do FPGA e dispara o semáforo
 *        ao receber o byte 0xCC (comando "trigger request").
 *
 * Roda em paralelo com gateway_task: enquanto a gateway_task fica
 * bloqueada no semáforo, esta task fica bloqueada no uart_read_bytes.
 * Ambas são produtoras do mesmo sinal (semáforo), do ponto de vista
 * da gateway_task não há diferença entre um aperto de PRG e um 0xCC
 * vindo do FPGA.
 *
 * Por que polling em vez de interrupção?
 *   - O driver UART do ESP-IDF já implementa buffering interno.
 *     uart_read_bytes bloqueia eficientemente sem consumir CPU.
 *   - Configurar IRQ direto seria mais código, sem benefício aqui:
 *     a latência de alguns ms não impacta o caso de uso.
 *
 * Bytes diferentes de 0xCC são ignorados — protege contra ruído na
 * linha quando o FPGA está sendo ligado/desligado.
 */
static void fpga_listener_task(void *arg)
{
    uint8_t rx_byte;

    ESP_LOGI(TAG, "FPGA listener iniciado: aguardando 0xCC no RX");

    while (1) {
        /* Bloqueia indefinidamente até chegar 1 byte. portMAX_DELAY
           impede polling ativo — a task fica em estado blocked. */
        int n = uart_read_bytes(FPGA_UART_NUM, &rx_byte, 1, portMAX_DELAY);
        if (n != 1) {
            continue;
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

    /* gateway_task: a tarefa principal (request LoRa + forwarding) */
    xTaskCreatePinnedToCore(gateway_task, "gateway", 8192, NULL, 5, NULL, 1);

    /* fpga_listener_task: escuta a UART vinda do FPGA.
       Stack menor pois só faz uart_read + give de semáforo.
       Prioridade igual à da gateway_task: o semáforo dá a sincronização. */
    xTaskCreatePinnedToCore(fpga_listener_task, "fpga_lst", 3072, NULL, 5, NULL, 1);
}
