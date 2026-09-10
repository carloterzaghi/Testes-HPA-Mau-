/**
 * @file Teste-HPA.c
 * @brief Transceptor LoRa Ping-Pong usando SX1262 LoRa Node (LF)
 *
 * O no envia um pacote de teste, aguarda resposta por RX_TIMEOUT_MS,
 * imprime o resultado no terminal USB (115200 baud) e repete.
 *
 * Monitor serial: minicom / PuTTY / Thonny a 115200 baud,
 *                 ou: python -m serial.tools.miniterm COMx 115200
 *
 * Nota: No Pico W o LED onboard e controlado pelo chip CYW43 (nao GPIO).
 *       O codigo usa GP22 como LED de status externo opcional.
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "sx1262.hpp"

/* --- Configuracao da aplicacao ------------------------------------------ */
#define TX_INTERVAL_MS   2000U   /**< Intervalo entre transmissoes (ms)     */
#define TX_TIMEOUT_MS    1000U   /**< Timeout para TxDone (ms)              */
#define RX_TIMEOUT_MS    3000U   /**< Timeout para aguardar resposta (ms)   */
#define MAX_PAYLOAD      255U    /**< Tamanho maximo do buffer de recepcao  */

/** LED de status externo opcional (GP22, ~330 ohm para GND).
 *  Se nao houver LED, o codigo funciona normalmente sem ele.               */
#define STATUS_LED_PIN   22u

/* --- Separador visual no terminal --------------------------------------- */
#define SEP "--------------------------------------------\n"

int main(void) {
    /* Inicializa stdio USB (aguarda ate 3 s pelo host USB) */
    stdio_init_all();
    sleep_ms(10000);

    printf("\n");
    printf("============================================\n");
    printf("   SX1262 LoRa Transceptor - Pico W\n");
    printf("   433 MHz | SF7 | BW125 | CR4/5\n");
    printf("============================================\n\n");

    /* -- LED de status (GP22, opcional) --------------------------------- */
    gpio_init(STATUS_LED_PIN);
    gpio_set_dir(STATUS_LED_PIN, GPIO_OUT);
    gpio_put(STATUS_LED_PIN, 0);

    /* -- Inicializacao do SX1262 --------------------------------------- */
    printf("[INIT] Inicializando SX1262...\n");
    if (!sx1262_init()) {
        printf("[ERRO] Falha ao inicializar o SX1262! Verifique as conexoes.\n");
        /* Pisca LED rapidamente para sinalizar erro de hardware */
        while (true) {
            gpio_put(STATUS_LED_PIN, 1); sleep_ms(100);
            gpio_put(STATUS_LED_PIN, 0); sleep_ms(100);
        }
    }
    printf("[INIT] SX1262 inicializado com sucesso!\n\n");

    uint32_t tx_counter = 0;
    uint8_t  rx_buf[MAX_PAYLOAD + 1]; /* +1 para terminador '\0' */

    while (true) {
        /* -- Transmissao ------------------------------------------------ */
        char msg[64];
        int  msg_len = snprintf(msg, sizeof(msg),
                                "Hello LoRa! #%lu", (unsigned long)tx_counter++);

        /* Liga LED durante transmissao */
        gpio_put(STATUS_LED_PIN, 1);

        printf("[TX] Enviando: \"%s\" (%d bytes)\n", msg, msg_len);
        bool tx_ok = sx1262_send((const uint8_t *)msg,
                                 (uint8_t)msg_len,
                                 TX_TIMEOUT_MS);
        gpio_put(STATUS_LED_PIN, 0);

        if (tx_ok) {
            printf("[TX] OK - Pacote transmitido com sucesso.\n");
        } else {
            printf("[TX] FALHA - Timeout ou erro ao transmitir.\n");
        }

        /* -- Recepcao --------------------------------------------------- */
        printf("[RX] Aguardando resposta por %u ms...\n", RX_TIMEOUT_MS);

        uint8_t rx_len = 0;
        int result = sx1262_receive(rx_buf, &rx_len, RX_TIMEOUT_MS);

        if (result > 0) {
            rx_buf[rx_len] = '\0'; /* garante terminador de string */
            printf("[RX] OK - Recebido %d byte(s): \"%s\"\n", rx_len, rx_buf);
            printf("[RX]   RSSI: %d dBm  |  SNR: %d dB\n",
                   (int)sx1262_get_last_rssi(),
                   (int)sx1262_get_last_snr());
        } else {
            switch (result) {
                case SX1262_RX_TIMEOUT:
                    printf("[RX] Timeout - nenhum pacote recebido.\n");
                    break;
                case SX1262_RX_CRC_ERROR:
                    printf("[RX] ERRO - CRC invalido no pacote recebido.\n");
                    break;
                default:
                    printf("[RX] ERRO - codigo %d.\n", result);
                    break;
            }
        }

        printf(SEP);
        sleep_ms(TX_INTERVAL_MS);
    }
}
