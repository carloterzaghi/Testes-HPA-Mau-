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
#include "tfluna.hpp"
#include "gy_gps6mv2.hpp"


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

TFLuna lidar(TFLUNA_DEFAULT_ADDRESS, 5, 4); // endereço, SCL, SDA
GYGPS6MV2 gps(uart0, 0, 1); // TX do Pico: GP0, RX do Pico: GP1


int main(void) {
    /* Inicializa stdio USB (aguarda ate 3 s pelo host USB) */
    stdio_init_all();
    sleep_ms(10000);

    printf("\n");
    printf("============================================\n");
    printf("   SX1262 LoRa Transceptor - Pico W\n");
    printf("   433 MHz | SF7 | BW125 | CR4/5\n");
    printf("============================================\n\n");
    printf("[GPS] UART0 9600 baud | GPS TX -> GP1 (RX) | GPS RX -> GP0 (TX)\n");

    /* -- LED de status (GP22, opcional) --------------------------------- */
    gpio_init(STATUS_LED_PIN);
    gpio_set_dir(STATUS_LED_PIN, GPIO_OUT);
    gpio_put(STATUS_LED_PIN, 0);

    /* -- Inicializacao do SX1262 --------------------------------------- */
    printf("[INIT] Inicializando SX1262...\n");
    if (!sx1262_init()) {
        printf("[ERRO] Falha ao inicializar o SX1262. Continuando com GPS e Lidar.\n");
    } else {
        printf("[INIT] SX1262 inicializado com sucesso!\n\n");
    }

    uint32_t tx_counter = 0;
    uint8_t  rx_buf[MAX_PAYLOAD + 1]; /* +1 para terminador '\0' */
    uint32_t gps_report_counter = 0;
    bool gps_diagnostic_done = false;

    while (true) {
        if (lidar.read()) {
            printf("Distancia: %u cm\n", lidar.data.distance_cm);
        }

        if (!gps_diagnostic_done) {
            char gps_sentence[96];
            if (gps.read_sentence(gps_sentence, sizeof(gps_sentence), 1000)) {
                printf("[GPS] Sentenca recebida: %s", gps_sentence);
                gps_diagnostic_done = true;
            } else {
                printf("[GPS] Nenhum byte recebido em 1 s. Verifique baud, TX/RX e GND.\n");
            }
        } else if (gps.update(1000)) {
            printf("Latitude: %ld\n", gps.data.latitude_e6);
            printf("Longitude: %ld\n", gps.data.longitude_e6);
            printf("Altitude: %ld mm\n", gps.data.altitude_mm);
            printf("Satelites: %u\n", gps.data.satellites);
        } else if (++gps_report_counter % 5 == 0) {
            printf("[GPS] Sem fix valido. Verifique antena, alimentacao e GPS TX -> GP1.\n");
        }

        sleep_ms(100);
    }
}
