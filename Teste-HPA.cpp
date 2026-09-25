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
#include "pico/stdio_usb.h"
#include "pico/time.h"
#include "sx1262.hpp"
#include "tfluna.hpp"
#include "bmp280.hpp"
#include "as5600.hpp"
#include "gy_gps6mv2.hpp"
#include "i2c_bus.hpp"


/* --- Configuracao da aplicacao ------------------------------------------ */
#define TX_INTERVAL_MS   2000U   /**< Intervalo entre transmissoes (ms)     */
#define TX_TIMEOUT_MS    1000U   /**< Timeout para TxDone (ms)              */
#define RX_TIMEOUT_MS    3000U   /**< Timeout para aguardar resposta (ms)   */
#define MAX_PAYLOAD      255U    /**< Tamanho maximo do buffer de recepcao  */

/** LED de status externo opcional (GP22, ~330 ohm para GND).
 *  Se nao houver LED, o codigo funciona normalmente sem ele.               */
#define STATUS_LED_PIN   22u

#define sensor_interval_ms 1U /**< Intervalo de leitura dos sensores (ms) */
/* --- Separador visual no terminal --------------------------------------- */
#define SEP "--------------------------------------------\n"

static bool is_gps_navigation_sentence(const char *sentence) {
    if (!sentence || sentence[0] != '$') {
        return false;
    }

    return !strncmp(sentence + 1, "GPGGA", 5) ||
           !strncmp(sentence + 1, "GNGGA", 5) ||
           !strncmp(sentence + 1, "GPRMC", 5) ||
           !strncmp(sentence + 1, "GNRMC", 5);
}

I2CBus i2c_bus(5, 4); // SCL, SDA
TFLuna lidar(i2c_bus, TFLUNA_DEFAULT_ADDRESS);
BMP280 barometer(i2c_bus, BMP280_ADDRESS_LOW);
AS5600 encoder(i2c_bus);
GYGPS6MV2 gps(uart0, 0, 1); // TX do Pico: GP0, RX do Pico: GP1

volatile bool sensor_read_pending = false;


bool sensor_timer_callback(repeating_timer_t*) {
    sensor_read_pending = true;
    return true;
}


int main(void) {
    /* Inicializa stdio USB e aguarda o terminal, sem bloquear indefinidamente. */
    stdio_init_all();
    uint32_t usb_wait_start = to_ms_since_boot(get_absolute_time());
    while (!stdio_usb_connected() &&
           to_ms_since_boot(get_absolute_time()) - usb_wait_start < 15000U) {
        sleep_ms(100);
    }

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

    printf("[INIT] Inicializando BMP280...\n");
    repeating_timer_t sensor_timer;
    if (!barometer.init()) {
        printf("[ERRO] Falha ao inicializar o BMP280. Verifique o endereco 0x76/0x77.\n");
    } else {
        printf("[INIT] BMP280 inicializado com sucesso!\n");
    }

    if (!encoder.init()) {
        printf("[ERRO] Falha ao comunicar com o AS5600 via I2C (endereco 0x36).\n");
    } else {
        printf("[INIT] AS5600 I2C configurado em SDA GP4, SCL GP5 (0x36).\n");
    }

    if (!add_repeating_timer_us(-static_cast<int64_t>(sensor_interval_ms) * 1e3,
                                sensor_timer_callback,
                                nullptr,
                                &sensor_timer)) {
        printf("[ERRO] Falha ao iniciar o timer dos sensores.\n");
    } else {
        printf("[INIT] Timer dos sensores configurado em %.2f kHz.\n", 1.0f / sensor_interval_ms);
    }

    uint32_t tx_counter = 0;
    uint8_t  rx_buf[MAX_PAYLOAD + 1]; /* +1 para terminador '\0' */
    uint32_t gps_report_counter = 0;
    uint32_t heartbeat_counter = 0;
    uint32_t encoder_wait_start_ms =
        to_ms_since_boot(get_absolute_time());
    bool encoder_signal_warning_printed = false;
    bool gps_diagnostic_done = false;

    while (true) {
        if (sensor_read_pending) {
            sensor_read_pending = false;

            if (lidar.read()) {
                printf("Distancia: %u cm\n", lidar.data.distance_cm);
            }

            if (barometer.read()) {
                printf("Pressao: %lu Pa | Temperatura: %ld.%02ld C\n",
                       static_cast<unsigned long>(barometer.data.pressure_pa),
                       static_cast<long>(barometer.data.temperature_centi_c / 100),
                       static_cast<long>(barometer.data.temperature_centi_c % 100));
            }

            if (encoder.read()) {
                encoder_wait_start_ms =
                    to_ms_since_boot(get_absolute_time());
                encoder_signal_warning_printed = false;
                int32_t speed_milli = encoder.data.speed_rpm_milli;
                uint32_t speed_abs = speed_milli < 0
                    ? static_cast<uint32_t>(-static_cast<int64_t>(speed_milli))
                    : static_cast<uint32_t>(speed_milli);
                  printf("AS5600: angulo %u.%03u graus | raw %u | velocidade %s%lu.%03lu RPM\n",
                       encoder.data.angle_mdeg / 1000,
                       encoder.data.angle_mdeg % 1000,
                      encoder.data.raw_angle,
                       speed_milli < 0 ? "-" : "",
                       static_cast<unsigned long>(speed_abs / 1000),
                       static_cast<unsigned long>(speed_abs % 1000));
            } else if (!encoder_signal_warning_printed) {
                uint32_t encoder_wait_ms =
                    to_ms_since_boot(get_absolute_time()) - encoder_wait_start_ms;
                if (encoder_wait_ms >= 3000U) {
                              printf("[AS5600] Nenhuma leitura I2C recebida. "
                                  "Verifique SDA GP4, SCL GP5, 3V3 e GND.\n");
                    encoder_signal_warning_printed = true;
                }
            }

            if (!gps_diagnostic_done) {
                char gps_sentence[96];
                if (gps.read_sentence(gps_sentence, sizeof(gps_sentence), 1000)) {
                    printf("[GPS] Sentenca recebida: %s", gps_sentence);
                    gps_diagnostic_done = is_gps_navigation_sentence(gps_sentence);
                    if (!gps_diagnostic_done) {
                        printf("[GPS] Aguardando sentenca GGA/RMC; GPTXT e apenas mensagem de inicializacao.\n");
                    }
                } else {
                    //printf("[GPS] Nenhuma sentenca NMEA recebida em 1 s. Verifique baud, TX/RX e GND.\n");
                }
            } else if (gps.update(1000)) {
                printf("Latitude: %ld\n", gps.data.latitude_e6);
                printf("Longitude: %ld\n", gps.data.longitude_e6);
                printf("Altitude: %ld mm\n", gps.data.altitude_mm);
                printf("Satelites: %u\n", gps.data.satellites);
            } else if (++gps_report_counter % 5 == 0) {
                printf("[GPS] Sem fix valido. Verifique antena, alimentacao e GPS TX -> GP1.\n");
            }
        }

        if (++heartbeat_counter >= 20) {
            heartbeat_counter = 0;
            printf("[STATUS] Firmware em execucao; USB serial conectada=%s\n",
                   stdio_usb_connected() ? "sim" : "nao");
        }

        sleep_ms(100);
    }
}
