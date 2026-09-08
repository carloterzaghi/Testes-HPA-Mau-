/**
 * @file sx1262.h
 * @brief Driver para SX1262 LoRa Node (LF) — Raspberry Pi Pico W
 *
 * Pinagem SPI0:
 *   SCK  = GP18 | MOSI = GP19 | MISO = GP16
 *   CS   = GP17 | BUSY = GP2  | DIO1 = GP20 | RST = GP15
 *
 * Parâmetros LoRa: 433 MHz | SF7 | BW 125 kHz | CR 4/5
 */

#ifndef SX1262_H
#define SX1262_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

/* =========================================================================
 * Pinos de hardware
 * ========================================================================= */
#define SX1262_SPI_PORT     spi0
#define SX1262_SPI_FREQ_HZ  2000000U   /* 2 MHz */

#define SX1262_SCK_PIN      18
#define SX1262_MOSI_PIN     19
#define SX1262_MISO_PIN     16
#define SX1262_CS_PIN       17
#define SX1262_BUSY_PIN     2
#define SX1262_DIO1_PIN     20
#define SX1262_RST_PIN      15

/* =========================================================================
 * TCXO (oscilador de referência) — a maioria dos módulos SX1262 usa TCXO
 * alimentado pelo pino DIO3 do próprio chip, em vez de cristal passivo.
 * Se o seu módulo usar XTAL (sem TCXO), defina SX1262_USE_TCXO como 0.
 * ========================================================================= */
#define SX1262_USE_TCXO         1        /* 1 = módulo com TCXO no DIO3   */
#define SX1262_TCXO_VOLTAGE     0x02U    /* 0x02 = 1.8V (mais comum)      */
#define SX1262_TCXO_DELAY_MS    5U       /* Tempo de estabilização (ms)   */

/* =========================================================================
 * Parâmetros LoRa (altere conforme necessário)
 * ========================================================================= */
#define LORA_FREQ_HZ        433000000UL   /* 433 MHz                  */
#define LORA_SF             7             /* Spreading Factor 7       */
#define LORA_BW             SX1262_BW_125 /* Bandwidth 125 kHz        */
#define LORA_CR             SX1262_CR_4_5 /* Coding Rate 4/5          */
#define LORA_TX_POWER_DBM   14            /* Potência TX em dBm       */
#define LORA_PREAMBLE_LEN   8             /* Símbolos de preâmbulo    */
#define LORA_SYNC_WORD_PRI  0x14          /* Sync word rede privada   */
#define LORA_SYNC_WORD_SEC  0x24          /* (par: 0x1424 = privado)  */

/* =========================================================================
 * Comandos do SX1262 (Tabela 11-1 do datasheet)
 * ========================================================================= */
#define SX1262_CMD_SET_SLEEP              0x84U
#define SX1262_CMD_SET_STANDBY            0x80U
#define SX1262_CMD_SET_FS                 0xC1U
#define SX1262_CMD_SET_TX                 0x83U
#define SX1262_CMD_SET_RX                 0x82U
#define SX1262_CMD_SET_PACKET_TYPE        0x8AU
#define SX1262_CMD_SET_RF_FREQUENCY       0x86U
#define SX1262_CMD_SET_PA_CONFIG          0x95U
#define SX1262_CMD_SET_TX_PARAMS          0x8EU
#define SX1262_CMD_SET_MODULATION_PARAMS  0x8BU
#define SX1262_CMD_SET_PACKET_PARAMS      0x8CU
#define SX1262_CMD_SET_BUFFER_BASE_ADDR   0x8FU
#define SX1262_CMD_SET_REGULATOR_MODE     0x96U
#define SX1262_CMD_CALIBRATE              0x89U
#define SX1262_CMD_CALIBRATE_IMAGE        0x98U
#define SX1262_CMD_SET_DIO_IRQ_PARAMS     0x08U
#define SX1262_CMD_GET_IRQ_STATUS         0x12U
#define SX1262_CMD_CLEAR_IRQ_STATUS       0x02U
#define SX1262_CMD_SET_DIO2_AS_RF_SWITCH  0x9DU
#define SX1262_CMD_SET_DIO3_AS_TCXO_CTRL 0x97U
#define SX1262_CMD_WRITE_BUFFER           0x0EU
#define SX1262_CMD_READ_BUFFER            0x1EU
#define SX1262_CMD_WRITE_REGISTER         0x0DU
#define SX1262_CMD_READ_REGISTER          0x1DU
#define SX1262_CMD_GET_RX_BUFFER_STATUS   0x13U
#define SX1262_CMD_GET_PACKET_STATUS      0x14U
#define SX1262_CMD_GET_STATUS             0xC0U
#define SX1262_CMD_GET_DEVICE_ERRORS      0x17U
#define SX1262_CMD_CLEAR_DEVICE_ERRORS    0x07U

/* =========================================================================
 * Registradores internos do SX1262
 * ========================================================================= */
#define SX1262_REG_LORA_SYNC_WORD_MSB     0x0740U
#define SX1262_REG_LORA_SYNC_WORD_LSB     0x0741U
#define SX1262_REG_OCP                    0x08E7U
#define SX1262_REG_TX_CLAMP_CFG           0x08D8U
#define SX1262_REG_RX_GAIN                0x08ACU

/* =========================================================================
 * IRQ bits (campo de 16 bits)
 * ========================================================================= */
#define SX1262_IRQ_TX_DONE          (1u << 0)
#define SX1262_IRQ_RX_DONE          (1u << 1)
#define SX1262_IRQ_PREAMBLE_DET     (1u << 2)
#define SX1262_IRQ_SYNC_WORD_VALID  (1u << 3)
#define SX1262_IRQ_HEADER_VALID     (1u << 4)
#define SX1262_IRQ_HEADER_ERR       (1u << 5)
#define SX1262_IRQ_CRC_ERR          (1u << 6)
#define SX1262_IRQ_CAD_DONE         (1u << 7)
#define SX1262_IRQ_CAD_DETECTED     (1u << 8)
#define SX1262_IRQ_TIMEOUT          (1u << 9)
#define SX1262_IRQ_ALL              0x03FFu

/* =========================================================================
 * Enumerações
 * ========================================================================= */

/** Modo standby */
typedef enum {
    SX1262_STANDBY_RC   = 0x00, /**< Oscilador RC interno (~13 MHz) */
    SX1262_STANDBY_XOSC = 0x01  /**< Cristal externo / TCXO       */
} sx1262_standby_mode_t;

/** Tipo de pacote */
typedef enum {
    SX1262_PACKET_TYPE_GFSK = 0x00,
    SX1262_PACKET_TYPE_LORA = 0x01
} sx1262_packet_type_t;

/** Largura de banda LoRa */
typedef enum {
    SX1262_BW_7_8   = 0x00,
    SX1262_BW_10_4  = 0x08,
    SX1262_BW_15_6  = 0x01,
    SX1262_BW_20_8  = 0x09,
    SX1262_BW_31_25 = 0x02,
    SX1262_BW_41_7  = 0x0A,
    SX1262_BW_62_5  = 0x03,
    SX1262_BW_125   = 0x04,  /**< 125 kHz */
    SX1262_BW_250   = 0x05,
    SX1262_BW_500   = 0x06
} sx1262_bw_t;

/** Coding Rate LoRa */
typedef enum {
    SX1262_CR_4_5 = 0x01,
    SX1262_CR_4_6 = 0x02,
    SX1262_CR_4_7 = 0x03,
    SX1262_CR_4_8 = 0x04
} sx1262_cr_t;

/** Tempo de rampa de potência TX */
typedef enum {
    SX1262_RAMP_10U   = 0x00,
    SX1262_RAMP_20U   = 0x01,
    SX1262_RAMP_40U   = 0x02,
    SX1262_RAMP_80U   = 0x03,
    SX1262_RAMP_200U  = 0x04,
    SX1262_RAMP_800U  = 0x05,
    SX1262_RAMP_1700U = 0x06,
    SX1262_RAMP_3400U = 0x07
} sx1262_ramp_time_t;

/** Códigos de retorno de sx1262_receive() */
typedef enum {
    SX1262_RX_OK        =  0,  /**< Sucesso (ret = bytes recebidos) */
    SX1262_RX_TIMEOUT   = -1,  /**< Sem pacote dentro do timeout   */
    SX1262_RX_CRC_ERROR = -2,  /**< Pacote recebido com CRC errado  */
    SX1262_RX_ERROR     = -3   /**< Erro genérico                   */
} sx1262_rx_status_t;

/* =========================================================================
 * API pública
 * ========================================================================= */

/**
 * @brief Inicializa SPI, GPIOs e configura o SX1262 em modo LoRa.
 * @return true  se inicialização OK
 * @return false em caso de erro
 */
bool sx1262_init(void);

/**
 * @brief Transmite um pacote LoRa.
 * @param data       Ponteiro para o buffer de dados
 * @param len        Número de bytes a enviar (máx. 255)
 * @param timeout_ms Timeout em ms para aguardar TxDone
 * @return true  se pacote enviado com sucesso
 * @return false em caso de timeout ou erro
 */
bool sx1262_send(const uint8_t *data, uint8_t len, uint32_t timeout_ms);

/**
 * @brief Aguarda e lê um pacote LoRa recebido.
 * @param buf        Buffer de destino (mínimo 256 bytes)
 * @param len        [out] Número de bytes recebidos
 * @param timeout_ms Timeout em ms para aguardar RxDone
 * @return Número de bytes recebidos (> 0) ou sx1262_rx_status_t (< 0)
 */
int sx1262_receive(uint8_t *buf, uint8_t *len, uint32_t timeout_ms);

/**
 * @brief Retorna o RSSI do último pacote recebido (em dBm).
 */
int8_t sx1262_get_last_rssi(void);

/**
 * @brief Retorna o SNR do último pacote recebido (em dB).
 */
int8_t sx1262_get_last_snr(void);

/**
 * @brief Lê o registro de erros internos do rádio (GetDeviceErrors, 0x17).
 *
 * Útil para diagnosticar travamentos de TX/RX sem IRQ: bit 5 = XOSC_START_ERR
 * (oscilador/TCXO não estabilizou) e bit 6 = PLL_LOCK_ERR (PLL de RF não travou).
 * @return Bitmap de 16 bits com os erros reportados pelo chip.
 */
uint16_t sx1262_get_device_errors(void);

#endif /* SX1262_H */
