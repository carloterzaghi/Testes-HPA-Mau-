/**
 * @file sx1262.c
 * @brief Implementação do driver SX1262 para Raspberry Pi Pico W
 *
 * Referência: SX1262 Datasheet v2.1 — Semtech Corporation
 *
 * Sequência de inicialização (seção 14.2 do datasheet):
 *   1. Reset
 *   2. SetStandby(RC)
 *   3. SetRegulatorMode (DC-DC)
 *   4. SetDIO2AsRfSwitchCtrl
 *   5. CalibrateImage (para a faixa de frequência)
 *   6. SetPacketType (LoRa)
 *   7. SetRfFrequency
 *   8. SetPaConfig
 *   9. SetTxParams
 *  10. SetModulationParams
 *  11. SetPacketParams
 *  12. SetBufferBaseAddress
 *  13. SetDioIrqParams
 */

#include "sx1262.hpp"
#include <stdio.h>
#include <string.h>

/* =========================================================================
 * Estado interno
 * ========================================================================= */
static int8_t s_last_rssi = 0;
static int8_t s_last_snr  = 0;

/* =========================================================================
 * Primitivas SPI / GPIO
 * ========================================================================= */

/** Seleciona o chip (CS ativo em nível baixo) */
static inline void cs_low(void)  { gpio_put(SX1262_CS_PIN, 0); }

/** Deseleciona o chip */
static inline void cs_high(void) { gpio_put(SX1262_CS_PIN, 1); }

/**
 * @brief Aguarda o sinal BUSY ir para nível baixo.
 *
 * O SX1262 mantém BUSY HIGH enquanto processa um comando.
 * Nenhum novo comando SPI deve ser enviado enquanto BUSY estiver HIGH.
 */
static void wait_busy(void) {
    /* Timeout de segurança: 2 s */
    uint32_t start = to_ms_since_boot(get_absolute_time());
    while (gpio_get(SX1262_BUSY_PIN)) {
        if (to_ms_since_boot(get_absolute_time()) - start > 2000U) {
            printf("[SX1262] AVISO: BUSY timeout!\n");
            break;
        }
        tight_loop_contents();
    }
}

/* ─── Escrita de comando (sem leitura de resposta) ─────────────────────── */
static void spi_write_cmd(uint8_t cmd, const uint8_t *data, size_t len) {
    wait_busy();
    cs_low();
    spi_write_blocking(SX1262_SPI_PORT, &cmd, 1);
    if (data && len > 0) {
        spi_write_blocking(SX1262_SPI_PORT, data, len);
    }
    cs_high();
}

/* ─── Leitura de resposta de comando ─────────────────────────────────────
 * Protocolo SX1262 read:
 *   [CMD] → [NOP → Status byte] → [NOP → Data[0]] → [NOP → Data[1]] ...
 *
 * O byte de status é o primeiro byte retornado após o comando.
 * O `spi_write_blocking` envia NOP e descarta o retorno (status).
 * O `spi_read_blocking` envia NOPs e captura os bytes de dados.
 */
static void spi_read_cmd(uint8_t cmd, uint8_t *buf, size_t len) {
    wait_busy();
    cs_low();
    uint8_t nop = 0x00;
    spi_write_blocking(SX1262_SPI_PORT, &cmd, 1);  /* envia comando        */
    spi_write_blocking(SX1262_SPI_PORT, &nop, 1);  /* descarta byte status */
    spi_read_blocking(SX1262_SPI_PORT, 0x00, buf, len); /* lê dados        */
    cs_high();
}

/* ─── Escrita em registrador interno ─────────────────────────────────────
 * Formato: [0x0D] [addr_MSB] [addr_LSB] [data_0] [data_1] ...
 */
static void write_register(uint16_t addr, const uint8_t *data, size_t len) {
    uint8_t header[3] = {
        SX1262_CMD_WRITE_REGISTER,
        (uint8_t)(addr >> 8),
        (uint8_t)(addr & 0xFF)
    };
    wait_busy();
    cs_low();
    spi_write_blocking(SX1262_SPI_PORT, header, 3);
    spi_write_blocking(SX1262_SPI_PORT, data, len);
    cs_high();
}

/* ─── Leitura de registrador interno ─────────────────────────────────────
 * Formato: [0x1D] [addr_MSB] [addr_LSB] [NOP → Status] [NOP → data_0] ...
 */
static void read_register(uint16_t addr, uint8_t *buf, size_t len) {
    uint8_t header[4] = {
        SX1262_CMD_READ_REGISTER,
        (uint8_t)(addr >> 8),
        (uint8_t)(addr & 0xFF),
        0x00    /* NOP → descarta byte de status */
    };
    wait_busy();
    cs_low();
    spi_write_blocking(SX1262_SPI_PORT, header, 4);
    spi_read_blocking(SX1262_SPI_PORT, 0x00, buf, len);
    cs_high();
}

/* ─── Escrita no buffer FIFO do rádio ────────────────────────────────────
 * Formato: [0x0E] [offset] [data_0] [data_1] ...
 */
static void write_buffer(uint8_t offset, const uint8_t *data, size_t len) {
    uint8_t header[2] = { SX1262_CMD_WRITE_BUFFER, offset };
    wait_busy();
    cs_low();
    spi_write_blocking(SX1262_SPI_PORT, header, 2);
    spi_write_blocking(SX1262_SPI_PORT, data, len);
    cs_high();
}

/* ─── Leitura do buffer FIFO do rádio ────────────────────────────────────
 * Formato: [0x1E] [offset] [NOP → Status] [NOP → data_0] ...
 */
static void read_buffer(uint8_t offset, uint8_t *buf, size_t len) {
    uint8_t header[2] = { SX1262_CMD_READ_BUFFER, offset };
    wait_busy();
    cs_low();
    spi_write_blocking(SX1262_SPI_PORT, header, 2);
    uint8_t nop = 0x00;
    spi_write_blocking(SX1262_SPI_PORT, &nop, 1); /* descarta byte status */
    spi_read_blocking(SX1262_SPI_PORT, 0x00, buf, len);
    cs_high();
}

/* =========================================================================
 * Helpers de configuração
 * ========================================================================= */

/** Reset de hardware do SX1262 via pino RST */
static void sx1262_reset(void) {
    gpio_put(SX1262_RST_PIN, 0);
    sleep_ms(10);
    gpio_put(SX1262_RST_PIN, 1);
    sleep_ms(20);
    wait_busy();
}

/**
 * @brief Converte frequência em Hz para o valor de 4 bytes do registrador.
 *
 * Fórmula (datasheet eq. 4.1):
 *   fRF_reg = (f_hz * 2^25) / f_XTAL
 *   onde f_XTAL = 32 MHz
 *
 * Usa aritmética de 64 bits para evitar overflow.
 */
static void set_rf_frequency(uint32_t freq_hz) {
    uint64_t val = ((uint64_t)freq_hz << 25) / 32000000UL;
    uint8_t buf[4] = {
        (uint8_t)(val >> 24),
        (uint8_t)(val >> 16),
        (uint8_t)(val >>  8),
        (uint8_t)(val      )
    };
    spi_write_cmd(SX1262_CMD_SET_RF_FREQUENCY, buf, 4);
}

/** Converte timeout em ms para o formato de 24 bits do SX1262.
 *
 *  Unidade de timeout: 15.625 µs → timeout_raw = timeout_ms * 64
 */
static void timeout_to_bytes(uint32_t timeout_ms, uint8_t out[3]) {
    uint32_t raw = timeout_ms * 64U;
    out[0] = (uint8_t)(raw >> 16);
    out[1] = (uint8_t)(raw >>  8);
    out[2] = (uint8_t)(raw      );
}

/* ─── IRQ helpers ──────────────────────────────────────────────────────── */

static uint16_t get_irq_status(void) {
    uint8_t buf[2];
    spi_read_cmd(SX1262_CMD_GET_IRQ_STATUS, buf, 2);
    return ((uint16_t)buf[0] << 8) | buf[1];
}

static void clear_irq(uint16_t mask) {
    uint8_t buf[2] = { (uint8_t)(mask >> 8), (uint8_t)(mask & 0xFF) };
    spi_write_cmd(SX1262_CMD_CLEAR_IRQ_STATUS, buf, 2);
}

/** Imprime os bits de erro relevantes retornados por GetDeviceErrors */
static void print_device_errors(uint16_t err) {
    if (err == 0) {
        printf("[SX1262] DeviceErrors: nenhum erro reportado.\n");
        return;
    }
    printf("[SX1262] DeviceErrors: 0x%04X", err);
    if (err & (1u << 0)) printf(" RC64K_CALIB_ERR");
    if (err & (1u << 1)) printf(" RC13M_CALIB_ERR");
    if (err & (1u << 2)) printf(" PLL_CALIB_ERR");
    if (err & (1u << 3)) printf(" ADC_CALIB_ERR");
    if (err & (1u << 4)) printf(" IMG_CALIB_ERR");
    if (err & (1u << 5)) printf(" XOSC_START_ERR (TCXO/cristal nao estabilizou)");
    if (err & (1u << 6)) printf(" PLL_LOCK_ERR (PLL de RF nao travou)");
    if (err & (1u << 8)) printf(" PA_RAMP_ERR");
    printf("\n");
}

/* ─── Standby ──────────────────────────────────────────────────────────── */
static void set_standby_rc(void) {
    uint8_t mode = SX1262_STANDBY_RC;
    spi_write_cmd(SX1262_CMD_SET_STANDBY, &mode, 1);
}

/** Lê o byte de status bruto (GetStatus, 0xC0); 0x00 ou 0xFF indica SPI quebrado */
static uint8_t get_status(void) {
    wait_busy();
    cs_low();
    uint8_t cmd = SX1262_CMD_GET_STATUS;
    uint8_t nop = 0x00, status = 0x00;
    spi_write_blocking(SX1262_SPI_PORT, &cmd, 1);
    spi_read_blocking(SX1262_SPI_PORT, nop, &status, 1);
    cs_high();
    return status;
}

/* =========================================================================
 * Inicialização pública
 * ========================================================================= */
bool sx1262_init(void) {
    /* ── 1. Inicializar SPI0 ─────────────────────────────────────────── */
    spi_init(SX1262_SPI_PORT, SX1262_SPI_FREQ_HZ);
    spi_set_format(SX1262_SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_set_function(SX1262_SCK_PIN,  GPIO_FUNC_SPI);
    gpio_set_function(SX1262_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SX1262_MISO_PIN, GPIO_FUNC_SPI);

    /* ── 2. CS (saída, inativo em HIGH) ──────────────────────────────── */
    gpio_init(SX1262_CS_PIN);
    gpio_set_dir(SX1262_CS_PIN, GPIO_OUT);
    gpio_put(SX1262_CS_PIN, 1);

    /* ── 3. RST (saída) ──────────────────────────────────────────────── */
    gpio_init(SX1262_RST_PIN);
    gpio_set_dir(SX1262_RST_PIN, GPIO_OUT);
    gpio_put(SX1262_RST_PIN, 1);

    /* ── 4. BUSY (entrada) ───────────────────────────────────────────── */
    gpio_init(SX1262_BUSY_PIN);
    gpio_set_dir(SX1262_BUSY_PIN, GPIO_IN);

    /* ── 5. DIO1 (entrada — IRQ) ─────────────────────────────────────── */
    gpio_init(SX1262_DIO1_PIN);
    gpio_set_dir(SX1262_DIO1_PIN, GPIO_IN);

    /* ── 6. Reset do chip ────────────────────────────────────────────── */
    sx1262_reset();

    /* ── 6b. Checagem de comunicação SPI: 0x00/0xFF = MISO/BUSY não conectados */
    uint8_t status = get_status();
    printf("[SX1262] GetStatus bruto apos reset: 0x%02X\n", status);
    if (status == 0x00 || status == 0xFF) {
        printf("[ERRO] SPI nao esta se comunicando com o chip (MISO/CS/BUSY/alimentacao).\n");
        return false;
    }

    /* ── 7. Standby RC ───────────────────────────────────────────────── */
    set_standby_rc();
    sleep_ms(5);

    /* ── 8. Regulador DC-DC (mais eficiente que LDO) ─────────────────── */
    uint8_t reg_mode = 0x01;
    spi_write_cmd(SX1262_CMD_SET_REGULATOR_MODE, &reg_mode, 1);

    /* ── 9. DIO2 controla chave RF (antena TX/RX) automaticamente ──────
     *    Necessário para boards com chave RF controlada por DIO2.          */
    uint8_t dio2_rf = 0x01;
    spi_write_cmd(SX1262_CMD_SET_DIO2_AS_RF_SWITCH, &dio2_rf, 1);

#if SX1262_USE_TCXO
    /* ── 9b. Habilita o TCXO via DIO3 ────────────────────────────────────
     *    Sem isso o PLL de RF nunca sincroniza: BUSY continua respondendo
     *    normalmente (é lógica digital), mas TX/RX nunca completam e o
     *    chip acaba gerando IRQ_TIMEOUT em vez de TxDone/RxDone.          */
    uint32_t tcxo_delay_raw = SX1262_TCXO_DELAY_MS * 1000U * 64U / 1000U; /* unidades de 15.625 us */
    uint8_t tcxo_ctrl[4] = {
        SX1262_TCXO_VOLTAGE,
        (uint8_t)(tcxo_delay_raw >> 16),
        (uint8_t)(tcxo_delay_raw >> 8),
        (uint8_t)(tcxo_delay_raw)
    };
    spi_write_cmd(SX1262_CMD_SET_DIO3_AS_TCXO_CTRL, tcxo_ctrl, 4);

    /* Recalibração completa exigida após habilitar o TCXO (todos os blocos) */
    uint8_t cal_all = 0x7F;
    spi_write_cmd(SX1262_CMD_CALIBRATE, &cal_all, 1);
    sleep_ms(SX1262_TCXO_DELAY_MS + 5);
    wait_busy();
#endif

    /* ── 10. Calibração de imagem para a faixa 430-440 MHz ──────────────
     *    Valores de frequência da Tabela 9-2 do datasheet SX1262.          */
    uint8_t cal_img[2] = { 0x6B, 0x6F };
    spi_write_cmd(SX1262_CMD_CALIBRATE_IMAGE, cal_img, 2);
    sleep_ms(5);
    wait_busy();

    /* ── 11. Tipo de pacote: LoRa ────────────────────────────────────── */
    uint8_t pkt_type = SX1262_PACKET_TYPE_LORA;
    spi_write_cmd(SX1262_CMD_SET_PACKET_TYPE, &pkt_type, 1);

    /* ── 12. Frequência RF: 433 MHz ──────────────────────────────────── */
    set_rf_frequency(LORA_FREQ_HZ);

    /* ── 13. PA Config para SX1262 (até 22 dBm) ──────────────────────────
     *    paDutyCycle=0x04, hpMax=0x07, deviceSel=0x00(SX1262), paLut=0x01  */
    uint8_t pa_cfg[4] = { 0x04, 0x07, 0x00, 0x01 };
    spi_write_cmd(SX1262_CMD_SET_PA_CONFIG, pa_cfg, 4);

    /* ── 14. OCP: 140 mA (adequado para ≥ 22 dBm) ───────────────────── */
    uint8_t ocp = 0x38;
    write_register(SX1262_REG_OCP, &ocp, 1);

    /* ── 15. TX Params: potência + rampa ─────────────────────────────── */
    uint8_t tx_params[2] = {
        (uint8_t)(int8_t)LORA_TX_POWER_DBM,
        SX1262_RAMP_200U
    };
    spi_write_cmd(SX1262_CMD_SET_TX_PARAMS, tx_params, 2);

    /* ── 16. Parâmetros de modulação LoRa: SF, BW, CR, LDRO ─────────────
     *    LDRO=0: desabilitado (necessário apenas se ts > 16,38 ms,          *
     *            o que ocorre para SF ≥ 11 com BW=125 kHz)                 */
    uint8_t mod_params[4] = {
        (uint8_t)LORA_SF,
        (uint8_t)LORA_BW,
        (uint8_t)LORA_CR,
        0x00    /* LDRO desabilitado */
    };
    spi_write_cmd(SX1262_CMD_SET_MODULATION_PARAMS, mod_params, 4);

    /* Correção necessária para SF5/SF6 (seção 15.1 do datasheet) */
    if (LORA_SF == 5 || LORA_SF == 6) {
        uint8_t tmp;
        read_register(0x0889, &tmp, 1);
        tmp |= 0x04;
        write_register(0x0889, &tmp, 1);
    }

    /* ── 17. Parâmetros de pacote LoRa ───────────────────────────────────
     *    Header explícito (tamanho variável), CRC habilitado               */
    uint8_t pkt_params[6] = {
        (uint8_t)(LORA_PREAMBLE_LEN >> 8), /* Preamble MSB                */
        (uint8_t)(LORA_PREAMBLE_LEN),      /* Preamble LSB                */
        0x00,                               /* Header explícito (variável) */
        0xFF,                               /* Max payload (ajustado no TX)*/
        0x01,                               /* CRC habilitado              */
        0x00                                /* IQ normal (não invertido)   */
    };
    spi_write_cmd(SX1262_CMD_SET_PACKET_PARAMS, pkt_params, 6);

    /* ── 18. Sync word LoRa (rede privada = 0x1424) ──────────────────── */
    uint8_t sw[2] = { LORA_SYNC_WORD_PRI, LORA_SYNC_WORD_SEC };
    write_register(SX1262_REG_LORA_SYNC_WORD_MSB, sw, 2);

    /* ── 19. Base address do buffer TX e RX (ambos em 0x00) ─────────── */
    uint8_t base_addr[2] = { 0x00, 0x00 };
    spi_write_cmd(SX1262_CMD_SET_BUFFER_BASE_ADDR, base_addr, 2);

    /* ── 20. Configuração de IRQ: TxDone, RxDone, Timeout, CrcErr ───────
     *    Todos mapeados em DIO1                                             */
    uint16_t irq_mask = SX1262_IRQ_TX_DONE | SX1262_IRQ_RX_DONE
                      | SX1262_IRQ_TIMEOUT  | SX1262_IRQ_CRC_ERR;
    uint8_t irq_cfg[8] = {
        (uint8_t)(irq_mask >> 8), (uint8_t)(irq_mask),  /* irqMask  */
        (uint8_t)(irq_mask >> 8), (uint8_t)(irq_mask),  /* DIO1     */
        0x00, 0x00,                                       /* DIO2     */
        0x00, 0x00                                        /* DIO3     */
    };
    spi_write_cmd(SX1262_CMD_SET_DIO_IRQ_PARAMS, irq_cfg, 8);

    /* ── 21. Limpa quaisquer IRQs pendentes ─────────────────────────── */
    clear_irq(SX1262_IRQ_ALL);

    return true;
}

/* =========================================================================
 * Transmissão
 * ========================================================================= */
bool sx1262_send(const uint8_t *data, uint8_t len, uint32_t timeout_ms) {
    /* Volta ao modo Standby antes de configurar TX */
    set_standby_rc();

    /* Atualiza packet params com o tamanho real do payload */
    uint8_t pkt_params[6] = {
        (uint8_t)(LORA_PREAMBLE_LEN >> 8),
        (uint8_t)(LORA_PREAMBLE_LEN),
        0x00,   /* header explícito */
        len,    /* tamanho real do payload */
        0x01,   /* CRC habilitado */
        0x00    /* IQ normal */
    };
    spi_write_cmd(SX1262_CMD_SET_PACKET_PARAMS, pkt_params, 6);

    /* Copia dados para o buffer FIFO do rádio (offset 0x00) */
    write_buffer(0x00, data, len);

    /* Limpa IRQs anteriores e inicia transmissão */
    clear_irq(SX1262_IRQ_ALL);

    uint8_t tx_timeout[3];
    timeout_to_bytes(timeout_ms, tx_timeout);
    spi_write_cmd(SX1262_CMD_SET_TX, tx_timeout, 3);

    /* Aguarda DIO1 ficar HIGH (IRQ: TxDone ou Timeout) via polling */
    uint32_t start = to_ms_since_boot(get_absolute_time());
    while (!gpio_get(SX1262_DIO1_PIN)) {
        if (to_ms_since_boot(get_absolute_time()) - start > timeout_ms + 200U) {
            printf("[SX1262] DEBUG - DIO1 nunca subiu (nem TxDone nem Timeout do chip).\n");
            print_device_errors(sx1262_get_device_errors());
            return false; /* timeout de segurança do software */
        }
        tight_loop_contents();
    }

    uint16_t irq = get_irq_status();
    clear_irq(SX1262_IRQ_ALL);

    if (!(irq & SX1262_IRQ_TX_DONE)) {
        printf("[SX1262] DEBUG - IRQ bruto no fim do TX: 0x%04X\n", irq);
    }

    return (irq & SX1262_IRQ_TX_DONE) != 0;
}

/* =========================================================================
 * Recepção
 * ========================================================================= */
int sx1262_receive(uint8_t *buf, uint8_t *len, uint32_t timeout_ms) {
    /* Volta ao modo Standby antes de configurar RX */
    set_standby_rc();

    /* Packet params para RX: payload máximo */
    uint8_t pkt_params[6] = {
        (uint8_t)(LORA_PREAMBLE_LEN >> 8),
        (uint8_t)(LORA_PREAMBLE_LEN),
        0x00,   /* header explícito */
        0xFF,   /* max payload em RX */
        0x01,   /* CRC habilitado */
        0x00    /* IQ normal */
    };
    spi_write_cmd(SX1262_CMD_SET_PACKET_PARAMS, pkt_params, 6);

    /* Limpa IRQs e entra em modo RX com timeout configurado */
    clear_irq(SX1262_IRQ_ALL);

    uint8_t rx_timeout[3];
    timeout_to_bytes(timeout_ms, rx_timeout);
    spi_write_cmd(SX1262_CMD_SET_RX, rx_timeout, 3);

    /* Aguarda DIO1 (RxDone, Timeout ou CrcErr) */
    uint32_t start = to_ms_since_boot(get_absolute_time());
    while (!gpio_get(SX1262_DIO1_PIN)) {
        /* Timeout de segurança do software com 500 ms de margem */
        if (to_ms_since_boot(get_absolute_time()) - start > timeout_ms + 500U) {
            return SX1262_RX_TIMEOUT;
        }
        tight_loop_contents();
    }

    uint16_t irq = get_irq_status();
    clear_irq(SX1262_IRQ_ALL);

    /* Verifica o resultado do IRQ */
    if (irq & SX1262_IRQ_TIMEOUT) {
        return SX1262_RX_TIMEOUT;
    }
    if (irq & SX1262_IRQ_CRC_ERR) {
        return SX1262_RX_CRC_ERROR;
    }
    if (!(irq & SX1262_IRQ_RX_DONE)) {
        return SX1262_RX_ERROR;
    }

    /* Lê status do buffer RX: [payloadLen, rxStartBufferPointer] */
    uint8_t rx_status[2];
    spi_read_cmd(SX1262_CMD_GET_RX_BUFFER_STATUS, rx_status, 2);
    uint8_t payload_len   = rx_status[0];
    uint8_t rx_buf_offset = rx_status[1];

    if (payload_len == 0) {
        *len = 0;
        return 0;
    }

    /* Lê os dados do buffer FIFO */
    read_buffer(rx_buf_offset, buf, payload_len);
    *len = payload_len;

    /* Lê estatísticas do pacote: [rssiPkt, snrPkt, signalRssiPkt]
     * RSSI [dBm] = -rssiPkt / 2
     * SNR  [dB]  =  snrPkt  / 4  (int8_t com sinal)
     */
    uint8_t pkt_status[3];
    spi_read_cmd(SX1262_CMD_GET_PACKET_STATUS, pkt_status, 3);
    s_last_rssi = (int8_t)(-(int16_t)pkt_status[0] / 2);
    s_last_snr  = (int8_t)((int8_t)pkt_status[1]   / 4);

    return (int)payload_len;
}

/* =========================================================================
 * Getters
 * ========================================================================= */
int8_t sx1262_get_last_rssi(void) { return s_last_rssi; }
int8_t sx1262_get_last_snr(void)  { return s_last_snr;  }

uint16_t sx1262_get_device_errors(void) {
    uint8_t buf[2];
    spi_read_cmd(SX1262_CMD_GET_DEVICE_ERRORS, buf, 2);
    return ((uint16_t)buf[0] << 8) | buf[1];
}
