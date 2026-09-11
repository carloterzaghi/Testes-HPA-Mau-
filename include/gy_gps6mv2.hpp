#ifndef GY_GPS6MV2_HPP
#define GY_GPS6MV2_HPP

#include <stdint.h>
#include <stddef.h>

#include "hardware/uart.h"

#define GY_GPS6MV2_DEFAULT_BAUD_RATE 9600

typedef struct {
    bool valid;
    int32_t latitude_e6;
    int32_t longitude_e6;
    int32_t altitude_mm;
    uint16_t speed_knots_e2;
    uint16_t course_degrees_e2;
    uint8_t satellites;
} gy_gps6mv2_data_t;

class GYGPS6MV2 {
public:
    GYGPS6MV2(uart_inst_t *uart, uint8_t tx_pin, uint8_t rx_pin,
              uint32_t baud_rate = GY_GPS6MV2_DEFAULT_BAUD_RATE);

    bool update(uint32_t timeout_ms = 1000);

    bool read_sentence(char *buffer, size_t buffer_size, uint32_t timeout_ms = 1000);

    gy_gps6mv2_data_t data;

private:
    uart_inst_t *_uart;
    uint8_t _tx_pin;
    uint8_t _rx_pin;

    bool _parse_sentence(char *sentence);
    bool _parse_gga(char **fields, uint8_t field_count);
    bool _parse_rmc(char **fields, uint8_t field_count);
};

#endif