#include "gy_gps6mv2.hpp"

#include <string.h>

#include "pico/stdlib.h"

namespace {

constexpr uint8_t sentence_buffer_size = 96;

bool parse_unsigned(const char *text, uint32_t *value) {
    if (!text || !*text) {
        return false;
    }

    uint32_t result = 0;
    for (const char *cursor = text; *cursor; ++cursor) {
        if (*cursor < '0' || *cursor > '9') {
            return false;
        }
        result = result * 10U + static_cast<uint32_t>(*cursor - '0');
    }
    *value = result;
    return true;
}

bool parse_scaled(const char *text, uint32_t scale, uint64_t *value) {
    if (!text || !*text) {
        return false;
    }

    uint64_t whole = 0;
    uint64_t fraction = 0;
    uint64_t fraction_scale = 1;
    bool after_decimal = false;

    for (const char *cursor = text; *cursor; ++cursor) {
        if (*cursor == '.') {
            if (after_decimal) {
                return false;
            }
            after_decimal = true;
        } else if (*cursor >= '0' && *cursor <= '9') {
            if (after_decimal) {
                if (fraction_scale < 100000000U) {
                    fraction = fraction * 10U + static_cast<uint64_t>(*cursor - '0');
                    fraction_scale *= 10U;
                }
            } else {
                whole = whole * 10U + static_cast<uint64_t>(*cursor - '0');
            }
        } else {
            return false;
        }
    }

    *value = whole * scale + (fraction * scale) / fraction_scale;
    return true;
}

bool parse_coordinate(const char *text, char hemisphere, int32_t *value_e6) {
    uint64_t raw_e6 = 0;
    if (!parse_scaled(text, 1000000U, &raw_e6)) {
        return false;
    }

    uint64_t degrees = raw_e6 / 100000000U;
    uint64_t minutes_e6 = raw_e6 % 100000000U;
    int32_t coordinate = static_cast<int32_t>(degrees * 1000000U +
                                               minutes_e6 / 60U);
    if (hemisphere == 'S' || hemisphere == 'W') {
        coordinate = -coordinate;
    }
    *value_e6 = coordinate;
    return true;
}

bool valid_checksum(char *sentence) {
    if (sentence[0] != '$') {
        return false;
    }

    char *checksum_marker = strchr(sentence, '*');
    if (!checksum_marker || strlen(checksum_marker) < 3) {
        return false;
    }

    uint8_t checksum = 0;
    for (char *cursor = sentence + 1; cursor < checksum_marker; ++cursor) {
        checksum ^= static_cast<uint8_t>(*cursor);
    }

    uint32_t expected = 0;
    if (checksum_marker[1] >= '0' && checksum_marker[1] <= '9') {
        expected = static_cast<uint32_t>(checksum_marker[1] - '0') << 4;
    } else if (checksum_marker[1] >= 'A' && checksum_marker[1] <= 'F') {
        expected = static_cast<uint32_t>(checksum_marker[1] - 'A' + 10) << 4;
    } else {
        return false;
    }
    if (checksum_marker[2] >= '0' && checksum_marker[2] <= '9') {
        expected |= static_cast<uint32_t>(checksum_marker[2] - '0');
    } else if (checksum_marker[2] >= 'A' && checksum_marker[2] <= 'F') {
        expected |= static_cast<uint32_t>(checksum_marker[2] - 'A' + 10);
    } else {
        return false;
    }
    return checksum == expected;
}

} // namespace

GYGPS6MV2::GYGPS6MV2(uart_inst_t *uart, uint8_t tx_pin, uint8_t rx_pin,
                     uint32_t baud_rate)
    : _uart(uart), _tx_pin(tx_pin), _rx_pin(rx_pin), data{} {
    uart_init(_uart, baud_rate);
    gpio_set_function(_tx_pin, GPIO_FUNC_UART);
    gpio_set_function(_rx_pin, GPIO_FUNC_UART);
}

bool GYGPS6MV2::update(uint32_t timeout_ms) {
    char sentence[sentence_buffer_size];
    uint32_t start = to_ms_since_boot(get_absolute_time());

    while (to_ms_since_boot(get_absolute_time()) - start < timeout_ms) {
        if (read_sentence(sentence, sizeof(sentence), timeout_ms)) {
            if (_parse_sentence(sentence)) {
                return true;
            }
        }
    }
    return false;
}

bool GYGPS6MV2::read_sentence(char *buffer, size_t buffer_size,
                              uint32_t timeout_ms) {
    if (!buffer || buffer_size < 4) {
        return false;
    }

    uint8_t length = 0;
    uint32_t start = to_ms_since_boot(get_absolute_time());

    while (to_ms_since_boot(get_absolute_time()) - start < timeout_ms) {
        if (!uart_is_readable(_uart)) {
            tight_loop_contents();
            continue;
        }

        char character = static_cast<char>(uart_getc(_uart));
        if (character == '$') {
            length = 0;
            buffer[length++] = character;
        } else if (length > 0 && length < buffer_size - 1) {
            buffer[length++] = character;
            if (character == '\n') {
                buffer[length] = '\0';
                return true;
            }
        }
    }
    return false;
}

bool GYGPS6MV2::_parse_sentence(char *sentence) {
    if (!valid_checksum(sentence)) {
        return false;
    }

    char *checksum_marker = strchr(sentence, '*');
    *checksum_marker = '\0';
    char *fields[20] = {};
    uint8_t field_count = 0;
    char *field_start = sentence + 1;
    while (field_count < 20) {
        fields[field_count++] = field_start;
        char *separator = strpbrk(field_start, ",\r\n");
        if (!separator) {
            break;
        }
        *separator = '\0';
        if (separator[1] == '\r' || separator[1] == '\n') {
            break;
        }
        field_start = separator + 1;
    }

    if (field_count == 0) {
        return false;
    }
    if (!strcmp(fields[0], "GPGGA") || !strcmp(fields[0], "GNGGA")) {
        return _parse_gga(fields, field_count);
    }
    if (!strcmp(fields[0], "GPRMC") || !strcmp(fields[0], "GNRMC")) {
        return _parse_rmc(fields, field_count);
    }
    return false;
}

bool GYGPS6MV2::_parse_gga(char **fields, uint8_t field_count) {
    if (field_count < 10 || fields[6][0] == '0' || !fields[2][0] || !fields[4][0]) {
        return false;
    }

    int32_t latitude = 0;
    int32_t longitude = 0;
    uint32_t satellites = 0;
    uint64_t altitude_cm = 0;
    if (!parse_coordinate(fields[2], fields[3][0], &latitude) ||
        !parse_coordinate(fields[4], fields[5][0], &longitude) ||
        !parse_unsigned(fields[7], &satellites) ||
        !parse_scaled(fields[9], 100U, &altitude_cm)) {
        return false;
    }

    data.valid = true;
    data.latitude_e6 = latitude;
    data.longitude_e6 = longitude;
    data.satellites = static_cast<uint8_t>(satellites);
    data.altitude_mm = static_cast<int32_t>(altitude_cm * 10U);
    return true;
}

bool GYGPS6MV2::_parse_rmc(char **fields, uint8_t field_count) {
    if (field_count < 9 || fields[2][0] != 'A') {
        return false;
    }

    int32_t latitude = 0;
    int32_t longitude = 0;
    uint64_t speed = 0;
    uint64_t course = 0;
    if (!parse_coordinate(fields[3], fields[4][0], &latitude) ||
        !parse_coordinate(fields[5], fields[6][0], &longitude) ||
        !parse_scaled(fields[7], 100U, &speed) ||
        !parse_scaled(fields[8], 100U, &course)) {
        return false;
    }

    data.valid = true;
    data.latitude_e6 = latitude;
    data.longitude_e6 = longitude;
    data.speed_knots_e2 = static_cast<uint16_t>(speed);
    data.course_degrees_e2 = static_cast<uint16_t>(course);
    return true;
}