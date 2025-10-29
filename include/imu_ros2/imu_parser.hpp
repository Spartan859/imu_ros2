#ifndef IMU_PARSER_HPP
#define IMU_PARSER_HPP

#include <cstdint>
#include <array>
#include <vector>

class ImuParser {
public:
    ImuParser();

    void feed(uint8_t byte);
    std::array<float, 3> acc;
    std::array<float, 3> gyr;
    std::array<float, 3> eul;

private:
    int state;
    uint16_t payload_len;
    uint16_t crc_calculated;
    uint16_t crc_received;
    std::vector<uint8_t> data_buffer;

    static uint16_t _crc16_compute(const std::vector<uint8_t>& data_bytes, uint16_t init = 0);
    static uint16_t _crc16_modbus(const std::vector<uint8_t>& data_bytes, uint16_t init = 0xFFFF);
    void _parse_payload(const std::vector<uint8_t>& payload);
};

#endif // IMU_PARSER_HPP