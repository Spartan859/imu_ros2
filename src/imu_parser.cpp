#include "imu_ros2/imu_parser.hpp"
#include <cstring>
#include <rclcpp/rclcpp.hpp>

ImuParser::ImuParser() 
    : state(0), payload_len(0), crc_calculated(0), crc_received(0), data_buffer() {
    acc[0] = acc[1] = acc[2] = 0.0;
    gyr[0] = gyr[1] = gyr[2] = 0.0;
    eul[0] = eul[1] = eul[2] = 0.0;
}

uint16_t ImuParser::_crc16_compute(const std::vector<uint8_t>& data_bytes, uint16_t init) {
    uint16_t crc = init & 0xFFFF;
    for (size_t i = 0; i < data_bytes.size(); ++i) {
        crc ^= (data_bytes[i] << 8) & 0xFFFF;
        for (int j = 0; j < 8; ++j) {
            uint16_t temp = (crc << 1) & 0xFFFF;
            if (crc & 0x8000) {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    }
    return crc & 0xFFFF;
}

uint16_t ImuParser::_crc16_modbus(const std::vector<uint8_t>& data_bytes, uint16_t init) {
    uint16_t crc = init & 0xFFFF;
    for (size_t i = 0; i < data_bytes.size(); ++i) {
        crc ^= data_bytes[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc & 0xFFFF;
}

void ImuParser::_parse_payload(const std::vector<uint8_t>& payload) {
    size_t length = payload.size();
    size_t offset = 0;
    while (offset < length) {
        uint8_t tag = payload[offset];
        if (tag == 0xA0 && offset + 7 <= length) {
            int16_t ax, ay, az;
            std::memcpy(&ax, &payload[offset + 1], sizeof(ax));
            std::memcpy(&ay, &payload[offset + 3], sizeof(ay));
            std::memcpy(&az, &payload[offset + 5], sizeof(az));
            acc[0] = ax / 1000.0;
            acc[1] = ay / 1000.0;
            acc[2] = az / 1000.0;
            offset += 7;
        } else if (tag == 0xB0 && offset + 7 <= length) {
            int16_t gx, gy, gz;
            std::memcpy(&gx, &payload[offset + 1], sizeof(gx));
            std::memcpy(&gy, &payload[offset + 3], sizeof(gy));
            std::memcpy(&gz, &payload[offset + 5], sizeof(gz));
            gyr[0] = gx / 10.0;
            gyr[1] = gy / 10.0;
            gyr[2] = gz / 10.0;
            offset += 7;
        } else if (tag == 0xD0 && offset + 7 <= length) {
            int16_t t0, t1, t2;
            std::memcpy(&t0, &payload[offset + 1], sizeof(t0));
            std::memcpy(&t1, &payload[offset + 3], sizeof(t1));
            std::memcpy(&t2, &payload[offset + 5], sizeof(t2));
            eul[0] = t1 / 100.0; // pitch
            eul[1] = t0 / 100.0; // roll
            eul[2] = t2 / 10.0;  // yaw
            offset += 7;
        } else if (tag == 0x91 && offset + 76 <= length) {
            const uint8_t* data = &payload[offset];
            std::memcpy(acc.data(), &data[12], sizeof(float) * 3);
            std::memcpy(gyr.data(), &data[24], sizeof(float) * 3);
            std::memcpy(eul.data(), &data[48], sizeof(float) * 3);
            offset += 76;
        } else {
            offset += 1;
        }
    }
}

void ImuParser::feed(uint8_t byte) {
    // 状态机实现，参考 Python 版逻辑
    if (state == 0) {
        if (byte == 0x5A) {
            header.clear();
            header.push_back(0x5A);
            state = 1;
        }
    } else if (state == 1) {
        type = byte;
        header.push_back(byte);
        if (type == 0xA5) {
            state = 2;
        } else {
            state = 0;
        }
    } else if (state == 2) {
        payload_len = byte;
        header.push_back(byte);
        state = 3;
    } else if (state == 3) {
        payload_len |= (static_cast<uint16_t>(byte) << 8);
        header.push_back(byte);
        state = 4;
    } else if (state == 4) {
        crc_received = byte;
        state = 5;
    } else if (state == 5) {
        crc_received |= (static_cast<uint16_t>(byte) << 8);
        data_buffer.clear();
        if (payload_len > 0) {
            state = 6;
        } else {
            uint16_t calc = _crc16_compute(header, 0);
            if (calc != crc_received) {
                RCLCPP_WARN(rclcpp::get_logger("ImuParser"), "CRC error: recv=%04X calc=%04X", crc_received, calc);
            }
            state = 0;
        }
    } else if (state == 6) {
        data_buffer.push_back(byte);
        if (data_buffer.size() >= payload_len && type == 0xA5) {
            std::vector<uint8_t> combined = header;
            combined.insert(combined.end(), data_buffer.begin(), data_buffer.end());
            uint16_t calc = _crc16_compute(combined, 0);
            if (calc == crc_received) {
                _parse_payload(data_buffer);
            } else {
                uint16_t calc_ff = _crc16_compute(combined, 0xFFFF);
                uint16_t calc_modbus = _crc16_modbus(combined, 0xFFFF);
                RCLCPP_ERROR(rclcpp::get_logger("ImuParser"),
                    "CRC error:\n  recv CRC   = %04X\n  calc(init=0)= %04X\n  calc(init=0xFFFF)= %04X\n  calc(modbus)= %04X",
                    crc_received, calc, calc_ff, calc_modbus);
            }
            state = 0;
        }
    }
}