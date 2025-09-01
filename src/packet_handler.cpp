#include "vesc_nexus/packet_handler.hpp"

namespace VescPacket {
    std::vector<uint8_t> packDutyCycle(float duty, uint8_t can_id)
    {
        // ?????????? ?????? COMM_SET_DUTY
        return std::vector<uint8_t>();
    }

    bool unpackStatus(const uint8_t *data, int len, void *status)
    {
        // ??????? COMM_GET_VALUES
        return true;
    }
}
