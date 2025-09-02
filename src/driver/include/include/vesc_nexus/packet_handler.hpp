#ifndef PACKET_HANDLER_HPP_
#define PACKET_HANDLER_HPP_

#include <vector>
#include <cstdint>

namespace VescPacket {
    std::vector<uint8_t> packDutyCycle(float duty, uint8_t can_id);
    bool unpackStatus(const uint8_t *data, int len, /* out */ void *status);
};

#endif // PACKET_HANDLER_HPP_
