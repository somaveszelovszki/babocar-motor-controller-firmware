#pragma once

#include <cstdint>

struct RemoteControllerData {
    enum class channel_t : uint8_t {
        INVALID = 0,
        DirectControl,
        SafetyEnable
    };

    channel_t activeChannel = channel_t::INVALID;
    float acceleration      = 0.0f;
    float steering          = 0.0f;
};
