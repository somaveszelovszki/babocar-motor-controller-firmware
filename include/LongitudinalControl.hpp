#pragma once

#include <micro/utils/units.hpp>

struct LongitudinalControl {
    micro::m_per_sec_t speed;
    micro::millisecond_t rampTime;
};
