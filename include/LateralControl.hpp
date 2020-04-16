#pragma once

#include <micro/utils/units.hpp>

struct LateralControl {
    micro::radian_t frontSteeringAngle;
    micro::radian_t rearSteeringAngle;
    micro::radian_t extraServoAngle;
};
