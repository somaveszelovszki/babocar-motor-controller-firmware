#pragma once

#include <micro/utils/units.hpp>

struct LateralControl {
    micro::radian_t frontWheelAngle;
    micro::radian_t rearWheelAngle;
    micro::radian_t extraServoAngle;
};
