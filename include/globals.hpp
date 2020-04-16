#pragma once

#include <micro/utils/units.hpp>

namespace globals {

extern bool useSafetyEnableSignal;
extern micro::radian_t frontSteeringServoOffset;
extern micro::radian_t rearSteeringServoOffset;
extern micro::radian_t extraServoOffset;

extern bool isRemoteControllerTaskOk;
extern bool isVehicleCanTaskOk;
extern bool isControlTaskOk;

} // namespace globals
