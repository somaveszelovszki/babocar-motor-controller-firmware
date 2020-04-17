#pragma once

#include <micro/utils/units.hpp>

namespace globals {

extern bool useSafetyEnableSignal;
extern micro::radian_t frontSteeringServoOffset;
extern micro::radian_t rearSteeringServoOffset;
extern micro::radian_t extraServoOffset;

extern float MotorCtrl_P;
extern float MotorCtrl_I;
extern float MotorCtrl_D;
extern float MotorCtrl_integralMax;

extern bool isRemoteControllerTaskOk;
extern bool isVehicleCanTaskOk;
extern bool isControlTaskOk;

} // namespace globals
