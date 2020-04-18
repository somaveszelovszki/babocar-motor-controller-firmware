#include <globals.hpp>

namespace globals {

bool useSafetyEnableSignal         = true;
micro::radian_t frontWheelOffset   = micro::radian_t(0);
micro::radian_t frontWheelMaxDelta = micro::radian_t(0);
micro::radian_t rearWheelOffset    = micro::radian_t(0);
micro::radian_t rearWheelMaxDelta  = micro::radian_t(0);
micro::radian_t extraServoOffset   = micro::radian_t(0);
micro::radian_t extraServoMaxDelta = micro::radian_t(0);

float MotorCtrl_P                  = 0.0f; // TODO
float MotorCtrl_I                  = 0.0f;
float MotorCtrl_D                  = 0.0f;
float MotorCtrl_integralMax        = 4.0f;

bool isRemoteControllerTaskOk      = false;
bool isVehicleCanTaskOk            = false;
bool isControlTaskOk               = false;

}  // namespace globals
