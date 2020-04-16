#include <globals.hpp>

namespace globals {

bool useSafetyEnableSignal               = true;
micro::radian_t frontSteeringServoOffset = micro::radian_t(0);
micro::radian_t rearSteeringServoOffset  = micro::radian_t(0);
micro::radian_t extraServoOffset         = micro::radian_t(0);

bool isRemoteControllerTaskOk            = false;
bool isVehicleCanTaskOk                  = false;
bool isControlTaskOk                     = false;

}  // namespace globals
