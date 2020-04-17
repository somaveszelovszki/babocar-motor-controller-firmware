#include <micro/hw/DC_Motor.hpp>
#include <micro/hw/Encoder.hpp>
#include <micro/control/PID_Controller.hpp>
#include <micro/utils/timer.hpp>

#include <cfg_board.h>
#include <cfg_car.hpp>
#include <globals.hpp>
#include <LateralControl.hpp>
#include <LongitudinalControl.hpp>
#include <RemoteControllerData.hpp>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

using namespace micro;

extern QueueHandle_t lateralControlQueue;
extern QueueHandle_t longitudinalControlQueue;
extern QueueHandle_t remoteControllerQueue;

namespace {

hw::DC_Motor dcMotor(tim_DC_Motor, timChnl_DC_Motor_Bridge1, timChnl_DC_Motor_Bridge2, cfg::MOTOR_MAX_DUTY);
hw::Encoder encoder(tim_Encoder);
PID_Controller speedCtrl(globals::MotorCtrl_P, globals::MotorCtrl_I, globals::MotorCtrl_D, globals::MotorCtrl_integralMax, -cfg::MOTOR_MAX_DUTY, cfg::MOTOR_MAX_DUTY, 0.01f);

m_per_sec_t speed;
meter_t distance;

} // namespace

extern "C" void runControlTask(void) {

    LateralControl lateralControl;
    LongitudinalControl longitudinalControl;
    RemoteControllerData remoteControllerData;

    WatchdogTimer lateralWd(millisecond_t(50)), longitudinalWd(millisecond_t(50)), remoteControlWd(millisecond_t(50));

    while (true) {
        globals::isControlTaskOk = !lateralWd.hasTimedOut() && !longitudinalWd.hasTimedOut() && !remoteControlWd.hasTimedOut();

        if (xQueueReceive(lateralControlQueue, &lateralControl, 0)) {
            lateralWd.reset();
        }

        if (xQueueReceive(longitudinalControlQueue, &lateralControl, 0)) {
            longitudinalWd.reset();
        }

        if (xQueueReceive(remoteControllerQueue, &remoteControllerData, 0)) {
            remoteControlWd.reset();
        }

        // TODO determine which input is active

        vTaskDelay(1);
    }

    vTaskDelete(nullptr);
}

void tim_ControlLoop_PeriodElapsedCallback() {

    static millisecond_t lastUpdateTime = getExactTime();

    const millisecond_t now = getExactTime();
    encoder.update();

    speed = encoder.lastDiff() * cfg::ENCODER_INCR_DISTANCE / (now - lastUpdateTime);
    distance = encoder.numIncr() * cfg::ENCODER_INCR_DISTANCE;

    speedCtrl.update(speed.get());
    dcMotor.write(speedCtrl.output());

    lastUpdateTime = now;
}
