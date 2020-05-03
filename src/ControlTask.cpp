#include <micro/hw/DC_Motor.hpp>
#include <micro/hw/Encoder.hpp>
#include <micro/hw/SteeringServo.hpp>
#include <micro/control/PID_Controller.hpp>
#include <micro/port/task.hpp>
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

hw::SteeringServo frontSteeringServo(tim_SteeringServo, timChnl_FrontSteeringServo, cfg::FRONT_STEERING_SERVO_PWM0, cfg::FRONT_STEERING_SERVO_PWM180,
    cfg::SERVO_MAX_ANGULAR_VELO, globals::frontWheelOffset, globals::frontWheelMaxDelta, cfg::SERVO_WHEEL_TRANSFER_RATE);

hw::SteeringServo rearSteeringServo(tim_SteeringServo, timChnl_RearSteeringServo, cfg::REAR_STEERING_SERVO_PWM0, cfg::REAR_STEERING_SERVO_PWM180,
    cfg::SERVO_MAX_ANGULAR_VELO, globals::rearWheelOffset, globals::rearWheelMaxDelta, cfg::SERVO_WHEEL_TRANSFER_RATE);

} // namespace

extern "C" void runControlTask(void) {

    micro::waitReady(lateralControlQueue);
    micro::waitReady(longitudinalControlQueue);
    micro::waitReady(remoteControllerQueue);

    LateralControl lateralControl;
    LongitudinalControl longitudinalControl;
    RemoteControllerData remoteControllerData;

    struct {
        m_per_sec_t startSpeed, targetSpeed;
        microsecond_t startTime, duration;
    } speedRamp;

    WatchdogTimer lateralWd(millisecond_t(50)), longitudinalWd(millisecond_t(50)), remoteControlWd(millisecond_t(50));

    while (true) {
        globals::isControlTaskOk = !lateralWd.hasTimedOut() && !longitudinalWd.hasTimedOut() && !remoteControlWd.hasTimedOut();

        if (xQueueReceive(lateralControlQueue, &lateralControl, 0)) {
            lateralWd.reset();
        }

        if (xQueueReceive(longitudinalControlQueue, &longitudinalControl, 0)) {
            longitudinalWd.reset();
        }

        if (xQueueReceive(remoteControllerQueue, &remoteControllerData, 0)) {
            remoteControlWd.reset();
        }

        frontSteeringServo.setWheelOffset(globals::frontWheelOffset);
        frontSteeringServo.setWheelMaxDelta(globals::frontWheelMaxDelta);
        rearSteeringServo.setWheelOffset(globals::rearWheelOffset);
        rearSteeringServo.setWheelMaxDelta(globals::rearWheelMaxDelta);

        switch (remoteControllerData.activeChannel) {

        case RemoteControllerData::channel_t::DirectControl:
            speedRamp.startSpeed = speedRamp.targetSpeed = map(remoteControllerData.acceleration, -1.0f, 1.0f, -cfg::DIRECT_CONTROL_MAX_SPEED, cfg::DIRECT_CONTROL_MAX_SPEED);
            speedRamp.startTime  = getExactTime();
            speedRamp.duration   = millisecond_t(0);

            frontSteeringServo.writeWheelAngle(map(remoteControllerData.steering, -1.0f, 1.0f, -frontSteeringServo.wheelMaxDelta(), frontSteeringServo.wheelMaxDelta()));
            rearSteeringServo.writeWheelAngle(map(remoteControllerData.steering, -1.0f, 1.0f, rearSteeringServo.wheelMaxDelta(), -rearSteeringServo.wheelMaxDelta()));
            break;

        case RemoteControllerData::channel_t::SafetyEnable:
            if (globals::isControlTaskOk && isBtw(remoteControllerData.acceleration, 0.5f, 1.0f)) {
                if (longitudinalControl.speed != speedRamp.targetSpeed || longitudinalControl.rampTime != speedRamp.duration) {
                    speedRamp.startSpeed  = globals::car.speed;
                    speedRamp.targetSpeed = longitudinalControl.speed;
                    speedRamp.startTime   = getExactTime();
                    speedRamp.duration    = longitudinalControl.rampTime;
                }

                frontSteeringServo.writeWheelAngle(lateralControl.frontWheelAngle);
                rearSteeringServo.writeWheelAngle(lateralControl.rearWheelAngle);
            } else {
                if (speedRamp.targetSpeed != m_per_sec_t(0) || speedRamp.duration != cfg::EMERGENCY_BRAKE_DURATION) {
                    speedRamp.startSpeed  = globals::car.speed;
                    speedRamp.targetSpeed = m_per_sec_t(0);
                    speedRamp.startTime   = getExactTime();
                    speedRamp.duration    = cfg::EMERGENCY_BRAKE_DURATION;
                }

                frontSteeringServo.writeWheelAngle(radian_t(0));
                rearSteeringServo.writeWheelAngle(radian_t(0));
            }
            break;

        default:
            break;
        }

        speedCtrl.desired = map(getExactTime(), speedRamp.startTime, speedRamp.startTime + speedRamp.duration, speedRamp.startSpeed, speedRamp.targetSpeed).get();

        globals::car.frontWheelAngle = frontSteeringServo.wheelAngle();
        globals::car.rearWheelAngle  = rearSteeringServo.wheelAngle();

        vTaskDelay(1);
    }

    vTaskDelete(nullptr);
}

void tim_ControlLoop_PeriodElapsedCallback() {

    static millisecond_t lastUpdateTime = getExactTime();

    const millisecond_t now = getExactTime();
    encoder.update();

    globals::car.speed = encoder.lastDiff() * cfg::ENCODER_INCR_DISTANCE / (now - lastUpdateTime);
    globals::car.distance = encoder.numIncr() * cfg::ENCODER_INCR_DISTANCE;

    speedCtrl.update(globals::car.speed.get());
    dcMotor.write(speedCtrl.output());

    lastUpdateTime = now;
}
