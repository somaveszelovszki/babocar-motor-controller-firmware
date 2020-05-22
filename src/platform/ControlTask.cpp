#include <cfg_board.hpp>
#include <micro/debug/DebugLed.hpp>
#include <micro/debug/SystemManager.hpp>
#include <micro/hw/DC_Motor.hpp>
#include <micro/hw/Encoder.hpp>
#include <micro/hw/SteeringServo.hpp>
#include <micro/control/PID_Controller.hpp>
#include <micro/control/ramp.hpp>
#include <micro/panel/CanManager.hpp>
#include <micro/port/queue.hpp>
#include <micro/port/task.hpp>
#include <micro/utils/CarProps.hpp>
#include <micro/utils/state.hpp>
#include <micro/utils/timer.hpp>

#include <cfg_car.hpp>
#include <RemoteControllerData.hpp>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

using namespace micro;

extern queue_t<RemoteControllerData, 1> remoteControllerQueue;

CanManager vehicleCanManager(can_Vehicle, millisecond_t(50));

namespace {

bool useSafetyEnableSignal         = true;
micro::CarProps car                = micro::CarProps();
micro::radian_t frontWheelOffset   = micro::radian_t(0);
micro::radian_t frontWheelMaxDelta = micro::radian_t(0);
micro::radian_t rearWheelOffset    = micro::radian_t(0);
micro::radian_t rearWheelMaxDelta  = micro::radian_t(0);
micro::radian_t extraServoOffset   = micro::radian_t(0);
micro::radian_t extraServoMaxDelta = micro::radian_t(0);

struct LateralControl {
    micro::radian_t frontWheelAngle;
    micro::radian_t rearWheelAngle;
    micro::radian_t extraServoAngle;
};

struct LongitudinalControl {
    micro::m_per_sec_t speed;
    micro::millisecond_t rampTime;
};

struct ControlData {
    state_t<LateralControl> lat;
    state_t<LongitudinalControl> lon;
};

PID_Params speedControllerParams;

ramp_t<m_per_sec_t> speedRamp;

hw::DC_Motor dcMotor(tim_DC_Motor, timChnl_DC_Motor_Bridge1, timChnl_DC_Motor_Bridge2, cfg::MOTOR_MAX_DUTY);
hw::Encoder encoder(tim_Encoder);
PID_Controller speedController(speedControllerParams, cfg::MOTOR_MAX_DUTY, 0.01f);

hw::SteeringServo frontSteeringServo(tim_SteeringServo, timChnl_FrontSteeringServo, cfg::FRONT_STEERING_SERVO_PWM0, cfg::FRONT_STEERING_SERVO_PWM180,
    cfg::SERVO_MAX_ANGULAR_VELO, frontWheelOffset, frontWheelMaxDelta, cfg::SERVO_WHEEL_TRANSFER_RATE);

hw::SteeringServo rearSteeringServo(tim_SteeringServo, timChnl_RearSteeringServo, cfg::REAR_STEERING_SERVO_PWM0, cfg::REAR_STEERING_SERVO_PWM180,
    cfg::SERVO_MAX_ANGULAR_VELO, rearWheelOffset, rearWheelMaxDelta, cfg::SERVO_WHEEL_TRANSFER_RATE);

template <typename T>
bool hasControlTimedOut(const state_t<T>& control) {
    return getTime() - control.timestamp() > millisecond_t(50);
}

bool isSafetySignalOk(const RemoteControllerData& rc) {
    return rc.activeChannel == RemoteControllerData::channel_t::SafetyEnable && isBtw(rc.acceleration, 0.5f, 1.0f);
}

ControlData getControl(const ControlData& swControl, const state_t<RemoteControllerData>& remoteControl) {

    ControlData control = { { { radian_t(0), radian_t(0), radian_t(0) } }, { { m_per_sec_t(0), cfg::EMERGENCY_BRAKE_DURATION } } }; // emergency brake by default

    // remote control must be present, otherwise it means remote controller task or inter-task communication has died
    if (!hasControlTimedOut(remoteControl)) {

        const RemoteControllerData& rc = remoteControl.value();

        if (rc.activeChannel == RemoteControllerData::channel_t::DirectControl) {
            control.lat = {
                map(rc.steering, -1.0f, 1.0f, -frontSteeringServo.wheelMaxDelta(), frontSteeringServo.wheelMaxDelta()),
                map(rc.steering, -1.0f, 1.0f, rearSteeringServo.wheelMaxDelta(), -rearSteeringServo.wheelMaxDelta()),
                radian_t(0)
            };

            control.lon = {
                map(rc.acceleration, -1.0f, 1.0f, -cfg::DIRECT_CONTROL_MAX_SPEED, cfg::DIRECT_CONTROL_MAX_SPEED),
                millisecond_t(0)
            };
        } else if (!hasControlTimedOut(swControl.lat) && !hasControlTimedOut(swControl.lon) && (!useSafetyEnableSignal || isSafetySignalOk(rc))) {
            control = swControl;
        }
    }

    return control;
}

} // namespace

extern "C" void runControlTask(void) {

    SystemManager::instance().registerTask();

    canFrame_t rxCanFrame;
    CanFrameHandler vehicleCanFrameHandler;

    state_t<RemoteControllerData> remoteControl;
    ControlData swControl;

    vehicleCanFrameHandler.registerHandler(can::LateralControl::id(), [&swControl] (const uint8_t * const data) {
        LateralControl lateral;
        reinterpret_cast<const can::LateralControl*>(data)->acquire(lateral.frontWheelAngle, lateral.rearWheelAngle, lateral.extraServoAngle);
        swControl.lat = lateral;
    });

    vehicleCanFrameHandler.registerHandler(can::LongitudinalControl::id(), [&swControl] (const uint8_t * const data) {
        LongitudinalControl longitudinal;
        reinterpret_cast<const can::LongitudinalControl*>(data)->acquire(longitudinal.speed, useSafetyEnableSignal, longitudinal.rampTime);
        swControl.lon = longitudinal;
    });

    vehicleCanFrameHandler.registerHandler(can::SetMotorControlParams::id(), [] (const uint8_t * const data) {
        reinterpret_cast<const can::SetMotorControlParams*>(data)->acquire(speedControllerParams.P, speedControllerParams.I);
        speedController.tune(speedControllerParams);
    });

    vehicleCanFrameHandler.registerHandler(can::SetFrontWheelParams::id(), [] (const uint8_t * const data) {
        reinterpret_cast<const can::SetFrontWheelParams*>(data)->acquire(frontWheelOffset, frontWheelMaxDelta);
        vehicleCanManager.send(can::FrontWheelParams(frontWheelOffset, frontWheelMaxDelta));
    });

    vehicleCanFrameHandler.registerHandler(can::SetRearWheelParams::id(), [] (const uint8_t * const data) {
        reinterpret_cast<const can::SetRearWheelParams*>(data)->acquire(rearWheelOffset, rearWheelMaxDelta);
        vehicleCanManager.send(can::RearWheelParams(rearWheelOffset, rearWheelMaxDelta));
    });

    const CanManager::subscriberId_t vehicleCanSubsciberId = vehicleCanManager.registerSubscriber(vehicleCanFrameHandler.identifiers());

    while (true) {
        while (vehicleCanManager.read(vehicleCanSubsciberId, rxCanFrame)) {
            vehicleCanFrameHandler.handleFrame(rxCanFrame);
        }

        RemoteControllerData remoteControlData;
        if (remoteControllerQueue.receive(remoteControlData, millisecond_t(0))) {
            remoteControl = remoteControlData;
        }

        frontSteeringServo.setWheelOffset(frontWheelOffset);
        frontSteeringServo.setWheelMaxDelta(frontWheelMaxDelta);
        rearSteeringServo.setWheelOffset(rearWheelOffset);
        rearSteeringServo.setWheelMaxDelta(rearWheelMaxDelta);

        const ControlData validControl = getControl(swControl, remoteControl);

        speedController.desired = speedRamp.update(car.speed, validControl.lon.value().speed, validControl.lon.value().rampTime).get();
        frontSteeringServo.writeWheelAngle(validControl.lat.value().frontWheelAngle);
        rearSteeringServo.writeWheelAngle(validControl.lat.value().rearWheelAngle);

        car.frontWheelAngle = frontSteeringServo.wheelAngle();
        car.rearWheelAngle  = rearSteeringServo.wheelAngle();

        SystemManager::instance().notify(!hasControlTimedOut(swControl.lat) && !hasControlTimedOut(swControl.lon) && !hasControlTimedOut(remoteControl));
        os_sleep(millisecond_t(1));
    }
}

void tim_ControlLoop_PeriodElapsedCallback() {

    static millisecond_t lastUpdateTime = getExactTime();

    const millisecond_t now = getExactTime();
    encoder.update();

    car.speed = encoder.lastDiff() * cfg::ENCODER_INCR_DISTANCE / (now - lastUpdateTime);
    car.distance = encoder.numIncr() * cfg::ENCODER_INCR_DISTANCE;

    speedController.update(car.speed.get());
    dcMotor.write(speedController.output());

    lastUpdateTime = now;
}

void micro_Vehicle_Can_RxFifoMsgPendingCallback() {
    vehicleCanManager.onFrameReceived();
}
