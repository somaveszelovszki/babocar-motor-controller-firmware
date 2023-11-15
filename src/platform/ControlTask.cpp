#include <cfg_board.hpp>
#include <micro/debug/DebugLed.hpp>
#include <micro/debug/SystemManager.hpp>
#include <micro/hw/DcMotor.hpp>
#include <micro/hw/Encoder.hpp>
#include <micro/hw/ServoMotor.hpp>
#include <micro/control/PID_Controller.hpp>
#include <micro/control/ramp.hpp>
#include <micro/panel/CanManager.hpp>
#include <micro/port/queue.hpp>
#include <micro/port/task.hpp>
#include <micro/port/task.hpp>
#include <micro/utils/CarProps.hpp>
#include <micro/utils/state.hpp>
#include <micro/utils/timer.hpp>

#include <cfg_car.hpp>
#include <RemoteControllerData.hpp>

using namespace micro;

extern queue_t<RemoteControllerData, 1> remoteControllerQueue;

bool useSafetyEnableSignal = true;

namespace {

CanManager vehicleCanManager(can_Vehicle);

bool isRemoteControlled = false;

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

PID_Params speedControllerParams = { 0.0f, 0.0f, 0.0f };
CarProps car;
ramp_t<m_per_sec_t> speedRamp;

hw::BridgeDcMotor dcMotor(tim_DC_Motor, timChnl_DC_Motor_Bridge1, timChnl_DC_Motor_Bridge2, cfg::MOTOR_MAX_DUTY);
hw::Encoder encoder(tim_Encoder);
PID_Controller speedController(speedControllerParams, 1.0f, 0.1f, 0.005f);

hw::ServoMotor frontSteeringServo(tim_SteeringServo, timChnl_FrontSteeringServo, cfg::FRONT_STEERING_SERVO_PWM_CENTER,
    cfg::FRONT_STEERING_SERVO_POSITIVE_TRANSFER_RATE, cfg::FRONT_STEERING_SERVO_NEGATIVE_TRANSFER_RATE,
    cfg::FRONT_WHEEL_MAX_DELTA_ANGLE, cfg::FRONT_SERVO_MAX_ANGULAR_VELOCITY);

hw::ServoMotor rearSteeringServo(tim_SteeringServo, timChnl_RearSteeringServo, cfg::REAR_STEERING_SERVO_PWM_CENTER,
    cfg::REAR_STEERING_SERVO_POSITIVE_TRANSFER_RATE, cfg::REAR_STEERING_SERVO_NEGATIVE_TRANSFER_RATE,
    cfg::REAR_WHEEL_MAX_DELTA_ANGLE, cfg::REAR_SERVO_MAX_ANGULAR_VELOCITY);

CanFrameHandler vehicleCanFrameHandler;
CanSubscriber::Id vehicleCanSubscriberId = CanSubscriber::INVALID_ID;

state_t<RemoteControllerData> remoteControl;
ControlData swControl;

template <typename T>
bool hasControlTimedOut(const state_t<T>& control) {
    return getTime() - control.timestamp() > millisecond_t(50);
}

bool isSafetySignalOk(const RemoteControllerData& rc) {
    return rc.activeChannel == RemoteControllerData::channel_t::SafetyEnable && isBtw(rc.acceleration, 0.5f, 1.0f);
}

bool isSafetySignalSmoothBrake(const RemoteControllerData& rc) {
    return rc.activeChannel == RemoteControllerData::channel_t::SafetyEnable && isBtw(rc.acceleration, -0.25f, 0.5f);
}

ControlData getControl(const ControlData& swControl, const state_t<RemoteControllerData>& remoteControl) {

    // emergency brake by default
    ControlData control = {
        { { radian_t(0), radian_t(0), radian_t(0) } },
        { { m_per_sec_t(0), cfg::EMERGENCY_BRAKE_DURATION } }
    };

    // remote control must be present, otherwise it means remote controller task or inter-task communication has died
    if (!hasControlTimedOut(remoteControl)) {

        const RemoteControllerData& rc = remoteControl.value();

        isRemoteControlled = RemoteControllerData::channel_t::DirectControl == rc.activeChannel;

        if (RemoteControllerData::channel_t::DirectControl == rc.activeChannel) {
            control.lat = {
                micro::lerp(rc.steering, -1.0f, 1.0f, -frontSteeringServo.maxValue(), frontSteeringServo.maxValue()),
                micro::lerp(rc.steering, -1.0f, 1.0f, rearSteeringServo.maxValue(), -rearSteeringServo.maxValue()),
                radian_t(0)
            };

            control.lon = {
                micro::lerp(rc.acceleration, -1.0f, 1.0f, -cfg::DIRECT_CONTROL_MAX_SPEED, cfg::DIRECT_CONTROL_MAX_SPEED),
                millisecond_t(0)
            };
        } else if (!hasControlTimedOut(swControl.lat) && !hasControlTimedOut(swControl.lon)) {

            // enables full control when safety signal is received
            // enables lateral control only when safety signal is not received but car is moving
            // (in order to keep car on the line after safety signal is lost)

            if (!useSafetyEnableSignal || isSafetySignalOk(rc)) {
                control = swControl;
            } else {
                // enables lateral software control, but does not change default longitudinal behaviour, which is emergency brake
                control.lat = swControl.lat;
                control.lon = { { m_per_sec_t(0), isSafetySignalSmoothBrake(rc) ? cfg::SMOOTH_BRAKE_DURATION : cfg::EMERGENCY_BRAKE_DURATION } };
            }
        } // else: does not change default behaviour, which is emergency brake
    }

    return control;
}

void initializeVehicleCan() {
    vehicleCanFrameHandler.registerHandler(can::LateralControl::id(), [] (const uint8_t * const data) {
        LateralControl lateral;
        reinterpret_cast<const can::LateralControl*>(data)->acquire(lateral.frontWheelAngle, lateral.rearWheelAngle, lateral.extraServoAngle);
        swControl.lat = lateral;
    });

    vehicleCanFrameHandler.registerHandler(can::LongitudinalControl::id(), [] (const uint8_t * const data) {
        LongitudinalControl longitudinal;
        reinterpret_cast<const can::LongitudinalControl*>(data)->acquire(longitudinal.speed, useSafetyEnableSignal, longitudinal.rampTime);
        swControl.lon = longitudinal;
    });

    vehicleCanFrameHandler.registerHandler(can::SetMotorControlParams::id(), [] (const uint8_t * const data) {
        reinterpret_cast<const can::SetMotorControlParams*>(data)->acquire(speedControllerParams.P, speedControllerParams.I);
        speedController.tune(speedControllerParams);
        vehicleCanManager.send<can::MotorControlParams>(vehicleCanSubscriberId, speedControllerParams.P, speedControllerParams.I);
    });

    const CanFrameIds rxFilter = vehicleCanFrameHandler.identifiers();
    const CanFrameIds txFilter = {
        can::MotorControlParams::id(),
        can::LateralState::id(),
        can::LongitudinalState::id()
    };
    vehicleCanSubscriberId = vehicleCanManager.registerSubscriber(rxFilter, txFilter);
}

} // namespace

extern "C" void runControlTask(void) {

    SystemManager::instance().registerTask();

    encoder.initialize();

    initializeVehicleCan();

    while (true) {
        while (const auto frame = vehicleCanManager.read(vehicleCanSubscriberId)) {
            vehicleCanFrameHandler.handleFrame(*frame);
        }

        RemoteControllerData remoteControlData;
        if (remoteControllerQueue.receive(remoteControlData, millisecond_t(0))) {
            remoteControl = remoteControlData;
        }

        const ControlData validControl = getControl(swControl, remoteControl);

        speedController.target = speedRamp.update(car.speed, validControl.lon.value().speed, validControl.lon.value().rampTime).get();
        frontSteeringServo.write(validControl.lat.value().frontWheelAngle);
        rearSteeringServo.write(validControl.lat.value().rearWheelAngle);

        car.frontWheelAngle = frontSteeringServo.angle();
        car.rearWheelAngle  = rearSteeringServo.angle();

        vehicleCanManager.periodicSend<can::LongitudinalState>(vehicleCanSubscriberId, car.speed, isRemoteControlled, car.distance);
        vehicleCanManager.periodicSend<can::LateralState>(vehicleCanSubscriberId, car.frontWheelAngle, car.rearWheelAngle, radian_t(0));

        SystemManager::instance().notify(
            !vehicleCanManager.hasTimedOut(vehicleCanSubscriberId) &&
            !hasControlTimedOut(swControl.lat)                     &&
            !hasControlTimedOut(swControl.lon)                     &&
            !hasControlTimedOut(remoteControl));

        os_sleep(millisecond_t(1));
    }
}

void tim_ControlLoop_PeriodElapsedCallback() {

    static millisecond_t lastUpdateTime = getExactTime();

    const millisecond_t now = getExactTime();
    encoder.update();

    car.speed    = encoder.lastDiff() * cfg::ENCODER_INCR_DISTANCE / (now - lastUpdateTime);
    car.distance = encoder.numIncr() * cfg::ENCODER_INCR_DISTANCE;

    speedController.update(car.speed.get());
    dcMotor.write(speedController.output());

    lastUpdateTime = now;
}

void micro_Vehicle_Can_RxFifoMsgPendingCallback() {
    vehicleCanManager.onFrameReceived();
}
