#include <micro/panel/CanManager.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/timer.hpp>

#include <cfg_board.h>
#include <globals.hpp>
#include <LateralControl.hpp>
#include <LongitudinalControl.hpp>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

using namespace micro;

#include <micro/container/map.hpp>

#include <functional>

#define LATERAL_CONTROL_QUEUE_LENGTH 1
QueueHandle_t lateralControlQueue = nullptr;
static uint8_t lateralControlQueueStorageBuffer[LATERAL_CONTROL_QUEUE_LENGTH * sizeof(LateralControl)];
static StaticQueue_t lateralControlQueueBuffer;

#define LONGITUDINAL_CONTROL_QUEUE_LENGTH 1
QueueHandle_t longitudinalControlQueue = nullptr;
static uint8_t longitudinalControlQueueStorageBuffer[LONGITUDINAL_CONTROL_QUEUE_LENGTH * sizeof(LateralControl)];
static StaticQueue_t longitudinalControlQueueBuffer;

extern "C" void runVehicleCanTask(void) {

    lateralControlQueue = xQueueCreateStatic(LATERAL_CONTROL_QUEUE_LENGTH, sizeof(LateralControl), lateralControlQueueStorageBuffer, &lateralControlQueueBuffer);
    longitudinalControlQueue = xQueueCreateStatic(LONGITUDINAL_CONTROL_QUEUE_LENGTH, sizeof(LongitudinalControl), longitudinalControlQueueStorageBuffer, &longitudinalControlQueueBuffer);

    CanManager canManager(can_Vehicle, canRxFifo_Vehicle, millisecond_t(50));

    canManager.registerHandler(can::LateralControl::id(), [] (const uint8_t * const data) {
        LateralControl lateral;
        reinterpret_cast<const can::LateralControl*>(data)->acquire(lateral.frontWheelAngle, lateral.rearWheelAngle, lateral.extraServoAngle);
        xQueueOverwrite(lateralControlQueue, &lateral);
    });

    canManager.registerHandler(can::LongitudinalControl::id(), [] (const uint8_t * const data) {
        LongitudinalControl longitudinal;
        reinterpret_cast<const can::LongitudinalControl*>(data)->acquire(longitudinal.speed, globals::useSafetyEnableSignal, longitudinal.rampTime);
        xQueueOverwrite(lateralControlQueue, &longitudinal);
    });

    canManager.registerHandler(can::SetFrontWheelParams::id(), [&canManager] (const uint8_t * const data) {
        reinterpret_cast<const can::SetFrontWheelParams*>(data)->acquire(globals::frontWheelOffset, globals::frontWheelMaxDelta);
        canManager.send(can::FrontWheelParams(globals::frontWheelOffset, globals::frontWheelMaxDelta));
    });

    canManager.registerHandler(can::SetRearWheelParams::id(), [&canManager] (const uint8_t * const data) {
        reinterpret_cast<const can::SetRearWheelParams*>(data)->acquire(globals::rearWheelOffset, globals::rearWheelMaxDelta);
        canManager.send(can::RearWheelParams(globals::rearWheelOffset, globals::rearWheelMaxDelta));
    });

    Timer lateralStateTimer(can::LateralState::period());
    Timer longitudinalStateTimer(can::LongitudinalControl::period());

    while (true) {
        globals::isVehicleCanTaskOk = !canManager.hasRxTimedOut();

        canManager.handleIncomingFrames();

        if (lateralStateTimer.checkTimeout()) {
            canManager.send(can::LateralState(globals::car.frontWheelAngle, globals::car.rearWheelAngle, radian_t(0)));
        }

        if (longitudinalStateTimer.checkTimeout()) {
            canManager.send(can::LongitudinalState(globals::car.speed, globals::car.distance));
        }

        vTaskDelay(1);
    }

    vTaskDelete(nullptr);
}
