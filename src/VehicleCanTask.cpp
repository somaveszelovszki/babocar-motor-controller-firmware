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
QueueHandle_t lateralControlQueue;
static uint8_t lateralControlQueueStorageBuffer[LATERAL_CONTROL_QUEUE_LENGTH * sizeof(LateralControl)];
static StaticQueue_t lateralControlQueueBuffer;

#define LONGITUDINAL_CONTROL_QUEUE_LENGTH 1
QueueHandle_t longitudinalControlQueue;
static uint8_t longitudinalControlQueueStorageBuffer[LONGITUDINAL_CONTROL_QUEUE_LENGTH * sizeof(LateralControl)];
static StaticQueue_t longitudinalControlQueueBuffer;

namespace {

} // namespace

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

    canManager.registerHandler(can::SetServoOffsets::id(), [&canManager] (const uint8_t * const data) {
        reinterpret_cast<const can::SetServoOffsets*>(data)->acquire(globals::frontSteeringServoOffset, globals::rearSteeringServoOffset, globals::extraServoOffset);
        canManager.send(can::ServoOffsets(globals::frontSteeringServoOffset, globals::rearSteeringServoOffset, globals::extraServoOffset));
    });

    while (true) {
        globals::isVehicleCanTaskOk = !canManager.hasRxTimedOut();

        canManager.handleIncomingFrames();

        vTaskDelay(1);
    }

    vTaskDelete(nullptr);
}
