#include <micro/utils/state.hpp>
#include <micro/math/numeric.hpp>

#include <cfg_board.h>
#include <globals.hpp>
#include <RemoteControllerData.hpp>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

using namespace micro;

#define REMOTE_CONTROLLER_QUEUE_LENGTH 1
QueueHandle_t remoteControllerQueue;
static uint8_t remoteControllerQueueStorageBuffer[REMOTE_CONTROLLER_QUEUE_LENGTH * sizeof(RemoteControllerData)];
static StaticQueue_t remoteControllerQueueBuffer;

namespace {

struct {
    state_t<float> acceleration = state_t<float>(0.0f);
    state_t<float> steering     = state_t<float>(0.0f);
} directControl, safetyEnable;

void onRcCtrlInputCapture(const uint32_t chnl, uint32_t& prevCntr, state_t<float>& state) {
    const uint32_t cntr = __HAL_TIM_GET_COMPARE(tim_RcCtrl, chnl);
    const uint32_t duty = cntr >= prevCntr ? cntr - prevCntr : tim_RcCtrl->Instance->ARR - prevCntr + cntr;
    if (duty > 850 && duty < 2150) {
        state = map<uint32_t, float>(duty, 1000, 2000, -1.0f, 1.0f);
    }
    prevCntr = cntr;
}

} // namespace

extern "C" void runRemoteControllerTask(void) {

    remoteControllerQueue = xQueueCreateStatic(REMOTE_CONTROLLER_QUEUE_LENGTH, sizeof(RemoteControllerData), remoteControllerQueueStorageBuffer, &remoteControllerQueueBuffer);

    while (true) {

    }

    vTaskDelete(nullptr);
}

void tim_RcCtrlDirectAccel_IC_CaptureCallback() {
    static uint32_t cntr = 0;
    onRcCtrlInputCapture(timChnl_RcCtrlDirectAccel, cntr, directControl.acceleration);
}

void tim_RcCtrlDirectSteer_IC_CaptureCallback() {
    static uint32_t cntr = 0;
    onRcCtrlInputCapture(timChnl_RcCtrlDirectSteer, cntr, directControl.steering);
}

void tim_RcCtrlSafetyAccel_IC_CaptureCallback() {
    static uint32_t cntr = 0;
    onRcCtrlInputCapture(timChnl_RcCtrlSafetyAccel, cntr, safetyEnable.acceleration);
}

void tim_RcCtrlSafetySteer_IC_CaptureCallback() {
    static uint32_t cntr = 0;
    onRcCtrlInputCapture(timChnl_RcCtrlSafetySteer, cntr, safetyEnable.steering);
}

