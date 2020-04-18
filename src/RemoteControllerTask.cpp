#include <micro/utils/state.hpp>
#include <micro/math/numeric.hpp>
#include <micro/sensor/Filter.hpp>

#include <cfg_board.h>
#include <globals.hpp>
#include <RemoteControllerData.hpp>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

using namespace micro;

#define REMOTE_CONTROLLER_QUEUE_LENGTH 1
QueueHandle_t remoteControllerQueue = nullptr;
static uint8_t remoteControllerQueueStorageBuffer[REMOTE_CONTROLLER_QUEUE_LENGTH * sizeof(RemoteControllerData)];
static StaticQueue_t remoteControllerQueueBuffer;

namespace {

constexpr float INPUT_FILTER_DEADBAND = 0.2f;

struct RemoteInput {
    typedef BounceFilter<float, 3> filter_t;

    filter_t accelerationFilter = filter_t(0.0f, 0.0f, INPUT_FILTER_DEADBAND);
    filter_t steeringFilter     = filter_t(0.0f, 0.0f, INPUT_FILTER_DEADBAND);

    millisecond_t timestamp() const {
        return max(this->accelerationFilter.lastUpdateTimestamp(), this->steeringFilter.lastUpdateTimestamp());
    }
};

RemoteInput directControl, safetyEnable;

void onRcCtrlInputCapture(const uint32_t chnl, uint32_t& prevCntr, RemoteInput::filter_t& inputFilter) {
    const uint32_t cntr = __HAL_TIM_GET_COMPARE(tim_RcCtrl, chnl);
    const uint32_t duty = cntr >= prevCntr ? cntr - prevCntr : tim_RcCtrl->Instance->ARR - prevCntr + cntr;
    if (duty > 850 && duty < 2150) {
        inputFilter.update(map<uint32_t, float>(duty, 1000, 2000, -1.0f, 1.0f));
    }
    prevCntr = cntr;
}

RemoteControllerData::channel_t getActiveChannel() {
    return directControl.timestamp() > safetyEnable.timestamp() ? RemoteControllerData::channel_t::DirectControl : RemoteControllerData::channel_t::SafetyEnable;
}

void fillRemoteControllerData(RemoteControllerData& remoteControllerData, const RemoteControllerData::channel_t activeChannel) {
    remoteControllerData.activeChannel = activeChannel;
    switch (activeChannel) {
    case RemoteControllerData::channel_t::DirectControl:
        remoteControllerData.acceleration = directControl.accelerationFilter.value();
        remoteControllerData.steering     = directControl.steeringFilter.value();
        break;
    case RemoteControllerData::channel_t::SafetyEnable:
        remoteControllerData.acceleration = safetyEnable.accelerationFilter.value();
        remoteControllerData.steering     = safetyEnable.steeringFilter.value();
        break;
    default:
        break;
    }
}

} // namespace

extern "C" void runRemoteControllerTask(void) {

    remoteControllerQueue = xQueueCreateStatic(REMOTE_CONTROLLER_QUEUE_LENGTH, sizeof(RemoteControllerData), remoteControllerQueueStorageBuffer, &remoteControllerQueueBuffer);

    while (true) {
        RemoteControllerData remoteControllerData;
        fillRemoteControllerData(remoteControllerData, getActiveChannel());
        xQueueOverwrite(remoteControllerQueue, &remoteControllerData);
        vTaskDelay(20);
    }

    vTaskDelete(nullptr);
}

void tim_RcCtrlDirectAccel_IC_CaptureCallback() {
    static uint32_t cntr = 0;
    onRcCtrlInputCapture(timChnl_RcCtrlDirectAccel, cntr, directControl.accelerationFilter);
}

void tim_RcCtrlDirectSteer_IC_CaptureCallback() {
    static uint32_t cntr = 0;
    onRcCtrlInputCapture(timChnl_RcCtrlDirectSteer, cntr, directControl.steeringFilter);
}

void tim_RcCtrlSafetyAccel_IC_CaptureCallback() {
    static uint32_t cntr = 0;
    onRcCtrlInputCapture(timChnl_RcCtrlSafetyAccel, cntr, safetyEnable.accelerationFilter);
}

void tim_RcCtrlSafetySteer_IC_CaptureCallback() {
    static uint32_t cntr = 0;
    onRcCtrlInputCapture(timChnl_RcCtrlSafetySteer, cntr, safetyEnable.steeringFilter);
}

