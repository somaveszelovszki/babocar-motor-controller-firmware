#include <micro/debug/DebugLed.hpp>
#include <micro/debug/SystemManager.hpp>
#include <micro/math/numeric.hpp>
#include <micro/port/task.hpp>
#include <micro/sensor/Filter.hpp>

#include <cfg_board.h>
#include <RemoteControllerData.hpp>

using namespace micro;

queue_t<RemoteControllerData, 1> remoteControllerQueue;

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
    return getTime() - max(directControl.timestamp(), safetyEnable.timestamp()) < millisecond_t(50) ?
        directControl.timestamp() > safetyEnable.timestamp() ?
            RemoteControllerData::channel_t::DirectControl :
            RemoteControllerData::channel_t::SafetyEnable :
        RemoteControllerData::channel_t::INVALID;
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
        remoteControllerData.acceleration = 0.0f;
        remoteControllerData.steering     = 0.0f;
        break;
    }
}

} // namespace

extern "C" void runRemoteControllerTask(void) {

    SystemManager::instance().registerTask();

    RemoteControllerData remoteControllerData;

    while (true) {
        const RemoteControllerData::channel_t activeChannel = getActiveChannel();
        fillRemoteControllerData(remoteControllerData, activeChannel);
        remoteControllerQueue.overwrite(remoteControllerData);

        SystemManager::instance().notify(activeChannel != RemoteControllerData::channel_t::INVALID);
        os_delay(20);
    }
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

