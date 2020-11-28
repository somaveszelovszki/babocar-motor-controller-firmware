#include <micro/debug/DebugLed.hpp>
#include <micro/debug/SystemManager.hpp>
#include <micro/math/numeric.hpp>
#include <micro/port/queue.hpp>
#include <micro/port/task.hpp>
#include <micro/sensor/Filter.hpp>

#include <cfg_board.hpp>
#include <RemoteControllerData.hpp>

using namespace micro;

queue_t<RemoteControllerData, 1> remoteControllerQueue;

namespace {

constexpr uint32_t INPUT_CHANNEL_OFFSET      = 50;
constexpr float INPUT_ZERO_DEADBAND          = 0.15f;
constexpr float INPUT_FILTER_COMPLIANCE_RATE = 0.2f;
constexpr float INPUT_FILTER_DEADBAND        = 0.5f;

struct RemoteInput {
    typedef BounceFilter<float, 3> filter_t;

    filter_t accelerationFilter = filter_t(0.0f, INPUT_FILTER_COMPLIANCE_RATE, INPUT_FILTER_DEADBAND);
    filter_t steeringFilter     = filter_t(0.0f, INPUT_FILTER_COMPLIANCE_RATE, INPUT_FILTER_DEADBAND);
    filter_t modeSelectFilter   = filter_t(0.0f, INPUT_FILTER_COMPLIANCE_RATE, INPUT_FILTER_DEADBAND);
};

RemoteInput remoteInput;

void onRcCtrlInputCapture(const uint32_t chnl, uint32_t& prevCntr, RemoteInput::filter_t& inputFilter) {
    uint32_t cntr = 0;
    timer_getCaptured(tim_RcCtrl, chnl, cntr);

    uint32_t duty = (cntr >= prevCntr ? cntr - prevCntr : tim_RcCtrl.handle->Instance->ARR - prevCntr + cntr) - INPUT_CHANNEL_OFFSET;
    if (duty > 850 && duty < 2150) {
        float input = map<uint32_t, float>(duty, 1000, 2000, -1.0f, 1.0f);
        if (abs(input) < INPUT_ZERO_DEADBAND) {
            input = 0.0f;
        }

        inputFilter.update(input);

    }
    prevCntr = cntr;
}

RemoteControllerData::channel_t getActiveChannel() {
    return getTime() - remoteInput.modeSelectFilter.lastUpdateTimestamp() < millisecond_t(50) ?
        remoteInput.modeSelectFilter.value() < 0.0f ?
            RemoteControllerData::channel_t::DirectControl :
            RemoteControllerData::channel_t::SafetyEnable  :
        RemoteControllerData::channel_t::INVALID;
}

void fillRemoteControllerData(RemoteControllerData& remoteControllerData, const RemoteControllerData::channel_t activeChannel) {
    remoteControllerData.activeChannel = activeChannel;
    if (activeChannel == RemoteControllerData::channel_t::INVALID) {
        remoteControllerData.acceleration = 0.0f;
        remoteControllerData.steering     = 0.0f;
    } else {
        remoteControllerData.acceleration = remoteInput.accelerationFilter.value();
        remoteControllerData.steering     = remoteInput.steeringFilter.value();
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
        os_sleep(millisecond_t(10));
    }
}

void tim_RcCtrlAccel_IC_CaptureCallback() {
    static uint32_t cntr = 0;
    onRcCtrlInputCapture(timChnl_RcCtrlAccel, cntr, remoteInput.accelerationFilter);
}

void tim_RcCtrlSteer_IC_CaptureCallback() {
    static uint32_t cntr = 0;
    onRcCtrlInputCapture(timChnl_RcCtrlSteer, cntr, remoteInput.steeringFilter);
}

void tim_RcCtrlModeSelect_IC_CaptureCallback() {
    static uint32_t cntr = 0;
    onRcCtrlInputCapture(timChnl_RcCtrlModeSelect, cntr, remoteInput.modeSelectFilter);
}
