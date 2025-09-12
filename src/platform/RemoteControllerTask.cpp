#include <RemoteControllerData.hpp>
#include <cfg_board.hpp>
#include <globals.hpp>

#include <micro/debug/DebugLed.hpp>
#include <micro/debug/TaskMonitor.hpp>
#include <micro/math/numeric.hpp>
#include <micro/port/queue.hpp>
#include <micro/port/task.hpp>
#include <micro/sensor/Filter.hpp>

using namespace micro;

namespace {

constexpr uint32_t INPUT_CHANNEL_ACCEL_OFFSET = 80;
constexpr uint32_t INPUT_CHANNEL_STEER_OFFSET = 35;
constexpr uint32_t INPUT_CHANNEL_MODE_OFFSET  = 0;

constexpr float INPUT_ZERO_DEADBAND = 0.1f;

struct RemoteInput {
    typedef LowPassFilter<float, 3> filter_t;

    filter_t accelerationFilter = filter_t(0.0f);
    filter_t steeringFilter     = filter_t(0.0f);
    filter_t modeSelectFilter   = filter_t(0.0f);
};

RemoteInput remoteInput;

void onRcCtrlInputCapture(const uint32_t chnl, uint32_t& prevCntr, const uint32_t offset,
                          RemoteInput::filter_t& inputFilter) {
    uint32_t cntr = 0;
    timer_getCaptured(tim_RcCtrl, chnl, cntr);

    uint32_t duty =
        (cntr >= prevCntr ? cntr - prevCntr : tim_RcCtrl.handle->Instance->ARR - prevCntr + cntr) -
        offset;
    if (duty > 850 && duty < 2150) {
        float input = micro::lerp<uint32_t, float>(duty, 1000, 2000, -1.0f, 1.0f);
        if (abs(input) < INPUT_ZERO_DEADBAND) {
            input = 0.0f;
        }

        inputFilter.update(input);
    }
    prevCntr = cntr;
}

RemoteControllerData::channel_t getActiveChannel() {
    return getTime() - remoteInput.modeSelectFilter.lastUpdateTimestamp() < millisecond_t(50)
               ? remoteInput.modeSelectFilter.value() < 0.0f
                     ? RemoteControllerData::channel_t::DirectControl
                     : RemoteControllerData::channel_t::SafetyEnable
               : RemoteControllerData::channel_t::INVALID;
}

void fillRemoteControllerData(RemoteControllerData& remoteControllerData,
                              const RemoteControllerData::channel_t activeChannel) {
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
    taskMonitor.registerInitializedTask();

    RemoteControllerData remoteControllerData;

    while (true) {
        const RemoteControllerData::channel_t activeChannel = getActiveChannel();
        fillRemoteControllerData(remoteControllerData, activeChannel);
        remoteControllerQueue.overwrite(remoteControllerData);

        taskMonitor.notify(!useSafetyEnableSignal ||
                           activeChannel != RemoteControllerData::channel_t::INVALID);
        os_sleep(millisecond_t(10));
    }
}

void tim_RcCtrlAccel_IC_CaptureCallback() {
    static uint32_t cntr = 0;
    onRcCtrlInputCapture(timChnl_RcCtrlAccel, cntr, INPUT_CHANNEL_ACCEL_OFFSET,
                         remoteInput.accelerationFilter);
}

void tim_RcCtrlSteer_IC_CaptureCallback() {
    static uint32_t cntr = 0;
    onRcCtrlInputCapture(timChnl_RcCtrlSteer, cntr, INPUT_CHANNEL_STEER_OFFSET,
                         remoteInput.steeringFilter);
}

void tim_RcCtrlModeSelect_IC_CaptureCallback() {
    static uint32_t cntr = 0;
    onRcCtrlInputCapture(timChnl_RcCtrlModeSelect, cntr, INPUT_CHANNEL_MODE_OFFSET,
                         remoteInput.modeSelectFilter);
}
