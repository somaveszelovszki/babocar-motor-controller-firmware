#include <cfg_board.h>

// INTERRUPT CALLBACKS - Must be defined in a task's source file!

extern void tim_RcCtrlDirectAccel_IC_CaptureCallback();
extern void tim_RcCtrlDirectSteer_IC_CaptureCallback();
extern void tim_RcCtrlSafetyAccel_IC_CaptureCallback();
extern void tim_RcCtrlSafetySteer_IC_CaptureCallback();
extern void tim_ControlLoop_PeriodElapsedCallback();

namespace {

uint32_t getChannel(const HAL_TIM_ActiveChannel chnl) {
    uint32_t result = 0xFFFFFFFFU;
    switch (chnl) {
    case HAL_TIM_ACTIVE_CHANNEL_1: result = TIM_CHANNEL_1; break;
    case HAL_TIM_ACTIVE_CHANNEL_2: result = TIM_CHANNEL_2; break;
    case HAL_TIM_ACTIVE_CHANNEL_3: result = TIM_CHANNEL_3; break;
    case HAL_TIM_ACTIVE_CHANNEL_4: result = TIM_CHANNEL_4; break;
    default: /* should not get here */                     break;
    }
    return result;
}

} // namespace

extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim == tim_RcCtrl) {
        switch (getChannel(htim->Channel)) {
        case timChnl_RcCtrlDirectAccel: tim_RcCtrlDirectAccel_IC_CaptureCallback(); break;
        case timChnl_RcCtrlDirectSteer: tim_RcCtrlDirectSteer_IC_CaptureCallback(); break;
        case timChnl_RcCtrlSafetyAccel: tim_RcCtrlSafetyAccel_IC_CaptureCallback(); break;
        case timChnl_RcCtrlSafetySteer: tim_RcCtrlSafetySteer_IC_CaptureCallback(); break;
        default: /* should not get here */                                          break;
        }
    }
}

extern "C" void micro_tim_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == tim_ControlLoop) {
        tim_ControlLoop_PeriodElapsedCallback();
    }
}
