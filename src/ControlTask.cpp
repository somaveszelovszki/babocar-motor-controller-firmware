#include <cfg_board.h>
#include <globals.hpp>
#include <RemoteControllerData.hpp>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

using namespace micro;

extern QueueHandle_t remoteControllerQueue;

namespace {

} // namespace

extern "C" void runControlTask(void) {

    while (true) {

    }

    vTaskDelete(nullptr);
}

void tim_ControlLoop_PeriodElapsedCallback() {
//    encoder_update((encoder_t*)&encoder);
//    speed_measured_mps = encoder.last_diff / ENCODER_INCR_PER_MM / (ENCODER_PERIOD_US / 1000.0f);
//    distance_mm = encoder.num_incr / ENCODER_INCR_PER_MM;
//
//    pi_controller_update((pi_controller_t*)&speedCtrl, speed_measured_mps);
//    //dc_motor_write(speedCtrl.output);
//    dc_motor_write(0.0f);
}
