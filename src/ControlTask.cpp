#include <micro/hw/DC_Motor.hpp>
#include <micro/hw/Encoder.hpp>
#include <micro/control/PID_Controller.hpp>

#include <cfg_board.h>
#include <cfg_car.hpp>
#include <globals.hpp>
#include <RemoteControllerData.hpp>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

using namespace micro;

extern QueueHandle_t remoteControllerQueue;

namespace {

hw::DC_Motor dcMotor(tim_DC_Motor, timChnl_DC_Motor_Bridge1, timChnl_DC_Motor_Bridge2, cfg::DC_MOTOR_MAX_DUTY);
hw::Encoder encoder(tim_Encoder);
PID_Controller speedCtrl(millisecond_t Ts, millisecond_t Ti, millisecond_t Td, float Kc, -cfg::DC_MOTOR_MAX_DUTY, cfg::DC_MOTOR_MAX_DUTY)


} // namespace

extern "C" void runControlTask(void) {

    while (true) {

    }

    vTaskDelete(nullptr);
}

void tim_ControlLoop_PeriodElapsedCallback() {

    encoder.update();

    speed_measured_mps = encoder.last_diff / ENCODER_INCR_PER_MM / (ENCODER_PERIOD_US / 1000.0f);
    distance_mm = encoder.num_incr / ENCODER_INCR_PER_MM;

    pi_controller_update((pi_controller_t*)&speedCtrl, speed_measured_mps);
    //dc_motor_write(speedCtrl.output);
    dc_motor_write(0.0f);
}
