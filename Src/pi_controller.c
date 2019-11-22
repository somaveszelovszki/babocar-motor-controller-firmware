#include "pi_controller.h"
#include "common.h"

#include <stdlib.h>

static const float P = 0.8f;
static const float I = 0.04f;
static float integral = 0.0f;
static const float INTEGRAL_MAX = 2.0f;

static void update_coeffs(pi_controller_t *pi) {
    pi->b0 = pi->Kc * (1 + (float)pi->period_us / pi->Ti_us);
    pi->b1 = -pi->Kc;
}

void pi_controller_initialize(pi_controller_t *pi, uint32_t period_us, uint32_t Ti_us, float Kc, float deadband, float out_min, float out_max, float max_delta) {
    pi->period_us = period_us;
    pi->Ti_us     = Ti_us;
    pi->Kc        = Kc;
    pi->deadband  = deadband;
    pi->out_min   = out_min;
    pi->out_max   = out_max;
    pi->max_delta = max_delta;
    pi->desired   = pi->output = pi->ek1 = 0.0f;

    update_coeffs(pi);
}

void pi_controller_set_params(pi_controller_t *pi, uint32_t Ti_us, float Kc) {
    pi->Ti_us = Ti_us;
    pi->Kc = Kc;
    update_coeffs(pi);
}

void pi_controller_update(pi_controller_t *pi, float measured) {
	if (0.0f == pi->desired && ABS(measured) < pi->deadband) {
	    pi->output = 0.0f;
	    pi->ek1 = 0.0f;
	} else {
        const float ek = pi->desired - measured;
//        const float prev_out = pi->output;
//        pi->output += pi->b0 * ek + pi->b1 * pi->ek1;
//        pi->output = CLAMP(pi->output, prev_out - pi->max_delta, prev_out + pi->max_delta);
//        pi->output = CLAMP(pi->output, pi->out_min, pi->out_max);
        integral = CLAMP(integral + ek, -INTEGRAL_MAX, INTEGRAL_MAX);
        pi->output = CLAMP(ek * P + integral * I, pi->out_min, pi->out_max);
	}
}
