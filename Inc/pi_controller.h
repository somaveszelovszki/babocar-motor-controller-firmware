#ifndef PI_CONTROLLER_H_
#define PI_CONTROLLER_H_

#include <micro/utils/types.h>

typedef struct {
    float P;
    float I;
    float integral_max;
    float out_min;
    float out_max;
    float deadband;

    float integral;
    float desired;
    float output;

} pi_controller_t;

#define PI_CONTROLLER_INIT(_P_, _I_, _integral_max_, _out_min_, _out_max_, _deadband_) \
{                                                                                      \
    .P = _P_,                                                                          \
    .I = _I_,                                                                          \
    .integral_max = _integral_max_,                                                    \
    .out_min = _out_min_,                                                              \
    .out_max = _out_max_,                                                              \
    .deadband = _deadband_,                                                            \
    .integral = 0.0f,                                                                  \
    .desired = 0.0f,                                                                   \
    .output = 0.0f                                                                     \
}

void pi_controller_set_params(pi_controller_t *pi, uint32_t Ti_us, float Kc);

void pi_controller_update(pi_controller_t *pi, float measured);

#endif /* PI_CONTROLLER_H_ */
