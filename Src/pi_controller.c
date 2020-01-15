#include "pi_controller.h"

#include <micro/math/numeric.h>

void pi_controller_update(pi_controller_t *pi, float measured) {
	if (0.0f == pi->desired && ABS(measured) < pi->deadband) {
        pi->integral = 0.0f;
        pi->output = 0.0f;
	} else {
        const float ek = pi->desired - measured;
        pi->integral = CLAMP(pi->integral + ek, -pi->integral_max, pi->integral_max);
        pi->output = CLAMP(ek * pi->P + pi->integral * pi->I, pi->out_min, pi->out_max);
	}
}
