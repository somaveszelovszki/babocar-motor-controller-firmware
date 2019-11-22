#include "encoder.h"
#include "config.h"
#include "common.h"

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_tim.h"
#include "tim.h"

#include <stdlib.h>

void encoder_initialize(encoder_t *enc, int32_t max_value) {
    enc->abs_pos = 0LL;
	enc->prev_pos = 0;
	enc->last_diff = 0;
	enc->max_value = max_value;
	__HAL_TIM_SET_COUNTER(tim_encoder, 0);
}

void encoder_update(encoder_t *enc) {
	const int32_t pos = __HAL_TIM_GET_COUNTER(tim_encoder);
	int32_t diff = pos - enc->prev_pos;

	if (ABS(diff) > enc->max_value / 2) {
		const int32_t dp = diff + enc->max_value, dm = diff - enc->max_value;
		diff = ABS(dp) <= ABS(dm) ? dp : dm;
	}

	enc->abs_pos += (int64_t)diff;
	enc->prev_pos = pos;
	enc->last_diff = diff;
}
