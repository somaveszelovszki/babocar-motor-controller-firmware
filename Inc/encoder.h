#ifndef ENCODER_H_
#define ENCODER_H_

#include <micro/utils/types.h>

typedef struct {
    int64_t abs_pos;
    int64_t num_incr;
	int32_t prev_pos;
	int32_t last_diff;
	int32_t max_value;
} encoder_t;

void encoder_initialize(encoder_t *enc, int32_t max_value);

void encoder_update(encoder_t *enc);

#endif /* ENCODER_H_ */
