#ifndef COMMON_H_
#define COMMON_H_

#include <micro/utils/types.h>

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define ABS(a) ((a) >= 0 ? (a) : (-a))

int32_t round_to_int(float value);

#define CLAMP(value, bound1, bound2)    \
((bound1) <= (bound2) ?                 \
MAX(MIN((value), (bound2)), (bound1)) : \
MIN(MAX((value), (bound2)), (bound1)))

#define MAP(value, from_low, from_high, to_low, to_high) \
((to_low) + (CLAMP((value), (from_low), (from_high)) - (from_low)) * ((to_high) - (to_low)) / ((from_high) - (from_low)))

#endif /* COMMON_H_ */
