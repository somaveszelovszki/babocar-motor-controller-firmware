#pragma once

#include <micro/utils/units.hpp>

namespace cfg {

constexpr micro::micrometer_t ENCODER_INCR_DISTANCE   = micro::micrometer_t(10);
constexpr float MOTOR_MAX_DUTY                        = 0.92f;
constexpr micro::m_per_sec_t DIRECT_CONTROL_MAX_SPEED = micro::m_per_sec_t(3);

constexpr float SERVO_WHEEL_TRANSFER_RATE             = 1.0f;
constexpr uint32_t FRONT_STEERING_SERVO_PWM0          = 500;
constexpr uint32_t FRONT_STEERING_SERVO_PWM180        = 2500;
constexpr uint32_t REAR_STEERING_SERVO_PWM0           = 500;
constexpr uint32_t REAR_STEERING_SERVO_PWM180         = 2500;

} // namespace cfg
