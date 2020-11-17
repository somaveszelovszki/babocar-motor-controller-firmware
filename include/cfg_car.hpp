#pragma once

#include <micro/utils/units.hpp>

namespace cfg {

constexpr micro::micrometer_t ENCODER_INCR_DISTANCE            = micro::micrometer_t(8.16573169f); // 122463 ticks per 1 meter
constexpr float MOTOR_MAX_DUTY                                 = 0.92f;
constexpr micro::m_per_sec_t DIRECT_CONTROL_MAX_SPEED          = micro::m_per_sec_t(3);

constexpr uint32_t FRONT_STEERING_SERVO_PWM_CENTER             = 14300;
constexpr micro::radian_t FRONT_STEERING_SERVO_TRANSFER_RATE   = micro::milliradian_t(0.12f);
constexpr micro::radian_t FRONT_WHEEL_MAX_DELTA_ANGLE          = micro::degree_t(26);
constexpr micro::rad_per_sec_t FRONT_SERVO_MAX_ANGULAR_VELO    = micro::deg_per_sec_t(500);

constexpr uint32_t REAR_STEERING_SERVO_PWM_CENTER              = 14700;
constexpr micro::radian_t REAR_STEERING_SERVO_TRANSFER_RATE    = micro::milliradian_t(0.13f);
constexpr micro::radian_t REAR_WHEEL_MAX_DELTA_ANGLE           = micro::degree_t(26);
constexpr micro::rad_per_sec_t REAR_SERVO_MAX_ANGULAR_VELO     = micro::deg_per_sec_t(500);

constexpr micro::millisecond_t EMERGENCY_BRAKE_DURATION_HARD   = micro::millisecond_t(200);
constexpr micro::millisecond_t EMERGENCY_BRAKE_DURATION_SMOOTH = micro::millisecond_t(800);

} // namespace cfg
