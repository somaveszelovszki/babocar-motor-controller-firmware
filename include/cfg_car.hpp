#pragma once

#include <micro/utils/units.hpp>

namespace cfg {

constexpr micro::micrometer_t ENCODER_INCR_DISTANCE                    = micro::micrometer_t(8.061602656f);
constexpr float MOTOR_MAX_DUTY                                         = 0.96f;
constexpr micro::m_per_sec_t DIRECT_CONTROL_MAX_SPEED                  = micro::m_per_sec_t(1.7f);

constexpr uint32_t FRONT_STEERING_SERVO_PWM_CENTER                    = 14300;
constexpr micro::radian_t FRONT_STEERING_SERVO_POSITIVE_TRANSFER_RATE = micro::milliradian_t(0.12965f);
constexpr micro::radian_t FRONT_STEERING_SERVO_NEGATIVE_TRANSFER_RATE = micro::milliradian_t(0.12979f);
constexpr micro::radian_t FRONT_WHEEL_MAX_DELTA_ANGLE                 = micro::degree_t(26);
constexpr micro::rad_per_sec_t FRONT_SERVO_MAX_ANGULAR_VELOCITY       = micro::deg_per_sec_t(500);

constexpr uint32_t REAR_STEERING_SERVO_PWM_CENTER                     = 14560;
constexpr micro::radian_t REAR_STEERING_SERVO_POSITIVE_TRANSFER_RATE  = micro::milliradian_t(0.12901f);
constexpr micro::radian_t REAR_STEERING_SERVO_NEGATIVE_TRANSFER_RATE  = micro::milliradian_t(0.12779f);
constexpr micro::radian_t REAR_WHEEL_MAX_DELTA_ANGLE                  = micro::degree_t(26);
constexpr micro::rad_per_sec_t REAR_SERVO_MAX_ANGULAR_VELOCITY        = micro::deg_per_sec_t(500);

constexpr micro::millisecond_t SMOOTH_BRAKE_DURATION                  = micro::millisecond_t(1000);
constexpr micro::millisecond_t EMERGENCY_BRAKE_DURATION               = micro::millisecond_t(300);

} // namespace cfg
