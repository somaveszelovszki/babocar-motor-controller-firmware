#ifndef CONFIG_H_
#define CONFIG_H_

#define SPEED_CTRL_PERIOD_US          500u    // TODO [us]
#define SPEED_CTRL_Ti_US              484u    // in [us]
#define SPEED_CTRL_Kc                 0.01f    // TODO
#define SPEED_CTRL_MAX_DELTA          0.01f
#define SPEED_CTRL_DEADBAND_MPS       0.01f  // If measured speed is less than this value [m/s], speed controller output will be zero

#define ENCODER_PERIOD_US             500u    // Period time of encoder evaluation - in [us]
#define ENCODER_MAX_VALUE             65536
#define ENCODER_INCR_PER_MM           139.0627f

#define SAFETY_SIGNAL_CHECK_PERIOD_MS 20u   // Period of safety signal checking - in [ms]

#define uart_cmd                  (&huart1)

#define gpio_user_led             GPIOB
#define gpio_pin_user_led         GPIO_PIN_5

#define tim_motor                 (&htim1)
#define tim_encoder               (&htim3)
#define tim_speedControllerPeriod (&htim17)
#define tim_rc_recv               (&htim14)
#define chnl_rc_recv              TIM_CHANNEL_1

#define FACTORY_MOTOR_CONTROLLER  0
#if FACTORY_MOTOR_CONTROLLER
#define chnl_motor           TIM_CHANNEL_2
#define motor_PWM_PERIOD     20000
#define motor_PWM_STOP       1500
#define motor_HARD_MAX       1750
#define motor_HARD_MIN       1000
#else
#define chnl_bridge_1_high   TIM_CHANNEL_3
#define chnl_bridge_1_low    TIM_CHANNEL_4
#define chnl_bridge_2_high   TIM_CHANNEL_2
#define chnl_bridge_2_low    TIM_CHANNEL_1
#define motor_PWM_PERIOD     (48 * 20)
#define motor_HARD_MAX       ((int32_t)(motor_PWM_PERIOD * 0.90f))
#define motor_DEAD_TIME_TICK 24
#endif // FACTORY_MOTOR_CONTROLLER

#endif /* CONFIG_H_ */
