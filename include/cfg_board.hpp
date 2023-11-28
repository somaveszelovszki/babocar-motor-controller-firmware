#pragma once

#include <micro/port/can.hpp>
#include <micro/port/gpio.hpp>
#include <micro/port/timer.hpp>
#include <micro/port/uart.hpp>

#if defined STM32F4

extern CAN_HandleTypeDef  hcan1;
extern TIM_HandleTypeDef  htim1;
extern TIM_HandleTypeDef  htim2;
extern TIM_HandleTypeDef  htim3;
extern TIM_HandleTypeDef  htim4;
extern TIM_HandleTypeDef  htim5;
extern TIM_HandleTypeDef  htim7;
extern TIM_HandleTypeDef  htim8;
extern UART_HandleTypeDef huart2;

#define can_Vehicle                 micro::can_t{ &hcan1 }

#define gpio_Led                    micro::gpio_t{ GPIOA, GPIO_PIN_5 }

#define tim_DC_Motor                micro::timer_t{ &htim1 }
#define timChnl_DC_Motor_Bridge1    TIM_CHANNEL_1
#define timChnl_DC_Motor_Bridge2    TIM_CHANNEL_2

#define tim_System                  micro::timer_t{ &htim2 }

#define tim_SteeringServo           micro::timer_t{ &htim5 }
#define timChnl_FrontSteeringServo  TIM_CHANNEL_1
#define timChnl_RearSteeringServo   TIM_CHANNEL_2

#define tim_ServoX                  micro::timer_t{ &htim4 }
#define timChnl_ServoX              TIM_CHANNEL_1

#define tim_RcCtrl                  micro::timer_t{ &htim3 }
#define timChnl_RcCtrlSteer         TIM_CHANNEL_4
#define timChnl_RcCtrlAccel         TIM_CHANNEL_3
#define timChnl_RcCtrlModeSelect    TIM_CHANNEL_2

#define tim_ControlLoop             micro::timer_t{ &htim7 }

#define tim_Encoder                 micro::timer_t{ &htim8 }
#define timChnl_RcRecv1             TIM_CHANNEL_1
#define timChnl_RcRecv2             TIM_CHANNEL_2

#define uart_Debug                  micro::uart_t{ &huart2 }

#else // !STM32F4

#define can_Vehicle                 micro::can_t{}

#define gpio_Led                    micro::gpio_t{}

#define tim_DC_Motor                micro::timer_t{}
#define timChnl_DC_Motor_Bridge1    1
#define timChnl_DC_Motor_Bridge2    2

#define tim_System                  micro::timer_t{}

#define tim_SteeringServo           micro::timer_t{}
#define timChnl_FrontSteeringServo  1
#define timChnl_RearSteeringServo   2

#define tim_ServoX                  micro::timer_t{}
#define timChnl_ServoX              0

#define tim_RcCtrl                  micro::timer_t{}
#define timChnl_RcCtrlSteer         4
#define timChnl_RcCtrlAccel         3
#define timChnl_RcCtrlModeSelect    2

#define tim_ControlLoop             micro::timer_t{}

#define tim_Encoder                 micro::timer_t{}
#define timChnl_RcRecv1             1
#define timChnl_RcRecv2             2

#define uart_Debug                  micro::uart_t{}

#endif // !STM32F4

#define PANEL_VERSION               0x02

#define QUARTZ_FREQ                 megahertz_t(20)
