#ifndef CFG_BOARD_H
#define CFG_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"

extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;

#define can_Vehicle                 (&hcan1)

#define tim_DC_Motor                (&htim1)
#define timChnl_DC_Motor_Bridge1    TIM_CHANNEL_1
#define timChnl_DC_Motor_Bridge2    TIM_CHANNEL_2

#define tim_System                  (&htim2)

#define tim_SteeringServo           (&htim3)
#define timChnl_FrontSteeringServo  TIM_CHANNEL_1
#define timChnl_RearSteeringServo   TIM_CHANNEL_2

#define tim_ServoX                  (&htim4)
#define timChnl_ServoX              TIM_CHANNEL_1

#define tim_RcCtrl                  (&htim5)
#define timChnl_RcCtrlDirectAccel   TIM_CHANNEL_1
#define timChnl_RcCtrlDirectSteer   TIM_CHANNEL_2
#define timChnl_RcCtrlSafetyAccel   TIM_CHANNEL_3
#define timChnl_RcCtrlSafetySteer   TIM_CHANNEL_4

#define tim_ControlLoop             (&htim7)

#define tim_Encoder                 (&htim8)
#define timChnl_RcRecv1             TIM_CHANNEL_1
#define timChnl_RcRecv2             TIM_CHANNEL_2

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // CFG_BOARD_H
