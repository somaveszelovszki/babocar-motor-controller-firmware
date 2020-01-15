/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "config.h"
#include "pi_controller.h"
#include "encoder.h"
#include "dc_motor.h"

#include <micro/utils/types.h>
#include <micro/math/numeric.h>

#include <micro/panel/panelLink.h>
#include <micro/panel/MotorPanelData.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

static volatile encoder_t encoder;
static volatile uint32_t rc_recv_in_speed = 1500;
static volatile uint32_t last_rc_recv_in_speed_time = 0;
static volatile pi_controller_t speedCtrl = PI_CONTROLLER_INIT(SPEED_CTRL_P, SPEED_CTRL_I, SPEED_CTRL_INTEGRAL_MAX, -1.0f, 1.0f, SPEED_CTRL_DEADBAND_MPS);
static volatile float speed_measured_mps = 0.0f;
static volatile int32_t distance_mm = 0;
static volatile panelLink_t panelLink;
static volatile bool newCmd = false;
static volatile uint32_t lastConnectedTime = 0;

motorPanelDataIn_t inData;
motorPanelDataOut_t outData;

static bool useSafetyEnableSignal = true;
static bool isMotorEnabled = false;

static float targetSpeed_mps = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void updateMotorEnabled(void) {
    static const uint32_t BOUNCE_LIMIT = 3;
    static uint8_t bounce_cntr = 0;
    static uint32_t prev_rc_recv_in_speed_time = 0;

    bool enabled = false;

    if (useSafetyEnableSignal) {
        const uint32_t recvPwm = rc_recv_in_speed;
        const bool en = (recvPwm > 1700 && recvPwm < 2150);

        if (en == isMotorEnabled) {
            enabled = isMotorEnabled;
            bounce_cntr = 0;
        } else if (++bounce_cntr > BOUNCE_LIMIT) {
            enabled = en;
            bounce_cntr = 0;
        } else {
            enabled = isMotorEnabled;
        }
    } else {
        enabled = true;
    }

    isMotorEnabled = enabled;
}

void fillTxData(motorPanelDataOut_t *txData) {
    txData->distance_mm      = distance_mm;
    txData->actualSpeed_mmps = (int16_t)(speed_measured_mps * 1000);
    txData->isMotorEnabled   = isMotorEnabled;
}

void handleRxData(motorPanelDataIn_t *rxData) {
    targetSpeed_mps = rxData->targetSpeed_mmps / 1000.0f;

    // TODO
//    __disable_irq();
//    speedCtrl.P = rxData->controller_P;
//    speedCtrl.I = rxData->controller_I;
//    speedCtrl.integral_max = rxData->controller_integral_max;
//    __enable_irq();

    useSafetyEnableSignal = true;// TODO !!(rxData->flags & MOTOR_PANEL_FLAG_USE_SAFETY_SIGNAL);

}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_ADC_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_IC_Start_IT(tim_rc_recv, chnl_rc_recv);

  speedCtrl.desired = 0.0f;

  dc_motor_initialize();
  encoder_initialize((encoder_t*)&encoder, ENCODER_MAX_VALUE);

//  motorPanelDataOut_t txDataBuffer;
//  panelLink_initialize((panelLink_t*)&panelLink, PanelLinkRole_Active, uart_cmd,
//      &rxDataBuffer, sizeof(motorPanelDataIn_t), MOTOR_PANEL_LINK_IN_TIMEOUT_MS,
//      &txDataBuffer, sizeof(motorPanelDataOut_t), MOTOR_PANEL_LINK_OUT_PERIOD_MS);

  bool isConnected = false;
  uint32_t lastCmdTime               = 0;
  uint32_t nextSpeedSendTime         = HAL_GetTick();
  uint32_t nextSafetySignalCheckTime = HAL_GetTick();
  uint32_t nextLedToggleTime         = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
      const uint32_t currentTime = HAL_GetTick();

      if (currentTime >= nextSafetySignalCheckTime) {
          nextSafetySignalCheckTime += 10;
          updateMotorEnabled();
      }

      if (newCmd) {
          newCmd = false;
          isConnected = true;
          lastCmdTime = currentTime;
          handleRxData(&inData);
      }

      if (isConnected) {
          lastConnectedTime = HAL_GetTick();
          if (currentTime - lastCmdTime > 200) {
              HAL_UART_AbortReceive_IT(uart_cmd);
              isConnected = false;
              targetSpeed_mps = 0.0f;
              HAL_Delay(5);
          }

          if (currentTime >= nextSpeedSendTime) {
              nextSpeedSendTime += 5;
              fillTxData(&outData);
              HAL_UART_Transmit_DMA(uart_cmd, (uint8_t*)&outData, sizeof(motorPanelDataOut_t));
          }
      } else {
          char c = '\0';
          while (c != 'S') {
              HAL_UART_Receive(uart_cmd, (uint8_t*)&c, 1, 250);
              HAL_GPIO_TogglePin(gpio_user_led, gpio_pin_user_led);
          }

          HAL_UART_Transmit_DMA(uart_cmd, (uint8_t*)&outData, sizeof(motorPanelDataOut_t));
          HAL_Delay(5);
          HAL_UART_Receive_DMA(uart_cmd, (uint8_t*)&inData, sizeof(motorPanelDataIn_t));

          isConnected = true;
          lastCmdTime = currentTime;
          nextSpeedSendTime = currentTime + 5;
          nextLedToggleTime = currentTime + 500;
      }

      if (currentTime >= nextLedToggleTime) {
          nextLedToggleTime += (isConnected ? 500 : 250);
          HAL_GPIO_TogglePin(gpio_user_led, gpio_pin_user_led);
      }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim == tim_rc_recv) {
	    const uint32_t duty = __HAL_TIM_GET_COMPARE(tim_rc_recv, chnl_rc_recv);
	    if (duty > 850 && duty < 2150) {
	        rc_recv_in_speed = duty;
	        last_rc_recv_in_speed_time = HAL_GetTick();
	    }
		__HAL_TIM_SET_COUNTER(tim_rc_recv, 0); 	//resets counter after input capture interrupt occurs
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == uart_cmd) {
        newCmd = true;
        //panelLink_onNewRxData((panelLink_t*)&panelLink, sizeof(motorPanelDataIn_t) - huart->hdmarx->Instance->CNDTR);
        //HAL_UART_Receive_DMA(uart_cmd, (uint8_t*)&rxDataBuffer, sizeof(motorPanelDataIn_t));
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == tim_speedControllerPeriod) {
        encoder_update((encoder_t*)&encoder);
        speed_measured_mps = encoder.last_diff / ENCODER_INCR_PER_MM / (ENCODER_PERIOD_US / 1000.0f);
        distance_mm = encoder.num_incr / ENCODER_INCR_PER_MM;

        speedCtrl.desired = isMotorEnabled && HAL_GetTick() - lastConnectedTime < 500 ? targetSpeed_mps : 0.0f;
        pi_controller_update((pi_controller_t*)&speedCtrl, speed_measured_mps);
        dc_motor_write(speedCtrl.output);
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
