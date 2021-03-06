/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwip/opt.h"
#include "lwip/timeouts.h"
#include "lwip/tcpip.h"
#include "netif/ethernet.h"
#include "netif/etharp.h"
#include "lwip/tcp.h"
#include "ethernetif.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "lwip/apps/fs.h"
#include "ethernetConf.h"
#include "leds.h"

#include "driveControl.h"
#include "speedMeasure.h"
#include "tests.h"

#include "ros_proto.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct 
{
  float speed;
  float angle;
}trajectoryData_t;

typedef struct
{
  uint8_t errorLight;
  uint8_t stopLight;
}lightData_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BREAK_DIRECTION_R_Pin GPIO_PIN_3
#define BREAK_DIRECTION_R_GPIO_Port GPIOE
#define BREAK_DIRECTION_L_Pin GPIO_PIN_4
#define BREAK_DIRECTION_L_GPIO_Port GPIOE
#define BREAK_LOW_Pin GPIO_PIN_0
#define BREAK_LOW_GPIO_Port GPIOF
#define BREAK_LOW_EXTI_IRQn EXTI0_IRQn
#define EMERGANSY_BREAK_Pin GPIO_PIN_1
#define EMERGANSY_BREAK_GPIO_Port GPIOF
#define EMERGANSY_BREAK_EXTI_IRQn EXTI1_IRQn
#define BREAK_RESET_Pin GPIO_PIN_2
#define BREAK_RESET_GPIO_Port GPIOF
#define BREAK_RESET_EXTI_IRQn EXTI2_IRQn
#define ROS_CONNECT_INDICATOR_Pin GPIO_PIN_8
#define ROS_CONNECT_INDICATOR_GPIO_Port GPIOF
#define EMERGANSY_BREAK_INDICATOR_Pin GPIO_PIN_9
#define EMERGANSY_BREAK_INDICATOR_GPIO_Port GPIOF
#define ENABLE_INDICATOR_Pin GPIO_PIN_1
#define ENABLE_INDICATOR_GPIO_Port GPIOG
#define TURN_SIGNAL_RIGHT_Pin GPIO_PIN_10
#define TURN_SIGNAL_RIGHT_GPIO_Port GPIOG
#define TURN_SIGNAL_LEFT_Pin GPIO_PIN_15
#define TURN_SIGNAL_LEFT_GPIO_Port GPIOG
#define DRIVE_REVERSE_Pin GPIO_PIN_7
#define DRIVE_REVERSE_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
