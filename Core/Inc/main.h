/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DEADTIME_CLK 0
#define FET_Temp_Pin GPIO_PIN_0
#define FET_Temp_GPIO_Port GPIOC
#define Encoder_MOSI_Pin GPIO_PIN_1
#define Encoder_MOSI_GPIO_Port GPIOC
#define Encoder_MISO_Pin GPIO_PIN_2
#define Encoder_MISO_GPIO_Port GPIOC
#define Encoder_CS_Pin GPIO_PIN_3
#define Encoder_CS_GPIO_Port GPIOC
#define DC_Voltage_Pin GPIO_PIN_0
#define DC_Voltage_GPIO_Port GPIOA
#define Motor_Temp_Pin GPIO_PIN_1
#define Motor_Temp_GPIO_Port GPIOA
#define ControlBus_TX_Pin GPIO_PIN_2
#define ControlBus_TX_GPIO_Port GPIOA
#define ControlBus_RX_Pin GPIO_PIN_3
#define ControlBus_RX_GPIO_Port GPIOA
#define ControlBus_TXEN_Pin GPIO_PIN_4
#define ControlBus_TXEN_GPIO_Port GPIOA
#define AL_Pin GPIO_PIN_7
#define AL_GPIO_Port GPIOA
#define A_Current_Pin GPIO_PIN_4
#define A_Current_GPIO_Port GPIOC
#define B_Current_Pin GPIO_PIN_5
#define B_Current_GPIO_Port GPIOC
#define BL_Pin GPIO_PIN_0
#define BL_GPIO_Port GPIOB
#define CL_Pin GPIO_PIN_1
#define CL_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_2
#define LED_GPIO_Port GPIOB
#define Encoder_SCK_Pin GPIO_PIN_10
#define Encoder_SCK_GPIO_Port GPIOB
#define EN_GATE_Pin GPIO_PIN_7
#define EN_GATE_GPIO_Port GPIOC
#define DC_CAL_Pin GPIO_PIN_8
#define DC_CAL_GPIO_Port GPIOC
#define nFAULT_Pin GPIO_PIN_9
#define nFAULT_GPIO_Port GPIOC
#define AH_Pin GPIO_PIN_8
#define AH_GPIO_Port GPIOA
#define BH_Pin GPIO_PIN_9
#define BH_GPIO_Port GPIOA
#define CH_Pin GPIO_PIN_10
#define CH_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define TP0_Pin GPIO_PIN_10
#define TP0_GPIO_Port GPIOC
#define TP1_Pin GPIO_PIN_11
#define TP1_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
