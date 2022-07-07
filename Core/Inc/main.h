/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LCD_BL_CTRL_Pin GPIO_PIN_7
#define LCD_BL_CTRL_GPIO_Port GPIOF
#define UD_Pin GPIO_PIN_1
#define UD_GPIO_Port GPIOA
#define LR_Pin GPIO_PIN_2
#define LR_GPIO_Port GPIOA
#define XL_Pin GPIO_PIN_4
#define XL_GPIO_Port GPIOH
#define YD_Pin GPIO_PIN_5
#define YD_GPIO_Port GPIOH
#define YU_Pin GPIO_PIN_3
#define YU_GPIO_Port GPIOA
#define XR_Pin GPIO_PIN_4
#define XR_GPIO_Port GPIOA
#define GPIO_Input_Pin GPIO_PIN_5
#define GPIO_Input_GPIO_Port GPIOA
#define GPIO_InputA6_Pin GPIO_PIN_6
#define GPIO_InputA6_GPIO_Port GPIOA
#define GPIO_InputA7_Pin GPIO_PIN_7
#define GPIO_InputA7_GPIO_Port GPIOA
#define SELB_Pin GPIO_PIN_4
#define SELB_GPIO_Port GPIOC
#define PWM_BL_Pin GPIO_PIN_0
#define PWM_BL_GPIO_Port GPIOB
#define GPIO_InputA8_Pin GPIO_PIN_8
#define GPIO_InputA8_GPIO_Port GPIOA
#define GPIO_Output_Pin GPIO_PIN_9
#define GPIO_Output_GPIO_Port GPIOA
#define GPIO_OutputA10_Pin GPIO_PIN_10
#define GPIO_OutputA10_GPIO_Port GPIOA
#define GPIO_OutputA11_Pin GPIO_PIN_11
#define GPIO_OutputA11_GPIO_Port GPIOA
#define GPIO_OutputA12_Pin GPIO_PIN_12
#define GPIO_OutputA12_GPIO_Port GPIOA
#define JTAG_SWDIO_Pin GPIO_PIN_13
#define JTAG_SWDIO_GPIO_Port GPIOA
#define JTAG_SWCLK_Pin GPIO_PIN_14
#define JTAG_SWCLK_GPIO_Port GPIOA
#define STBYB_Pin GPIO_PIN_3
#define STBYB_GPIO_Port GPIOD
#define SHTDNb_Pin GPIO_PIN_4
#define SHTDNb_GPIO_Port GPIOD
#define GPIO_OUT_BEEP_Pin GPIO_PIN_5
#define GPIO_OUT_BEEP_GPIO_Port GPIOD
#define CTP_INT_Pin GPIO_PIN_3
#define CTP_INT_GPIO_Port GPIOB
#define CTP_RST_Pin GPIO_PIN_4
#define CTP_RST_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
