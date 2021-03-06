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

#include "stm32f3xx_hal.h"

typedef enum {
	FORWARD,
	LEFT,
	RIGHT,
	STOP,
	BACKWARDS
}States;

// Default exported functions
void Error_Handler(void);
void SystemClock_Config(void);

// User functions
void setupUltrasonic(void);
void leftServoSetup(void);
void rightServoSetup(void);
void TIM2_CONTROL(void);
void TIM3_FEEDBACK_LEFTWHEEL(void);
void TIM4_FEEDBACK_RIGHTWHEEL(void);
void GPIO_Init(void);
void direction(States dir);
float getDistance();
void EXTI9_5_IRQHandler(void);
void TIM4_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM2_IRQHandler(void);
extern int getThetaAngle(int tHigh);
void GetMode();

/* Private defines -----------------------------------------------------------*/
#define clockSpeed 1000000
#define FiftyHertz (clockSpeed/50)-1
#define trigpin (1UL << 9)
#define echopin (1UL << 7)


#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
