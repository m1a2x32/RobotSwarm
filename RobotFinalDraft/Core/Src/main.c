/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include "stdio.h"
#include "stdbool.h"

static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

volatile long duration;
unsigned long millis;
float distance;


// FEEDBACK SYSTEM VARIABLES
int lTheta = 0, prevLTheta = 0, lwHighT = 0;
int rTheta = 0, prevRTheta = 0, rwHighT = 0;
int tCycle = 1070;
float lTurns = 0, rTurns = 0;
// UART
char buffer[255];
// STATES
States state = FORWARD, prevState = STOP;
// REMOTE MANAGEMENT
int automaticMode = 1;
char remoteCommand[1];
char setting[1];


int getThetaAngle(int tHigh);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  distance = getDistance();
  millis = HAL_GetTick();

  while (1)
  {
	  if(HAL_GetTick() - millis > 1000)
	  {
		  sprintf(buffer, "------Right Wheel----------|--------Left Wheel-------\n\rRevolutions: %0.3f cm/sec  |  Revolutions: %0.3f cm/sec \r\n", rTurns/1000, lTurns/1000);
		  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
		  sprintf(buffer,"%0.3f\r\n%0.3f\r\n", rTurns/1000, lTurns/1000);
		  HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
		  millis = HAL_GetTick();
	  }
	  GetMode();

	  if (automaticMode == 1)
	  {
			  switch(state)
			  {
				  case FORWARD:

					  if(distance <= 10.0 && distance != 0.0)
					  {
						  if(duration % 2 == 0)
						  {
							  state = LEFT;
						  }
						  else
						  {
							  state = RIGHT;
						  }
					  }
					  else
					  {
						  distance = getDistance();
						  direction(FORWARD);
					  }
					  break;
				  case LEFT:
					  if(prevState != state)
					  {
						  direction(STOP);
						  distance = getDistance();
						  prevState = state;
					  }
					  if(HAL_GetTick() - millis >= 250)
					  {
						  if(distance > 15.0)
						  {
							  state = FORWARD;
						  }
						  else
						  {
							  distance = getDistance();
							  direction(LEFT);
						  }
						  millis = HAL_GetTick();
					  }
					  break;
				  case RIGHT:
					  if(prevState != state)
					  {
						  direction(STOP);
						  distance = getDistance();
						  prevState = state;
					  }

					  if(HAL_GetTick() - millis > 250)
					  {
						  if(distance > 15.0)
						  {
							  state = FORWARD;
						  }
						  else
						  {
							  distance = getDistance();
							  direction(RIGHT);
						  }
						  millis = HAL_GetTick();
					  }
					  break;
				  case STOP:
					  direction(STOP);
					  prevState = state;
					  break;
				  case BACKWARDS:
					  break;
			  }
	  	  }
  }
}
// ----------------------------------------- FUNCTIONS ---------------------------------------------------------------------------

void GetMode()
{
	setting[0] = '\0';
	HAL_UART_Receive(&huart1, (uint8_t *)setting, 1, 1);
 	 if (strcmp(setting, "M") == 0)
 	 {
 		 direction(STOP);
 		 automaticMode = 0;
 	 }
 	 else if (strcmp(setting, "A") == 0)
 	 {
 		 state = FORWARD;
 		 automaticMode = 1;
 	 }

 	 if (automaticMode == 0)
 	 {
 		if (strcmp(setting, "F") == 0)
 		{
 			direction(FORWARD);
 		}
 		else if (strcmp(setting, "L") == 0)
 		{
 			direction(LEFT);
 		}
 		else if (strcmp(setting, "R") == 0)
 		{
 			direction(RIGHT);
 		}
 		else if (strcmp(setting, "S") == 0)
 		{
 			direction(STOP);
 		}
 	 }
}

void direction(States dir)
//	Left Wheel (CCR2) - Right Wheel (CCR1)
// 		BACKWARDS...FORWARD
// 	 		 1275 - 1720
{
	switch(dir)
	{
	case FORWARD:
		TIM2->CCR1 = 1450;
		TIM2->CCR2 = 1550;
		break;
	case RIGHT:
	    TIM2->CCR1 = 1550;
		TIM2->CCR2 = 1550;
		break;
	case LEFT:
		TIM2->CCR1 = 1450;
		TIM2->CCR2 = 1450;
		break;
	case STOP:
		TIM2->CCR1 = 1500;
		TIM2->CCR2 = 1500;
		break;
	case BACKWARDS:
		TIM2->CCR1 = 1550;
		TIM2->CCR2 = 1450;
		break;
	}
}
int getThetaAngle(int tHigh)
/* Calculating theta angle for angular velocity calculations in TIM2->C2*/
{
	int angle = 0, dutyCycle = 0,  dcMin = 29, dcMax = 971;
	dutyCycle = (1000 * tHigh) / tCycle;
	angle = 359 - ((dutyCycle - dcMin) * 360) / (dcMax - dcMin + 1);

	if(angle < 0)
		angle = 0;
	else if(angle > 359)
		angle = 359;

	return angle;
}
// ----------------------------------------- ULTRASONIC ---------------------------------------------------------------------------
void setupUltrasonic(void)
{
	// D8 - TRIG pin setup
    GPIOA->MODER &= ~(3UL<<18); // Clear mode
    GPIOA->MODER |= (1UL<<18);  // Mode to output
    GPIOA->PUPDR &= ~(3UL<<18); // Select no pull-up/pull-down
    // D9 - ECHO pin setup
    GPIOC->MODER &= ~(3UL<<14); // Mode to input
    GPIOC->PUPDR |= (2UL<<14);  // Select pull-down
    NVIC_EnableIRQ(EXTI9_5_IRQn); // Enable interrupt handler
}

float getDistance()
{
	  GPIOA->ODR &= ~trigpin; // Set trigpin to LOW
	  //HAL_Delay(3);
	  GPIOA->ODR |= trigpin; // Set trigpin to HIGH
	  ///HAL_Delay(10);
	  GPIOA->ODR &= ~trigpin; // Set trigpin to LOW
	  return (duration + 0.0f)*(0.034 / 2);
}

// ----------------------------------------- IRQ interrupt handlers ---------------------------------------------------------------------------

void EXTI9_5_IRQHandler(void)
{
    uint32_t EchoState = (GPIOC->IDR & echopin);
    unsigned long startMicros = 0;
    if((EXTI->PR & EXTI_PR_PIF7) != 0)
    {
        while(EchoState != 0)
        {
            EchoState = (GPIOC->IDR & echopin);
            startMicros++;
        }
        duration = startMicros;
        EXTI->PR |= EXTI_PR_PIF7;
    }
}

void TIM3_IRQHandler(void)
{
	if(TIM3->SR & TIM_SR_UIF)
	{
		lwHighT = TIM3->CCR1;
		TIM3->SR &= ~TIM_SR_UIF; // Reset interrupt flag
	}
}

void TIM4_IRQHandler(void)
{
	if(TIM4->SR & TIM_SR_UIF)
	{
		rwHighT = TIM4->CCR1;
		TIM4->SR &= ~TIM_SR_UIF; // Reset interrupt flag
	}
}

void TIM2_IRQHandler(void)
{
	if(TIM2->SR & TIM_SR_CC3IF_Msk)
	{
		// Speed calculation
		// Get theta angle of both wheels
		lTheta = getThetaAngle(lwHighT);

		// If there has been a rotation, we ge this by comparing previous angle and current angle
		// A2- A1 / 20 ms = Linear velocity
		// Linear velocity * radius(3.5cm) = velocity/20 ms
		// to achive velocity in seconds(cm/s) divide by 1000, this is done when printing
		if((lTheta < 90) && (prevLTheta > 270))
			lTurns = abs(((lTheta-prevLTheta) / 20)*3.5);

		// Save theta angle
		prevLTheta = lTheta;
		TIM2->SR &= ~TIM_SR_CC3IF_Msk; // Reset interrupt flag
	}
	if(TIM2->SR & TIM_SR_CC4IF_Msk)
	{
		rTheta = getThetaAngle(rwHighT);

		if((rTheta < 90) && (prevRTheta > 270))
			rTurns = abs(((rTheta-prevRTheta) / 20)*3.5);

		prevRTheta = rTheta;
		TIM2->SR &= ~TIM_SR_CC4IF_Msk; // Reset interrupt flag
	}
}

// ----------------------------------------- SERVO 360 ---------------------------------------------------------------------------
void rightServoSetup(void)
{ // PA5 control --- PA11 feedback
	// Right Wheel control pin
    GPIOA->MODER |= (GPIOA->MODER & ~GPIO_MODER_MODER5) | (0b10 << GPIO_MODER_MODER5_Pos); // Alternate Function Mode
    // AF1 TIM2 CH2
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~GPIO_AFRL_AFRL5) | (0b0001 <<GPIO_AFRL_AFRL5_Pos);

    // Right Wheel feedbaCk - AF MODE
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER11) | (0b10 << GPIO_MODER_MODER11_Pos);
    // AF10 TIM4 CH1
   	GPIOA->AFR[1] = (GPIOA->AFR[1] & ~GPIO_AFRH_AFRH3) | (0b1010 << GPIO_AFRH_AFRH3_Pos);
}

void leftServoSetup(void)
{ // PB3 control --- PA6 feedback
	// Left Wheel control pin
	GPIOB->MODER |= (GPIOB->MODER & ~GPIO_MODER_MODER3) | (0b10 << GPIO_MODER_MODER3_Pos); // Alternate Function Mode
	// AF1 TIM2 CH1
	GPIOB->AFR[0] = (GPIOB->AFR[0] & ~GPIO_AFRL_AFRL3) | (0b0001 <<GPIO_AFRL_AFRL3_Pos);
	// Left Wheel feedback - AF MODE
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER6) | (0b10 << GPIO_MODER_MODER6_Pos);
    // AF2 TIM3 CH1
   	GPIOA->AFR[0] = (GPIOA->AFR[0] & ~GPIO_AFRL_AFRL6) | (0b0010 << GPIO_AFRL_AFRL6_Pos);
   	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_0;
}

// ----------------------------------------- SETUP FUNCTIONS ---------------------------------------------------------------------------
// ----------------------------------------- SETUP FUNCTIONS ---------------------------------------------------------------------------
void GPIO_Init(void)
//______________________________________________________________________________
//| 			  Port A 				|			TIM 2						|
//| D13 - PA_5 (Right Wheel output)		|	Servo Control 			- 50Hz		|
//| D12 - PA_6 (Right Wheel input)		|	Channel 1 -> Right Wheel			|
//| D11 - PA_7 (Left Wheel input)		|	Channel 2 -> Left Wheel				|
//| D08 - PA_9 (Ultrasonic trigger)		|	Channel 3 -> Theta angle calculation|
//| ------------- Port B -------------------------- TIM 3 ----------------------|
//| D03 - PB_3 (Left Wheel output)		|	Feedback for left servo - 910Hz		|
//| -------------- Port C ------------------------- TIM 4 ----------------------|
//| D09 - PC_7 (Ultrasonic echo)		|   Feedback for right servo- 910Hz		|
//|_____________________________________________________________________________|
{
	// Enable Clocks
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN; // Enable TIM2 TIM3 TIM4 clock
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	// Enable Ports
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN; // Enable PORT A, B, C

	// Timer Setup
	TIM2_CONTROL();
	TIM3_FEEDBACK_LEFTWHEEL();
	TIM4_FEEDBACK_RIGHTWHEEL();

    // Hardware Setup
    setupUltrasonic();
    leftServoSetup();
    rightServoSetup();

    NVIC_SetPriority(TIM3_IRQn, 1);
    NVIC_SetPriority(TIM4_IRQn, 2);
    NVIC_SetPriority(TIM2_IRQn, 3);
    NVIC_SetPriority(EXTI9_5_IRQn, 4);

    // Ultrasonic interrupt EXTI7 Port C
    SYSCFG->EXTICR[1] |= (0b010 << SYSCFG_EXTICR2_EXTI7_Pos);
	// Rising trigger EXTI7
	EXTI->RTSR |= EXTI_RTSR_RT7;
    // Interrupt Mask Register not masked EXTI7 &
    EXTI->IMR |= EXTI_IMR_IM7;
 }

void TIM2_CONTROL(void)
// TIM2 will control the output of the robot
// Channel 1 & 2 will be used to send PWM signals to control the servo
// Channel 3 will be used to calculate the theta angle of the wheel at a 50Hz frequency
{
 // 16MHz clock speed
 TIM2->PSC = 16-1; // 16.000.000 / 16 = 1.000.000
 TIM2->ARR = FiftyHertz; // 1.000.000/50Hz = 20.000
 // Reset counter
 TIM2->CNT = 0;
 // CHANNEL 1
 TIM2->CCMR1 |= (0b0110 << TIM_CCMR1_OC1M_Pos);		// PWM mode 1
 TIM2->CCMR1 &= ~(0b1 << TIM_CCMR1_OC1PE_Pos);   	// Disable preload register
 TIM2->CCMR1 &= ~(0b11<< TIM_CCMR1_CC1S_Pos);  		// Set to output
 TIM2->CCR1 = 1500;
 TIM2->CCER |= (0b1 << TIM_CCER_CC1E_Pos); 		// Capture/compare output
 // CHANNEL 2
 TIM2->CCMR1 |= (0b0110 << TIM_CCMR1_OC2M_Pos); // PWM mode 1
 TIM2->CCMR1 &= ~(0b1 << TIM_CCMR1_OC2PE_Pos);   // Disable preload register
 TIM2->CCMR1 &= ~(0b11 << TIM_CCMR1_CC2S_Pos);   // Set to output
 TIM2->CCR2 = 1500;
 TIM2->CCER |= (0b1 << TIM_CCER_CC2E_Pos); 	// Capture/compare output
 // CHANNEL 3
 TIM2->DIER = TIM_DIER_CC3IE; // Capture/compare interrupt flag enabled
 // CHANNEL 4
 TIM2->DIER = TIM_DIER_CC4IE; // Capture/compare interrupt flag enabled
 // Enable interrupt handler
 NVIC_EnableIRQ(TIM2_IRQn);
 // Enable counter
 TIM2->CR1 |= TIM_CR1_CEN;
}

void TIM3_FEEDBACK_LEFTWHEEL(void)
// ------------- LEFT WHEEL ----------------------
// TIM3 will control the input of the robot, feedback control
{
 // 16MHz clock speed
 TIM3->PSC = 16-1; // 1Mhz to achieve result in ms
 // Reset counter
 TIM3->CNT = 0;
 // CHANNEL 1 - Duty cycle of PWM signal in CCR2
 TIM3->CCMR1 |=  (TIM3->CCMR1 & ~TIM_CCMR1_CC1S) |  (0b01<<TIM_CCMR1_CC1S_Pos); // Input mode mapped on TI1
 TIM3->CCER &= ~((0b11 << TIM_CCER_CC1P_Pos) | (0b11 << TIM_CCER_CC1NP_Pos)); 	// Clear Register //noninverted/rising edge
 TIM3->CCER |= (0b01 << TIM_CCER_CC1P_Pos) | (0b01 << TIM_CCER_CC1NP_Pos);   	// inverted/falling edge
 TIM3->CCER |= TIM_CCER_CC1E_Msk;   											// Input IC1 mapped on TI1
 // CHANNEL 2 - Measure period in CCR1
 TIM3->CCMR1 |= (TIM3->CCMR1 & ~TIM_CCMR1_CC2S) |  (0b01<<TIM_CCMR1_CC2S_Pos);	 // Input mode mapped on TI2
 TIM3->CCER &= ~((0b11 << TIM_CCER_CC2P_Pos) | (0b11 << TIM_CCER_CC2NP_Pos)); 	 // noninverted/rising edge
 //TIM3->CCER |= (0b01 << TIM_CCER_CC2P_Pos) | (0b01 << TIM_CCER_CC2NP_Pos); 	 // inverted/falling edge
 TIM3->CCER |= TIM_CCER_CC2E_Msk;   											 // Capture/Compare 2 output enable

 // Filtered Timer Input 1 (TI1FP1)
 TIM3->SMCR = (TIM3->SMCR & ~TIM_SMCR_TS) | (0b101 << TIM_SMCR_TS_Pos);
 // Reset Mode
 TIM3->SMCR = (TIM3->SMCR & ~TIM_SMCR_SMS) | (0b100 << TIM_SMCR_SMS_Pos);

 // Enable Capture / Compare interrupt for channel 1 and 2
 TIM3->DIER |= TIM_DIER_UIE | TIM_DIER_CC1IE | TIM_DIER_CC2IE;
 // Enable interrupt handler
 NVIC_EnableIRQ(TIM3_IRQn);
 // Enable counter
 TIM3->CR1 |= TIM_CR1_CEN | 0b10<<(TIM_CR1_CMS_Pos);
}

void TIM4_FEEDBACK_RIGHTWHEEL(void)
// ------------- Right WHEEL ----------------------
// TIM4 will control the input of the robot, feedback control
{
 // 16MHz clock speed
 TIM4->PSC = 16-1; // 1Mhz to achieve result in ms
 // Reset counter
 TIM4->CNT = 0;
 // CHANNEL 1 - Duty cycle of PWM signal in CCR2
 TIM4->CCMR1 |=  (TIM3->CCMR1 & ~TIM_CCMR1_CC1S) |  (0b01<<TIM_CCMR1_CC1S_Pos); // Input mode mapped on TI1
 TIM4->CCER &= ~((0b11 << TIM_CCER_CC1P_Pos) | (0b11 << TIM_CCER_CC1NP_Pos)); 	// Clear Register //noninverted/rising edge
 TIM4->CCER |= (0b01 << TIM_CCER_CC1P_Pos) | (0b01 << TIM_CCER_CC1NP_Pos);   	// inverted/falling edge
 TIM4->CCER |= TIM_CCER_CC1E_Msk;   											// Input IC1 mapped on TI1
 // CHANNEL 2 - Measure period in CCR1
 TIM4->CCMR1 |= (TIM3->CCMR1 & ~TIM_CCMR1_CC2S) |  (0b01<<TIM_CCMR1_CC2S_Pos);	 // Input mode mapped on TI2
 TIM4->CCER &= ~((0b11 << TIM_CCER_CC2P_Pos) | (0b11 << TIM_CCER_CC2NP_Pos)); 	 // noninverted/rising edge
 //TIM4->CCER |= (0b01 << TIM_CCER_CC2P_Pos) | (0b01 << TIM_CCER_CC2NP_Pos); 	 // inverted/falling edge
 TIM4->CCER |= TIM_CCER_CC2E_Msk;   											 // Capture/Compare 2 output enable

 // Filtered Timer Input 1 (TI1FP1)
 TIM4->SMCR = (TIM3->SMCR & ~TIM_SMCR_TS) | (0b101 << TIM_SMCR_TS_Pos);
 // Reset Mode
 TIM4->SMCR = (TIM3->SMCR & ~TIM_SMCR_SMS) | (0b100 << TIM_SMCR_SMS_Pos);

 // Enable Capture / Compare interrupt for channel 1 and 2
 TIM4->DIER |= TIM_DIER_UIE | TIM_DIER_CC1IE | TIM_DIER_CC2IE;
 // Enable interrupt handler
 NVIC_EnableIRQ(TIM4_IRQn);
 // Enable counter
 TIM4->CR1 |= TIM_CR1_CEN | 0b01<<(TIM_CR1_CMS_Pos);
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

// ------------ COMMUNICATION------------------------
static void MX_USART1_UART_Init(void)
// TX: PC3 - RX: PC4
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}


void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

