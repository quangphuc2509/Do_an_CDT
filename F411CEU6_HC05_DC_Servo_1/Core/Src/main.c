/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "stdint.h"
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim10;
DMA_HandleTypeDef hdma_tim1_ch1;
DMA_HandleTypeDef hdma_tim2_ch1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Array for GPIO
/* USER CODE BEGIN PV */
// Array for GPIO
uint16_t GPIO_PIN_Array[4] = {GPIO_PIN_15, GPIO_PIN_14, GPIO_PIN_13, GPIO_PIN_12};
uint8_t Array_1[4] = {};
uint16_t CODE[4] = {};
uint8_t Var;
uint8_t Data_Received[8];
uint8_t Data_Send[8];
uint16_t PWM_A, PWM_B;
uint8_t Chieu_quay_A, Chieu_quay_B;
double RPM_A, RPM_B;
uint32_t AA;
uint32_t Encoder_A, Encoder_A_Pre, Encoder_B, Encoder_B_Pre;
uint32_t Pos_A, Pos_B;

//PID
struct PID_Ver_1
{
	float Kp, Ki, Kd;
	float Up, Up_1;
	double Ui, Ui_1;
	float Ud, Ud_1;
	double Ui_Antiwindup;
	uint16_t Out_PWM;
	double Temp_PWM;

} PID_a, PID_b, PID_x, PID_y;

// Dong hoc Matlab
struct Kinematic_Matlab
{
	float vel, x_dot, y_dot, x, y, theta, theta_dot, b;
	float x1, y1, x1_dot, y1_dot, w1;
	float ax1, ay1;
	float w, Ratio_WR_WL;
	//////////////Quy hoach quy dao//////////////
	float x_QH_welding, y_QH_welding;
	//////////////Real Time///////////
	float vel_Real, x_dot_Real, y_dot_Real, w_Real;
	float x_Real, y_Real, x_welding_Real, y_welding_Real;
}Kinematic_Matlab_a;

struct error_PID
{
	float ek, ek_1;
} Error_a, Error_b, Error_x, Error_y;

// Anti windup
float Kb = 1935.5;

////PID
float SP_RPM_A, SP_RPM_B;

// Thong so phan cung
uint16_t Pulse_Per_Round = 2688;
uint16_t Max_PWM = 999;
float R_Wheles = 0.065;
float Khoang_cach_hai_banh = 0.17;
uint16_t HIGH_Limit_PWM = 999;
uint16_t LOW_Limit_PWM = 0;

float sum_A[20], sum_B[20];
float RPM_Avg_A, RPM_Avg_B;
int32_t delta_A, delta_B;
int i = 0;
float Time_Interrupt = 0.005;
// Pointer
struct PID_Ver_1* PID_A = &PID_a;
struct PID_Ver_1* PID_B = &PID_b;
struct error_PID* Error_A = &Error_a;
struct error_PID* Error_B = &Error_b;
struct error_PID* Error_X = &Error_x;
struct error_PID* Error_Y = &Error_y;
struct PID_Ver_1* PID_X = &PID_x;
struct PID_Ver_1* PID_Y = &PID_y;
struct Kinematic_Matlab* Kinematic_Matlab_A = &Kinematic_Matlab_a;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if defined(__GNUC__)
int _write(int fd, char * ptr, int len) {
  HAL_UART_Transmit( & huart2, (uint8_t * ) ptr, len, HAL_MAX_DELAY);
  return len;
}
#elif defined(__ICCARM__)#include "LowLevelIOInterface.h"

size_t __write(int handle,
  const unsigned char * buffer, size_t size) {
  HAL_UART_Transmit( & huart1, (uint8_t * ) buffer, size, HAL_MAX_DELAY);
  return size;
}
#elif defined(__CC_ARM)
int fputc(int ch, FILE * f) {
  HAL_UART_Transmit( & huart1, (uint8_t * ) & ch, 1, HAL_MAX_DELAY);
  return ch;
}
#endif
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
  void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
  void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
  void Bit_converter_to_direction(uint8_t Hex);
  float Average_5_times(float Var, float Temp[20]);
  void Controller(uint8_t Direction, float WR, float WL);
  void Direction_Wheles(uint8_t Tien_lui);
  void Bit_converter_to_direction(uint8_t Hex);
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);


  // Gán giá trị PID
	PID_A -> Kp = 0.0671641791;
	PID_A -> Ki = 129.9951014957;
	PID_A -> Kd = 0;
	PID_B -> Kp = 0.0671641791;
	PID_B -> Ki = 129.9951014957;
	PID_B -> Kd = 0;


	Kinematic_Matlab_A->theta = 0.5;
	Kinematic_Matlab_A->b = 0.00001;
	Kinematic_Matlab_A->Ratio_WR_WL = 1;

//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, SET);
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, RESET);
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, SET);
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_UART_Receive_IT(&huart1, &Data_Received[0], 1);
	  Direction_Wheles(Var);
	  //Controller(0, 0, 0);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 19;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 19;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 499;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Test_TIM_5_Pin|IN4_Pin|IN3_Pin|IN2_Pin
                          |IN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Test_TIM_5_Pin */
  GPIO_InitStruct.Pin = Test_TIM_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(Test_TIM_5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN4_Pin IN3_Pin IN2_Pin IN1_Pin */
  GPIO_InitStruct.Pin = IN4_Pin|IN3_Pin|IN2_Pin|IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/******************************************/

void calculateAdjugate(float A1[4][4], float adjA1[4][4])
{
    float C111, C112, C113, C114;
    float C121, C122, C123, C124;
    float C131, C132, C133, C134;
    float C141, C142, C143, C144;

    // Calculation for each element of the adjugate matrix
    C111 = A1[2][2] * A1[3][3] - A1[2][3] * A1[3][2];
    C112 = 0;
    C113 = -(A1[2][0] * A1[3][3] - A1[2][3] * A1[3][0]);
    C114 = A1[2][0] * A1[3][2]   - A1[3][0] * A1[3][3];

    C121 = 0;
    C122 =   A1[2][2]   * A1[3][3] - A1[2][3] * A1[3][2];
    C123 = -(A1[2][1]   * A1[3][3] - A1[2][3] * A1[3][1]);
    C124 =   A1[2][1]   * A1[3][2] - A1[2][2] * A1[3][1];

    C131 = 0;
    C132 = -(A1[1][2] * A1[3][3] - A1[1][3] * A1[3][2]);
    C133 =   A1[1][1] * A1[3][3] - A1[1][3] * A1[3][1];
    C134 = -(A1[1][1] * A1[3][2] - A1[1][2] * A1[3][1]);

    C141 = 0;
    C142 =   A1[1][2] * A1[2][3] - A1[1][3] * A1[2][2];
    C143 = -(A1[1][1] * A1[2][3] - A1[1][3] * A1[2][1]);
    C144 =   A1[1][1] * A1[2][2] - A1[1][2] * A1[2][1];

    // Populate the adjugate matrix
    adjA1[0][0] = C111; adjA1[0][1] = C121; adjA1[0][2] = C131; adjA1[0][3] = C141;
    adjA1[1][0] = C112; adjA1[1][1] = C122; adjA1[1][2] = C132; adjA1[1][3] = C142;
    adjA1[2][0] = C113; adjA1[2][1] = C123; adjA1[2][2] = C133; adjA1[2][3] = C143;
    adjA1[3][0] = C114; adjA1[3][1] = C124; adjA1[3][2] = C134; adjA1[3][3] = C144;
}

void det_A(float A1[4][4], float ADJ[4][4], float Final_A1[4][4])
{
    // Calculation of V11
    float V11 = A1[0][0] * (A1[1][1] * A1[2][2] * A1[3][3] + A1[1][2] * A1[2][3] * A1[3][1] + A1[1][3] * A1[2][1] * A1[3][2]
        - A1[1][3] * A1[2][2] * A1[3][1] - A1[1][1] * A1[2][3] * A1[3][2] - A1[1][2] * A1[2][1] * A1[3][3]);
    // Calculation of V13
    float V13 = A1[2][0] * (A1[0][1] * A1[1][2] * A1[3][3] + A1[0][2] * A1[1][3] * A1[3][1] + A1[0][3] * A1[1][1] * A1[3][2]
        - A1[0][3] * A1[1][2] * A1[3][1] - A1[0][1] * A1[1][3] * A1[3][2] - A1[0][2] * A1[1][1] * A1[3][3]);
    // Calculation of determinant
    float det_A1 = V11 + V13;
    // Gán giá trị
    int i, j;
    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            Final_A1[i][j] = ADJ[i][j] / det_A1;
        }
    }


}

// Multiple Matrix
void MUL_1(float FX1[4][4], float bx1[4][1], float Final[4][4])
{
    // Nhân 2 ma trận
    int m,n,k;
    // Corrected loop indices
    for(m = 0; m < 4; m++) {
        for(n = 0; n < 1; n++) {
            FX1[m][n] = 0;  // Initialize to zero before accumulation
            for(k = 0; k < 4; k++) {
                FX1[m][n] += (Final[m][k] * bx1[k][n]);
            }
        }
    }
}
/********************************************/


//////////////////////////////////////////////
//v= (WR+WL)*0.065/2;
//w= (WR-WL)*s*0.065/(2*0.17);
//xP=v*cos(th);
//yP=v*sin(th);
//thP=w;
//////////////////////////////////////////////
void Forward_Kinematic(float RPM_R, float RPM_L, struct Kinematic_Matlab* Kinematic_Matlab_A)
{
	RPM_R = RPM_R * 0.10471975512;
	RPM_L = RPM_L * 0.10471975512;
	Kinematic_Matlab_A->vel_Real = (RPM_R+RPM_L) * R_Wheles/2;
	Kinematic_Matlab_A->w_Real = ((RPM_R-RPM_L)*Kinematic_Matlab_A->Ratio_WR_WL*R_Wheles)/(2*Khoang_cach_hai_banh);
	Kinematic_Matlab_A->theta = Kinematic_Matlab_A->theta + Kinematic_Matlab_A->w_Real *Time_Interrupt;
	Kinematic_Matlab_A->x_dot_Real = Kinematic_Matlab_A->vel_Real * cos(Kinematic_Matlab_A->theta);
	Kinematic_Matlab_A->y_dot_Real = Kinematic_Matlab_A->vel_Real * sin(Kinematic_Matlab_A->theta);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart -> Instance == USART1)
	{
		if(Data_Received[0] == 'F')
		{
			SP_RPM_A = 30;
			SP_RPM_B = 30;
		}
		else if(Data_Received[0] == 'R')
		{
			SP_RPM_A = 30;
			SP_RPM_B = 12;
		}
		else if(Data_Received[0] == 'L')
		{
			SP_RPM_A = 12;
			SP_RPM_B = 30;
		}
		else if(Data_Received[0] == 'S')
		{
			SP_RPM_A = 0;
			SP_RPM_B = 0;
		}

		HAL_UART_Receive_IT(&huart1, &Data_Received[0], 1);

	}

}

//////////////******** Inverse Kinematic ********//////////////
//v = sqrt(xdot^2+ydot^2) ;
//w = 1/b * (u2*cos(th) - u1*sin(th));
//R = 0.065;  // Radius of the wheels
//L = 0.17;   // Distance between the wheels
//// Calculate wheel velocities
//WR = v/R + (L*w)/R;
//WL = v/R - (L*w)/R;
//////////////////////////////////////////////////////////////
void Inverse_Kinematic(float SP_WL, float SP_WR, struct Kinematic_Matlab* Kinematic_Matlab_A, struct PID_Ver_1* PID_X, struct PID_Ver_1* PID_Y)
{
	Kinematic_Matlab_A -> vel = sqrt(pow(Kinematic_Matlab_A->x_dot, 2) + pow(Kinematic_Matlab_A->y_dot, 2));
	Kinematic_Matlab_A -> theta_dot = (1 / Kinematic_Matlab_A -> b) + PID_X->Out_PWM * cos(Kinematic_Matlab_A->theta) - PID_Y->Out_PWM * sin(Kinematic_Matlab_A->theta);
	SP_WR = (Kinematic_Matlab_A -> vel / R_Wheles) + (Khoang_cach_hai_banh*Kinematic_Matlab_A -> theta_dot)/R_Wheles;
	SP_WL = (Kinematic_Matlab_A -> vel / R_Wheles) - (Khoang_cach_hai_banh*Kinematic_Matlab_A -> theta_dot)/R_Wheles;
}


// Chuyen tien lui thanh bit
// Bit IN1 IN2 IN3 IN4 la cac chan In cua Driver để xác định chi�?u động cơ lần lượt là Array 0 1 2 3
// Nhập vào mã hex để ra luôn giá trị các chi�?u quay của xe
// Các mã và hướng {Tiến 1010 = 0x0A ;Lùi 0101 = 0x05)
void Bit_converter_to_direction(uint8_t Hex)
{
	uint8_t Ma_code = 0x01;  // Mã code để (and) ra giá trị 0x01
	for(int i = 0; i < 4; i++)
	{
		// Mỗi lần chuyển giá trị sẽ dịch trái 1 bit
		CODE[i] = (Hex & Ma_code);
	if((Hex & Ma_code) == 0)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_Array[i], SET);
			Array_1[i] = 0;
		}
		else
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_Array[i], RESET);
			Array_1[i] = 1;
		}

		Ma_code <<= 1;


	}
}

// Chi�?u quay của xe
// Tiến là 0 ; Lùi là 1
void Direction_Wheles(uint8_t Tien_lui)
{
	if(Tien_lui == 0)
	{
		Bit_converter_to_direction(0x0A);
	}
	else if(Tien_lui == 1)
	{
		Bit_converter_to_direction(0x05);
	}

}


// Controller1
void Controller(uint8_t Direction, float WR, float WL)
{
	// Nhập setpoint tốc độ góc các bánh
	SP_RPM_A = WR;
	SP_RPM_B = WL;
	// Nhập hướng tiến lùi cho xe
	Direction_Wheles(Direction);
}


void Path_planing(float x, float y, float w)
{

}



// Tinh trung binh RPM
float Average_5_times(float Var, float Temp[20])
{
    float sum = 0, Out_Average_Var; // Initialize sum to 0
    for (int i = 0; i < 19; i++)
    {
        Temp[i] = Temp[i + 1];  // gán giá trị hiện tại vào giá trị trước
        sum += Temp[i];        // Cộng dần các giá trị vừa lưu
    }
    // Gán giá trị mới nhất
    Temp[19] = Var;
    sum += Temp[19] ;
    // Tính trung bình
    Out_Average_Var = sum / 20;
    return Out_Average_Var;
}


// Bộ antiwidup để tắt khâu tích luỹ khi nó vượt ngưỡng
float Anti_Windup(float Out_PWM, uint16_t HIGH_Limit, uint16_t LOW_Limit, float Kb)
{
    float e_reset = 0;
    float Ui_anti;

    if (Out_PWM > HIGH_Limit)
    {
        e_reset = (HIGH_Limit - Out_PWM );
    }
    else if (Out_PWM < LOW_Limit)
    {
        e_reset = (LOW_Limit - Out_PWM);
    }
    else
    {
        e_reset = 0;
    }

//    // Additional check to prevent division by zero or very large values
//    if (e_reset> pow(10,6))
//    {
//        Ui_anti = pow(10,6);  // or set to a default value
//    }

        Ui_anti = Time_Interrupt * e_reset * Kb;


    return Ui_anti;
}

// Ngat Timer PID
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim -> Instance == TIM5)
	{
		// �?�?c Encoder B
		Encoder_B = __HAL_TIM_GET_COUNTER(&htim4);
		// Lấy chênh lệch 2 khoảng Encoder
		delta_B = abs(Encoder_B - Encoder_B_Pre);
		// Nếu giá trị chênh lẹch lớn hơn nữa số xung nữa vòng là vô lý nên lấy giá trị tràn trừ cho chênh lệch
		if (delta_B  >= __HAL_TIM_GET_AUTORELOAD(&htim4) / 2)
		{
			delta_B = __HAL_TIM_GET_AUTORELOAD(&htim4) - delta_B;
		}
		//Tính giá trị RPM hiện tại
		RPM_Avg_B = (delta_B) / (Pulse_Per_Round * 4 * Time_Interrupt) * 60;
		// Tính trung bình
		RPM_B = Average_5_times(RPM_Avg_B, sum_B);


		// �?�?c Encoder A
		Encoder_A = __HAL_TIM_GET_COUNTER(&htim3);
		// Lấy chênh lệch 2 khoảng Encoder
		delta_A = abs(Encoder_A - Encoder_A_Pre);
		// Nếu giá trị chênh lẹch lớn hơn nữa số xung nữa vòng là vô lý nên lấy giá trị tràn trừ cho chênh lệch
		if (delta_A  >= __HAL_TIM_GET_AUTORELOAD(&htim3) / 2)
		{
			delta_A = __HAL_TIM_GET_AUTORELOAD(&htim3) - delta_A;
		}
		//Tính giá trị RPM hiện tại
		RPM_Avg_A = (delta_A) / (Pulse_Per_Round * 4 * Time_Interrupt) * 60;
		// Tính trung bình
		RPM_A = Average_5_times(RPM_Avg_A, sum_A);


		// PID controller
		Error_A -> ek = (SP_RPM_A - RPM_A);
		Error_B -> ek = (SP_RPM_B - RPM_B);

		// Kenh A
		PID_A -> Up =  PID_A -> Kp * Error_A -> ek;
		PID_A -> Ui = PID_A -> Ui_1 + PID_A -> Ki * Error_A -> ek_1 * Time_Interrupt + PID_A -> Ui_Antiwindup;
		PID_A -> Ud = PID_A -> Ud_1 * (Error_A -> ek - Error_A -> ek_1);
		// Antiwindup
		PID_A -> Ui_Antiwindup = Anti_Windup(PID_A -> Temp_PWM, HIGH_Limit_PWM, LOW_Limit_PWM, Kb);
		PID_A -> Temp_PWM = PID_A -> Up + PID_A -> Ui + PID_A -> Ud;

		PID_A -> Out_PWM = round(PID_A -> Temp_PWM);

			if(PID_A -> Out_PWM >= 999)
			{
				PID_A -> Out_PWM = 999;
			}
			else if (PID_A -> Out_PWM <=0 )
			{
				PID_A -> Out_PWM = 0;
			}


		// Kenh B
		PID_B -> Up =  PID_B -> Kp * Error_B -> ek;
		PID_B -> Ui = PID_B -> Ui_1 + PID_B -> Ki * Error_B -> ek_1 * Time_Interrupt + PID_B -> Ui_Antiwindup;
		PID_B -> Ud = PID_B -> Ud_1 * (Error_B -> ek - Error_B -> ek_1);
		PID_B -> Temp_PWM = PID_B -> Up + PID_B -> Ui + PID_B -> Ud;
		// Antiwindup
		PID_B -> Ui_Antiwindup = Anti_Windup(PID_B -> Temp_PWM, HIGH_Limit_PWM, LOW_Limit_PWM, Kb);
		PID_B -> Out_PWM = round(PID_B -> Temp_PWM);

			if(PID_B -> Out_PWM >= 999)
			{
				PID_B -> Out_PWM = 999;
			}
			else if (PID_B -> Out_PWM <=0 )
			{
				PID_B -> Out_PWM = 0;
			}

			// Gan lai gia tri
			Error_A -> ek_1 = Error_A -> ek;
			Error_B -> ek_1 = Error_B -> ek;
			PID_A -> Ui_1 = PID_A -> Ui;
			PID_B -> Ui_1 = PID_B -> Ui;
			PID_A -> Ud_1 = PID_A -> Ud;
			PID_B -> Ud_1 = PID_B -> Ud;

			Encoder_A_Pre = Encoder_A;
			Encoder_B_Pre = Encoder_B;


//			HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)Output_PWM_A, 1);
//			HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)Output_PWM_B, 1);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,PID_A -> Out_PWM);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,PID_B -> Out_PWM);



//			Kinematic_Matlab_A -> theta = atan2(Kinematic_Matlab_A -> x_dot_Real , Kinematic_Matlab_A -> y_dot_Real);
//			Kinematic_Matlab_A -> x1 = Kinematic_Matlab_A -> x + Kinematic_Matlab_A -> b * cos(Kinematic_Matlab_A -> theta);
//			Kinematic_Matlab_A -> y1 = Kinematic_Matlab_A -> y + Kinematic_Matlab_A -> b * sin(Kinematic_Matlab_A -> theta);
//			Kinematic_Matlab_A->w1 = ((Kinematic_Matlab_A->ay1*Kinematic_Matlab_A->x_dot) - (Kinematic_Matlab_A->ax1*Kinematic_Matlab_A->y1_dot))/(pow(Kinematic_Matlab_A->x_dot, 2)+pow(Kinematic_Matlab_A->y_dot, 2));
//			Kinematic_Matlab_A->x1_dot = Kinematic_Matlab_A->x_dot - Kinematic_Matlab_A->b*Kinematic_Matlab_A->w1*sin(Kinematic_Matlab_A->theta);
//			Kinematic_Matlab_A->y1_dot = Kinematic_Matlab_A->y_dot + Kinematic_Matlab_A->b*Kinematic_Matlab_A->w1*cos(Kinematic_Matlab_A->theta);

			// PID X,Y,Theta controller
			Error_X -> ek = (Kinematic_Matlab_A->x_QH_welding - Kinematic_Matlab_A->x_Real);
			Error_Y -> ek = (Kinematic_Matlab_A->y_QH_welding - Kinematic_Matlab_A->y_Real);
			// Toạ độ X
			PID_X -> Up =  PID_X -> Kp * Error_X -> ek;
			PID_X -> Ui = PID_X -> Ui_1 + PID_X -> Ki * Error_X -> ek_1 * Time_Interrupt + PID_X -> Ui_Antiwindup;
			PID_X -> Ud = PID_X -> Ud_1 * (Error_X -> ek - Error_X -> ek_1);
			// Antiwindup
			PID_X -> Ui_Antiwindup = Anti_Windup(PID_X -> Temp_PWM, HIGH_Limit_PWM, LOW_Limit_PWM, Kb);
			PID_X -> Temp_PWM = PID_X -> Up + PID_X -> Ui + PID_X -> Ud;
			PID_X -> Out_PWM = round(PID_X -> Temp_PWM);
			// Toạ độ Y
			PID_Y -> Up =  PID_Y -> Kp * Error_Y -> ek;
			PID_Y -> Ui = PID_Y -> Ui_1 + PID_Y -> Ki * Error_Y -> ek_1 * Time_Interrupt + PID_Y -> Ui_Antiwindup;
			PID_Y -> Ud = PID_Y -> Ud_1 * (Error_Y -> ek - Error_Y -> ek_1);
			// Antiwindup
			PID_Y -> Ui_Antiwindup = Anti_Windup(PID_Y -> Temp_PWM, HIGH_Limit_PWM, LOW_Limit_PWM, Kb);
			PID_Y -> Temp_PWM = PID_Y -> Up + PID_Y -> Ui + PID_Y -> Ud;
			PID_Y -> Out_PWM = round(PID_Y -> Temp_PWM);

			Inverse_Kinematic(RPM_A, RPM_B, Kinematic_Matlab_A, PID_X, PID_Y);
			Forward_Kinematic(RPM_A, RPM_B, Kinematic_Matlab_A);
			// Cập nhật giá trị mới
			Kinematic_Matlab_A->x_dot_Real = Kinematic_Matlab_A->vel_Real*cos(Kinematic_Matlab_A->theta);
			Kinematic_Matlab_A->y_dot_Real = Kinematic_Matlab_A->vel_Real*sin(Kinematic_Matlab_A->theta);
			// Cập nhật giá trị theo tích phân
			Kinematic_Matlab_A->x_Real = Kinematic_Matlab_A->x_Real + Kinematic_Matlab_A->x_dot_Real * Time_Interrupt;
			Kinematic_Matlab_A->y_Real = Kinematic_Matlab_A->y_Real  + Kinematic_Matlab_A->y_dot_Real * Time_Interrupt;
			// Cập nhật x_dot, y_dot
			Kinematic_Matlab_A->x_dot = Kinematic_Matlab_A->x_dot_Real;
			Kinematic_Matlab_A->y_dot = Kinematic_Matlab_A->y_dot_Real;


	}



//	//////////////////////////***************************///////////////////////////////
//	%th
//	th1 = atan2(ydot,xdot);
//
//	x1= x +b*cos(th1);
//	y1= y +b*sin(th1);
//
//	%w
//	w1 = (yddot*xdot- xddot*ydot)/(xdot^2+ydot^2) ;
//
//	%v
//	x1dot = xdot - b*w1*sin(th1);
//	y1dot = ydot + b*w1*cos(th1);
//	//////////////////////////***************************///////////////////////////////
	// Timer cho PID KInematic
//	else if (htim -> Instance == TIM9)
//	{
//		Kinematic_Matlab_A -> theta = atan2(Kinematic_Matlab_A -> y_dot , Kinematic_Matlab_A -> x_dot);
//		Kinematic_Matlab_A -> x1 = Kinematic_Matlab_A -> x + Kinematic_Matlab_A -> b * cos(Kinematic_Matlab_A -> theta);
//		Kinematic_Matlab_A -> y1 = Kinematic_Matlab_A -> y + Kinematic_Matlab_A -> b * sin(Kinematic_Matlab_A -> theta);
//		Kinematic_Matlab_A->w1 = ((Kinematic_Matlab_A->ay1*Kinematic_Matlab_A->x_dot) - (Kinematic_Matlab_A->ax1*Kinematic_Matlab_A->y1_dot))/(pow(Kinematic_Matlab_A->x_dot, 2)+pow(Kinematic_Matlab_A->y_dot, 2));
//		Kinematic_Matlab_A->x1_dot = Kinematic_Matlab_A->x_dot - Kinematic_Matlab_A->b*Kinematic_Matlab_A->w1*sin(Kinematic_Matlab_A->theta);
//		Kinematic_Matlab_A->y1_dot = Kinematic_Matlab_A->y_dot + Kinematic_Matlab_A->b*Kinematic_Matlab_A->w1*cos(Kinematic_Matlab_A->theta);
//
//
//		// PID X,Y,Theta controller
//		Error_X -> ek = (Kinematic_Matlab_A->x_QH_welding - Kinematic_Matlab_A->x_welding_Real);
//		Error_Y -> ek = (Kinematic_Matlab_A->y_QH_welding - Kinematic_Matlab_A->y_welding_Real);
//
//		// Toạ độ X
//		PID_X -> Up =  PID_X -> Kp * Error_X -> ek;
//		PID_X -> Ui = PID_X -> Ui_1 + PID_X -> Ki * Error_X -> ek_1 * Time_Interrupt + PID_X -> Ui_Antiwindup;
//		PID_X -> Ud = PID_X -> Ud_1 * (Error_X -> ek - Error_X -> ek_1);
//		// Antiwindup
//		PID_X -> Ui_Antiwindup = Anti_Windup(PID_X -> Temp_PWM, HIGH_Limit_PWM, LOW_Limit_PWM, Kb);
//		PID_X -> Temp_PWM = PID_X -> Up + PID_X -> Ui + PID_X -> Ud;
//		PID_X -> Out_PWM = round(PID_X -> Temp_PWM);
//		// Toạ độ Y
//		PID_Y -> Up =  PID_Y -> Kp * Error_Y -> ek;
//		PID_Y -> Ui = PID_Y -> Ui_1 + PID_Y -> Ki * Error_Y -> ek_1 * Time_Interrupt + PID_Y -> Ui_Antiwindup;
//		PID_Y -> Ud = PID_Y -> Ud_1 * (Error_Y -> ek - Error_Y -> ek_1);
//		// Antiwindup
//		PID_Y -> Ui_Antiwindup = Anti_Windup(PID_Y -> Temp_PWM, HIGH_Limit_PWM, LOW_Limit_PWM, Kb);
//		PID_Y -> Temp_PWM = PID_Y -> Up + PID_Y -> Ui + PID_Y -> Ud;
//		PID_Y -> Out_PWM = round(PID_Y -> Temp_PWM);
//
//		//�?ộng hoc nghịch
//		Inverse_Kinematic(SP_RPM_A, SP_RPM_B, Kinematic_Matlab_A, PID_X, PID_Y);
//		// �?ộng h�?c thuận để ra vị trí hiện tại
//		Forward_Kinematic(RPM_A, RPM_B, Kinematic_Matlab_A);
//		// Cập nhật giá trị mới
//		Kinematic_Matlab_A->x_dot_Real = Kinematic_Matlab_A->vel_Real*cos(Kinematic_Matlab_A->theta);
//		Kinematic_Matlab_A->y_dot_Real = Kinematic_Matlab_A->vel_Real*sin(Kinematic_Matlab_A->theta);
//		// Cập nhật giá trị theo tích phân
//		Kinematic_Matlab_A->x_Real = Kinematic_Matlab_A->x_dot_Real * Time_Interrupt;
//		Kinematic_Matlab_A->y_Real = Kinematic_Matlab_A->y_dot_Real * Time_Interrupt;
//		// Cập nhật giá trị đầu hàn
//		Kinematic_Matlab_A->x_welding_Real = Kinematic_Matlab_A->x_Real + Kinematic_Matlab_A->b*cos(Kinematic_Matlab_A->theta);
//		Kinematic_Matlab_A->y_welding_Real = Kinematic_Matlab_A->y_Real + Kinematic_Matlab_A->b*sin(Kinematic_Matlab_A->theta);
	//}
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
