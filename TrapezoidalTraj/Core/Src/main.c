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
#include "arm_math.h"
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
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//Parameters ==============================================
// Trajectory ---------------------------------------------
uint64_t t_traj = 0;	  // [us] 		Time use in trajectory function
float q_des;			  // [mm] 		Desire position calculated from trajectory
float qdot_des;			  // [mm/s] 	Desire velocity calculated from trajectory
float qddot_des;		  // [mm/s^2] 	Desire acceleration calculated from trajectory
float t_total;			  // [s]
float t_acc;			  // [s]
float Pi = 0;		  	  // [mm]
float Pf = 0;			  // [mm]
float Pf_last = 0;
// Time ---------------------------------------------------
uint64_t _micros = 0;
// QEI ----------------------------------------------------
typedef struct _QEIStructure
{
	int32_t data[2];
	uint32_t timestamp[2];
	float position;	// mm
	float velocity;	// mm/s
}QEIStructureTypeDef;
QEIStructureTypeDef QEIData = {0};
// PID ----------------------------------------------------
float PulseWidthModulation = 0;
float setposition = 0;
float first_error = 0;
float second_error = 0;
float third_error = 0;
float kp_position = 120;
float ki_position = 0.05;
float kd_position = 0;
// Safety -------------------------------------------------
uint8_t P_disallow = 0;
uint8_t N_disallow = 0;
// Set Home -----------------------------------------------
uint8_t SetHomeFlag = 1;
// Elec ---------------------------------------------------
// Emergency Switch
uint8_t emer_pushed = 1;
// Photoelectric sensor
uint8_t 	pe1_st;
uint8_t 	pe2_st;					//Photoelectric Sensor Value
uint8_t 	pe3_st;
//Variables ===============================================
float freq = 10;		  // Hz
float v_max = 1000.0;	  	  // mm/s
float a = 2000.0;		  	  // mm/s^2
uint16_t res = 8192;      // Resolution [pulse/revolution]
float pulley_dia = 30.558;// mm
//=========================================================
uint32_t Micro = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
inline uint64_t micros();

void QEIEncoderPositionVelocity_Update();

void SetHome();

void TrapezoidalTraj_PreCal(int16_t start_pos, int16_t final_pos);
void TrapezoidalTraj_GetState(int16_t start_pos, int16_t final_pos, uint32_t t_us);

void PositionControlVelocityForm();
void MotorDrive();

void ControllerState();

void check_pe();
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
  MX_USART2_UART_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1 || TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  TrapezoidalTraj_PreCal(0, 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  Micro = micros();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = 5000;
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
  htim3.Init.Period = QEI_PERIOD - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
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
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 99999;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA11 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim5)
	{
		_micros += 1000;

		QEIEncoderPositionVelocity_Update();
		check_pe();
		SetHome();
		ControllerState();
	}
}

uint64_t micros()
{
	return __HAL_TIM_GET_COUNTER(&htim5)*0.01 + _micros;
}


void QEIEncoderPositionVelocity_Update()
{
	QEIData.timestamp[0] = micros();
	uint32_t lastposition = __HAL_TIM_GET_COUNTER(&htim3);
	QEIData.data[0] = lastposition;
	if (lastposition > ((QEI_PERIOD/2) - 1))
	{
		QEIData.data[0] = lastposition - QEI_PERIOD;
	}

	// position calculation
	QEIData.position = QEIData.data[0] * PI *  pulley_dia/res;

	int32_t diffPosition = QEIData.data[0] - QEIData.data[1];
	float diffTime = QEIData.timestamp[0] - QEIData.timestamp[1];

	// unwrap
	if (diffPosition > QEI_PERIOD>>1) diffPosition -= QEI_PERIOD;
	if (diffPosition < -(QEI_PERIOD>>1)) diffPosition += QEI_PERIOD;

	// velocity calculation
	QEIData.velocity = (diffPosition * 1000000.0 * PI * pulley_dia)/(res * diffTime);

	QEIData.data[1] = QEIData.data[0];
	QEIData.timestamp[1] = QEIData.timestamp[0];
}

void TrapezoidalTraj_PreCal(int16_t start_pos, int16_t final_pos)
{
	if (start_pos != final_pos)
	{
		float s = final_pos - start_pos;

		t_acc = v_max/a;
		t_total = (pow(v_max,2)+a*fabs(s))/(a*v_max);
	}
}

void TrapezoidalTraj_GetState(int16_t start_pos, int16_t final_pos, uint32_t t_us)
{
	if (start_pos != final_pos)
	{
		float t = t_us/1000000.0;

		float s = final_pos - start_pos;
		int8_t dir = 1;
		if (s < 0)
		{
			dir = -1;
		}

		if (2*t_acc < t_total) // General Case
		{
			if (t <= t_acc)
			{
				qddot_des = dir*a;
				qdot_des = dir*a*t;
				q_des = start_pos + dir*(0.5*a*pow(t,2));
			}
			else if (t_acc < t && t < (t_total - t_acc))
			{
				qddot_des = 0;
				qdot_des = dir*a*t_acc;
				q_des = start_pos + dir*(0.5*a*pow(t_acc,2) + a*t_acc*(t - t_acc));
			}
			else if ((t_total - t_acc) <= t && t <= t_total)
			{
				qddot_des = -dir*a;
				qdot_des = dir*a*(t_total - t);
				q_des = start_pos + dir*(a*t_total*t+a*t_acc*t_total-a*pow(t_acc,2)-0.5*a*(pow(t,2)+pow(t_total,2)));
			}
		}
		else	// Triangle Case
		{
			t_acc = 0.5*sqrt(4*fabs(s)/a);
			t_total = 2*t_acc;

			if (t <= t_acc)
			{
				qddot_des = dir*a;
				qdot_des = dir*a*t;
				q_des = start_pos + dir*(0.5*a*pow(t,2));
			}
			else if (t_acc < t && t < t_total)
			{
				qddot_des = -dir*a;
				qdot_des = dir*a*(2*t_acc - t);
				q_des = start_pos + dir*(2*a*t_acc*t-0.5*a*pow(t,2)-a*pow(t_acc,2));
			}
		}
	}
}

void PositionControlVelocityForm()
{
	first_error = q_des - QEIData.position;

	PulseWidthModulation += ((kp_position + ki_position + kd_position) * first_error) - ((kp_position + (2 * kd_position)) * second_error) + (kd_position * third_error);

	third_error = second_error;

	second_error = first_error;
}

void MotorDrive()
{
	if(emer_pushed == 1){
		if (PulseWidthModulation >= 0)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
			N_disallow = 0;
			if (PulseWidthModulation > 8000)
			{
				PulseWidthModulation = 8000;
			}

			if (pe2_st || P_disallow)
			{
				PulseWidthModulation = 0;
				P_disallow = 1;
			}
		}
		else
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
			P_disallow = 0;
			if (PulseWidthModulation < -8000)
			{
				PulseWidthModulation = -8000;
			}

			if (pe3_st || P_disallow)
			{
				PulseWidthModulation = 0;
				N_disallow = 1;
			}
		}

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, fabs(PulseWidthModulation));
	}
}

void ControllerState()
{
	static enum {Idle, Follow} state = Idle;

	if (SetHomeFlag == 0)
	{
		switch(state)
		{
		case Idle:
			PulseWidthModulation = 0;
			MotorDrive();
			Pi = QEIData.position;

			if(Pf != Pf_last)
			{
				t_traj = 0;
				TrapezoidalTraj_PreCal(Pi, Pf);
				state = Follow;
			}
		break;

		case Follow:
			t_traj = t_traj + 1000;
			if (t_traj <= t_total * 1000000)
			{
				TrapezoidalTraj_GetState(Pi, Pf, t_traj);
			}
			else
			{
				q_des = Pf;
			}

			PositionControlVelocityForm();
			MotorDrive();

			if (((t_traj > t_total * 1000000) && (0.15 > fabs(q_des - QEIData.position))) || P_disallow || P_disallow)
			{
				state = Idle;
			}
		break;
		}
		Pf_last = Pf;
	}
}

void SetHome()
{
	static enum {Jog, Overcenter, Center, Recenter} SetHomeState = Jog;

	if (SetHomeFlag)
	{
		switch (SetHomeState)
		{
		case Jog:
			PulseWidthModulation = 3000;

			if (pe1_st)
			{
				__HAL_TIM_SET_COUNTER(&htim3, 0);
				SetHomeState = Overcenter;
			}
			else if (pe2_st)
			{
				__HAL_TIM_SET_COUNTER(&htim3, 0);
				SetHomeState = Recenter;
			}
			break;
		case Overcenter:
			PulseWidthModulation = 3000;

			if (QEIData.position >= 20)
			{
				SetHomeState = Center;
			}
			break;
		case Center:
			PulseWidthModulation = -1000;

			if (pe1_st)
			{
				PulseWidthModulation = 0;
				MotorDrive();
				__HAL_TIM_SET_COUNTER(&htim3, 0);
				SetHomeFlag = 0;
				SetHomeState = Jog;
			}
			break;
		case Recenter:
			PulseWidthModulation = -3000;

			if (QEIData.position <= -330)
			{
				SetHomeState = Center;
			}
			break;
		}
		MotorDrive();
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_12 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == 0)
	{
		emer_pushed = 0;
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
	}
	if(GPIO_Pin == GPIO_PIN_12 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == 1)
	{
		emer_pushed = 1;
	}
}

void check_pe(){
	// Photoelectric Sensor
	if(emer_pushed == 1){
		pe1_st = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
		pe2_st = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
		pe3_st = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
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
