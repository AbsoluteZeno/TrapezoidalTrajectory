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
#include "Encoder.h"
#include "TrapezoidalTrajectory.h"
#include "Controller.h"
#include "LinearDrive.h"
#include "ModBusRTU.h"
#include "joyStick.h"
#include "holePositionsCartesian.h"
#include "EndEffector.h"
#include "Effstatus.h"
#include "BaseSystemStateMachine.h"

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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
//Parameters =================================================================================
// Trajectory --------------------------------------------------------------------------------
uint64_t t_traj = 0;	  // [us] 		Time use in trajectory function
uint64_t t_total_actual = 0;
float q_des;			  // [mm] 		Desire position calculated from trajectory
float qdot_des;			  // [mm/s] 	Desire velocity calculated from trajectory
float qddot_des;		  // [mm/s^2] 	Desire acceleration calculated from trajectory
Traj  traj;
float Pi = 0;		  	  // [mm]
float Pf = 0;			  // [mm]
float Pf_last = 0;
uint8_t SteadyStateFlag = 0;
//// Time -----------------------------------------------------------------------------------
uint64_t _micros = 0;
// QEI --------------------------------------------------------------------------------------
QEIStructureTypeDef QEIData = {0};
// PID --------------------------------------------------------------------------------------
float PulseWidthModulation = 0;
uint8_t ControllerFinishedFollowFlag = 0;
PID Controller;
// Stroke Safety ----------------------------------------------------------------------------
uint8_t P_disallow = 0;
uint8_t N_disallow = 0;
// Set Home ---------------------------------------------------------------------------------
uint8_t SetHomeYFlag = 1;
uint8_t CenterFlag = 0;
// Elec -------------------------------------------------------------------------------------
//EndEffector
uint8_t 	softReset_cmd[] = {0x00,0xFF,0x55,0xAA};
uint8_t		emerMode_cmd[]  = {0xF1,0x99,0x99,0x99};
uint8_t		exitEmer_cmd[]  = {0xE5,0x7A,0xFF,0x81};
uint8_t 	testMode_cmd[]  = {0x01,0x10,0x99,0x99};
uint8_t 	exitTest_cmd[]  = {0x01,0x00,0x99,0x99};
uint8_t		runMode_cmd[]   = {0x10,0x13,0x99,0x99};
uint8_t		exitRun_cmd[]   = {0x10,0x8C,0x99,0x99};
uint8_t		pickup_cmd[]    = {0x10,0x5A,0x99,0x99};
uint8_t		place_cmd[]     = {0x10,0x69,0x99,0x99};
uint8_t 	AllOff_cmd[]	= {0x01,0x00,0x99,0x99,0x10,0x8C,0x99,0x99};

uint8_t 	effst[1];
uint8_t     readFlag 					= 0;
uint8_t 	writeflag_ls 				= 0;
uint8_t 	effstatus;
uint8_t 	effstatus_temp;
uint8_t 	effreg_temp;

uint16_t 	EffRegState					= 0;

uint8_t 	EndEffectorSoftResetFlag 	= 1;
uint8_t		EffAllOff_Flag				= 0;
uint8_t		EffLaserOn_Flag				= 0;
uint8_t		EffGripperOn_Flag			= 0;
uint8_t		EffGripperPick_Flag			= 0;
uint8_t		EffGripperPlace_Flag		= 0;
uint8_t 	eff_action 					= 0;
uint16_t 	eff_l 						= 0;
uint16_t 	eff_c 						= 0;
uint16_t 	effst_mb 					= 0;
// Emergency Switch
uint8_t emer_pushed = 1;
// Photoelectric sensor
uint8_t 	pe1_st;
uint8_t 	pe2_st;					//Photoelectric Sensor Value
uint8_t 	pe3_st;
//joy stick----------------------------------------
uint16_t adcRawData[20];
uint32_t IN0[10]; //Y
uint32_t IN1[10]; //X
uint8_t JoyStickSwitch = 0;
uint32_t X_axis, Y_axis;
uint32_t joystickXaxis, joystickYaxis;
float DummyA;
float DummyB;
//Overshoot Testing--------------------------------
float  max_pos = 0;
float  overshoot = 0;
//nine holes of tray-------------------------------
float32_t Pickreference[2] = {0, 0};
float32_t Pickopposite[2] = {0, 0};
float32_t PickrotationAngleRadian = 0;
float32_t PickrotationAngleDegree = 0;
float32_t PickTray9holes[9][2];
float32_t Placereference[2] = {0, 0};
float32_t Placeopposite[2] = {0, 0};
float32_t PlacerotationAngleRadian = 0;
float32_t PlacerotationAngleDegree = 0;
float32_t PlaceTray9holes[9][2];
uint8_t GoalReadyFlag = 0;
// Modbus -----------------------------------------------------------------------------------
ModbusHandleTypedef hmodbus;
u16u8_t registerFrame[70];
// Base System ------------------------------------------------------------------------------
uint8_t SetPickTrayFlag = 0;
uint8_t SetPlaceTrayFlag = 0;
uint8_t SetHomeFlag = 0;
uint8_t RunTrayFlag = 0;
uint8_t RunPointFlag = 0;
//===========================================================================================
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM11_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */	HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM11_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  //eff_write(testMode_cmd);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  Controller.Kp = 1100;
  Controller.Ki = 9.25;
  Controller.Kd = 0;

  hmodbus.huart = &huart2;
  hmodbus.htim = &htim11;
  hmodbus.slaveAddress = 0x15;
  hmodbus.RegisterSize =70;
  Modbus_init(&hmodbus, registerFrame);

  //joy stick--------------------------
  HAL_ADC_Start_DMA(&hadc1, adcRawData, 20);
  //-----------------------------------

//	PickTray9holes[1] = 300;
//	PlaceTray9holes[1] = -300;
//	PickTray9holes[3] = 300;
//	PlaceTray9holes[3] = -300;
//	PickTray9holes[5] = 200;
//	PlaceTray9holes[5] = -200;
//	PickTray9holes[7] = 200;
//	PlaceTray9holes[7] = -200;
//	PickTray9holes[9] = 100;
//	PlaceTray9holes[9] = -100;
//	PickTray9holes[11] = 100;
//	PlaceTray9holes[11] = -100;
//	PickTray9holes[13] = 100;
//	PlaceTray9holes[13] = -100;
//	PickTray9holes[15] = 200;
//	PlaceTray9holes[15] = -300;
//	PickTray9holes[17] = 300;
//	PlaceTray9holes[17] = -50;

//  	PickTray9holes[1] = 300;
//  	PlaceTray9holes[1] = -300;
//  	PickTray9holes[3] = 300;
//  	PlaceTray9holes[3] = -300;
//  	PickTray9holes[5] = 300;
//  	PlaceTray9holes[5] = -300;
//  	PickTray9holes[7] = 300;
//  	PlaceTray9holes[7] = -300;
//  	PickTray9holes[9] = 300;
//  	PlaceTray9holes[9] = -300;
//  	PickTray9holes[11] = 300;
//  	PlaceTray9holes[11] = -300;
//  	PickTray9holes[13] = 300;
//  	PlaceTray9holes[13] = -300;
//  	PickTray9holes[15] = 300;
//  	PlaceTray9holes[15] = -300;
//  	PickTray9holes[17] = 300;
//  	PlaceTray9holes[17] = -300;

//  Pickreference[0] = 100;
//  Pickreference[1] = 200;
//  PickrotationAngleRadian = 0.785;
//
//  Placereference[0] = -100;
//  Placereference[1] = -300;
//  PlacerotationAngleRadian = 2.845;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */
//#define break while(1){}
  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 49999;
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
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 99;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 2005;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
  sConfigOC.Pulse = 1433;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
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
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA11 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim5)
	{
//		if(writeflag_ls == 1){
//			eff_write(softReset_cmd);
//			writeflag_ls = 0;
//		}
//		if(writeflag_ls == 2){
//			eff_write(testMode_cmd);
//			writeflag_ls = 0;
//		}
//		if(writeflag_ls == 3){
//			eff_write(exitTest_cmd);
//			writeflag_ls = 0;
//		}
//		if(writeflag_ls == 4){
//			eff_write(runMode_cmd);
//			writeflag_ls = 0;
//		}
//		if(writeflag_ls == 5){
//			eff_write(exitRun_cmd);
//			writeflag_ls = 0;
//		}
//		if(writeflag_ls == 6){
//			eff_write(pickup_cmd);
//			writeflag_ls = 0;
//		}
//		if(writeflag_ls == 7){
//			eff_write(place_cmd);
//			writeflag_ls = 0;
//		}
//		if(writeflag_ls == 8){
//			eff_read();
//			writeflag_ls = 0;
//		}
//		if(writeflag_ls == 9){
//			eff_st();
//			writeflag_ls = 0;
//		}

		_micros += 1000;
		QEIEncoderPositionVelocity_Update(&htim3, &htim5);

		check_pe();
		SetHome(&htim3, &htim1);

		if (EndEffectorSoftResetFlag)
		{
			eff_write(softReset_cmd);
			EndEffectorSoftResetFlag = 0;
		}

		eff_c = registerFrame[2].U16;
		if(eff_l != eff_c){
			eff_action = 1;
		}

		switch (registerFrame[1].U16)
		{
		case 0b00001:
			SetPickTrayFlag = 1;
		break;
		case 0b00010:
			SetPlaceTrayFlag = 1;
		break;
		case 0b00100:
			SetHomeFlag = 1;
		break;
		case 0b01000:
			RunTrayFlag = 1;
		break;
		case 0b10000:
			RunPointFlag = 1;
		break;
		}

		EffRegState = registerFrame[2].U16;

		switch(EffRegState){
		case 0b0000:	//everything off
			if(eff_action == 1){
				EffAllOff_Flag = 1;
				eff_action = 0;
			}
			break;
		case 0b0001:	//laser on
			if(eff_action == 1){
				EffLaserOn_Flag = 1;
				eff_action = 0;
			}
			break;
		case 0b0010:	//gripper on
			if(eff_action == 1){
				EffGripperOn_Flag = 1;
				eff_action = 0;
			}
			break;
		case 0b0110:	//gripper picking
			if(eff_action == 1){
				EffGripperPick_Flag = 1;
				eff_action = 0;
			}
			break;
		case 0b1010:	//gripper placing
			if(eff_action == 1){
				EffGripperPlace_Flag = 1;
				eff_action = 0;
			}
			break;
		}

		if (!(SetPickTrayFlag || SetPlaceTrayFlag || SetHomeFlag || RunTrayFlag || RunPointFlag || SetHomeYFlag))
		{
			GetJoystickXYaxisValue(&DummyA, &DummyB);
			JoyStickControlCartesian();
		}

		if (emer_pushed)
		{
			BaseSystem_SetPickTray();
			BaseSystem_SetPlaceTray();
			BaseSystem_SetHome();
			BaseSystem_RuntrayMode();
			BaseSystem_RunPointMode();

			BaseSystem_EffAllOff();
			BaseSystem_EffLaserOn();
			BaseSystem_EffGripperOn();
			BaseSystem_EffGripperPick();
			BaseSystem_EffGripperPlace();
		}

		static uint8_t j = 0;
		if (j == 0)
		{
			eff_st();
		}
		j = (j + 1) % 500;

		eff_l = eff_c;

		static uint8_t i = 0;
		if (i == 0)
		{
			registerFrame[0].U16 = 0b0101100101100001; //Ya 22881
			Modbus_Protocal_Worker();
			//eff_st();
		}
		i = (i + 1) % 200;

	}
}

void ControllerState()
{
	static enum {Idle, Follow} state = Idle;

	if (SetHomeYFlag == 0)
	{
		switch(state)
		{
		case Idle:
			ControllerFinishedFollowFlag = 1;
			PulseWidthModulation = 0;
			MotorDrive(&htim1);
			Pi = QEIData.position;

			if(Pf != Pf_last)
			{
				t_traj = 0;
				SteadyStateFlag = 0;
				QuinticTraj_PreCal(Pi, Pf, &traj);
				ControllerFinishedFollowFlag = 0;
				state = Follow;
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
			}
		break;

		case Follow:
			t_traj = t_traj + 1000;
			if (t_traj <= traj.t_total * 1000000)
			{
				QuinticTraj_GetState(Pi, Pf, &traj, t_traj);
			}
			else
			{
				q_des = Pf;
			}

			PositionControlVelocityForm(&Controller);
			MotorDrive(&htim1);

			if(QEIData.position > max_pos)
			{
				max_pos = QEIData.position;
			}

			if (((t_traj > traj.t_total * 1000000) && (0.15 > fabs(q_des - QEIData.position)) && (SteadyStateFlag == 0)) || P_disallow || N_disallow)
			{
				t_total_actual = t_traj + 500000;
				SteadyStateFlag = 1;
			}

			if (SteadyStateFlag && (t_traj > t_total_actual) && (0.05 > fabs(q_des - QEIData.position)) || (P_disallow) || (N_disallow))
			{
				state = Idle;
				overshoot = max_pos * 100/(Pf - Pi);
				max_pos = 0;
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
			}
		break;
		}
		Pf_last = Pf;
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
