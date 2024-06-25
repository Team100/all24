/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct Quaternion {
	float qi;
	float qj;
	float qk;
	float qr;
	float acc;
} Quaternion_t;

typedef struct EulerAng {
	float roll;
	float pitch;
	float yaw;
	float acc;
} EulerAng_t;

typedef struct ARVRStabRotVectReport {
	uint8_t reportID;
	uint8_t repSeqNum;
	uint8_t repStatus;
	uint8_t repDelay;
	int16_t quat_i;
	int16_t quat_j;
	int16_t quat_k;
	int16_t quat_r;
	int16_t accEstim;
} ARVRStabRotVectReport_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SCALE_Q(n) (1.0f / (1 << n))


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char outputString[500];
HAL_StatusTypeDef transmissionStatus;

uint8_t txDataI2C[500];
uint8_t rxDataI2C[500];
uint8_t txProdIDReqI2C[6] = {0x06,0x00,0x02,0x00,0xF9,0x00};
uint8_t txMisteryReqI2C[5] = {0x05,0x00,0x01,0x00,0x01};
uint8_t txSetFeatureI2C[21] = {0x15,0x00,0x02,0x01,0xFD,0x28,0x00,0x00,0x00,0x88,0x13,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

uint8_t tgtAddr;
uint8_t bytes2Read;

GPIO_PinState button_state;
uint8_t execute_btn_actions;

Quaternion_t myQuat;
EulerAng_t myRollPitchYaw, myRPY, myRPYScaled;
ARVRStabRotVectReport_t myARVRReport;

float norm;

float sqi;
float sqj;
float sqk;
float sqr;

float sinr_cosp;
float cosr_cosp;
float sinp;
float cosp;
float siny_cosp;
float cosy_cosp;

// Private variables for CAN
CAN_FilterTypeDef canFltrCfg;
CAN_TxHeaderTypeDef canTxHeader;
//CAN_RxHeaderTypeDef canRxHeader;
uint32_t canMailbox;
uint8_t canTxMsg[8];

uint16_t tmp_uint16;

uint32_t tickStartCycle, tickEndCycle, tickLastStartCycle;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

uint8_t getARVRQuat(const ARVRStabRotVectReport_t* inputRep, Quaternion_t* quatStruct);
void canConfig(void);
void ClearCANPendingMessage(CAN_HandleTypeDef *pCANHandle, CAN_TxHeaderTypeDef *pHeader, uint8_t *pTxData, uint32_t *pTxMailbox);
void TransmitCANMessage(CAN_HandleTypeDef *pCANHandle, CAN_TxHeaderTypeDef *pHeader, uint8_t *pTxData, uint32_t *pTxMailbox);


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
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  sprintf(outputString, "Application is starting...\r\n");
  transmissionStatus = HAL_UART_Transmit(&huart2, (uint8_t*)outputString, strlen(outputString), HAL_MAX_DELAY);

  // print SYSCLK Speed
  sprintf(outputString, "SYSCLK Speed: %ld\r\n", HAL_RCC_GetSysClockFreq());
  transmissionStatus = HAL_UART_Transmit(&huart2, (uint8_t*)outputString, strlen(outputString), HAL_MAX_DELAY);
  // print HCLK Speed
  sprintf(outputString, "HCLK Speed: %ld\r\n", HAL_RCC_GetHCLKFreq());
  transmissionStatus = HAL_UART_Transmit(&huart2, (uint8_t*)outputString, strlen(outputString), HAL_MAX_DELAY);
  // print PCLK1 Speed
  sprintf(outputString, "PCLK1 Speed: %ld\r\n", HAL_RCC_GetPCLK1Freq());
  transmissionStatus = HAL_UART_Transmit(&huart2, (uint8_t*)outputString, strlen(outputString), HAL_MAX_DELAY);
  // print PCLK2 Speed
  sprintf(outputString, "PCLK2 Speed: %ld\r\n", HAL_RCC_GetPCLK2Freq());
  transmissionStatus = HAL_UART_Transmit(&huart2, (uint8_t*)outputString, strlen(outputString), HAL_MAX_DELAY);

  // Configure CAN structures and state machine
  canConfig();

  //Default address
  tgtAddr = 0x4A << 1;

  //Alternate address
//  tgtAddrRed = 0x4B << 1;

	bytes2Read = 100;

	// first I2C transaction
	HAL_I2C_Master_Transmit(&hi2c1, tgtAddr, txMisteryReqI2C, 5, HAL_MAX_DELAY);
	sprintf(outputString, "BNO085 write: %x\r\n", txMisteryReqI2C);
	transmissionStatus = HAL_UART_Transmit(&huart2, (uint8_t*)outputString, strlen(outputString), HAL_MAX_DELAY);

	HAL_Delay(2);

	HAL_I2C_Master_Receive(&hi2c1, tgtAddr, rxDataI2C, bytes2Read, HAL_MAX_DELAY);
	sprintf(outputString, "BNO085 read: ");
	for (int i = 0; i < bytes2Read; i++)
	{
		sprintf(outputString + strlen(outputString), "%02x ", rxDataI2C[i]);
	}
    sprintf(outputString + strlen(outputString), "\r\n");
	transmissionStatus = HAL_UART_Transmit(&huart2, (uint8_t*)outputString, strlen(outputString), HAL_MAX_DELAY);

	HAL_Delay(10);

	// second I2C transaction
	HAL_I2C_Master_Transmit(&hi2c1, tgtAddr, txProdIDReqI2C, 6, HAL_MAX_DELAY);
	sprintf(outputString, "BNO085 write: %x\r\n", txProdIDReqI2C);
	transmissionStatus = HAL_UART_Transmit(&huart2, (uint8_t*)outputString, strlen(outputString), HAL_MAX_DELAY);

	HAL_Delay(2);

	HAL_I2C_Master_Receive(&hi2c1, tgtAddr, rxDataI2C, bytes2Read, HAL_MAX_DELAY);
	sprintf(outputString, "BNO085 read: ");
	for (int i = 0; i < bytes2Read; i++)
	{
		sprintf(outputString + strlen(outputString), "%02x ", rxDataI2C[i]);
	}
    sprintf(outputString + strlen(outputString), "\r\n");
	transmissionStatus = HAL_UART_Transmit(&huart2, (uint8_t*)outputString, strlen(outputString), HAL_MAX_DELAY);

	HAL_Delay(10);

	// third I2C transaction
	HAL_I2C_Master_Transmit(&hi2c1, tgtAddr, txSetFeatureI2C, 21, HAL_MAX_DELAY);
	sprintf(outputString, "BNO085 write: %x\r\n", txSetFeatureI2C);
	transmissionStatus = HAL_UART_Transmit(&huart2, (uint8_t*)outputString, strlen(outputString), HAL_MAX_DELAY);

	HAL_Delay(2);

	HAL_I2C_Master_Receive(&hi2c1, tgtAddr, rxDataI2C, bytes2Read, HAL_MAX_DELAY);
	sprintf(outputString, "BNO085 read: ");
	for (int i = 0; i < bytes2Read; i++)
	{
		sprintf(outputString + strlen(outputString), "%02x ", rxDataI2C[i]);
	}
    sprintf(outputString + strlen(outputString), "\r\n");
	transmissionStatus = HAL_UART_Transmit(&huart2, (uint8_t*)outputString, strlen(outputString), HAL_MAX_DELAY);

	HAL_Delay(10);

	bytes2Read = 80;

	// start timer
	HAL_TIM_Base_Start(&htim6);
	//HAL_TIM_Base_Start_IT(&htim6);
	tickStartCycle = HAL_GetTick();
	tickLastStartCycle = tickStartCycle;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
	  //wait for the time event
	  while(!(TIM6->SR & TIM_SR_UIF));
	  // Reset timer 6 status register
	  TIM6->SR = 0;
	  tickLastStartCycle = tickStartCycle;
	  tickStartCycle = HAL_GetTick();

	  // Turn LED on
	  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);

	  // Toggle LED
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);

	  // Primary sensor readout
		rxDataI2C[0] = 0;
		HAL_I2C_Master_Receive(&hi2c1, tgtAddr, rxDataI2C, bytes2Read, HAL_MAX_DELAY);

		// Confirm there is a response (if not, reconfigure BNO085)
		if (rxDataI2C[9] != 0x28)
		{
			// Reset the feature request
			HAL_I2C_Master_Transmit(&hi2c1, tgtAddr, txSetFeatureI2C, 21, HAL_MAX_DELAY);
			/*
			sprintf(outputString, "Primary   BNO085 write: %x\r\n", txSetFeatureI2C);
			transmissionStatus = HAL_UART_Transmit(&huart2, (uint8_t*)outputString, strlen(outputString), HAL_MAX_DELAY);
			*/

			HAL_Delay(1);
		}
		else
		{
			// Extract quaternion from ARVR stabilized rotation vector report
			getARVRQuat(&rxDataI2C[9], &myQuat);

			// Compute quaternion norm (to verify quaternion unity)
			norm = myQuat.qi*myQuat.qi+myQuat.qj*myQuat.qj+myQuat.qk*myQuat.qk+myQuat.qr*myQuat.qr;

			// convert quaternion to Euler angles
			sqi = (myQuat.qi*myQuat.qi);
			sqj = (myQuat.qj*myQuat.qj);
			sqk = (myQuat.qk*myQuat.qk);
			sqr = (myQuat.qr*myQuat.qr);

			myRollPitchYaw.roll = atan2(2.0 * (myQuat.qj * myQuat.qk + myQuat.qi * myQuat.qr), (-sqi - sqj + sqk + sqr));
			myRollPitchYaw.pitch = asin(-2.0 * (myQuat.qi * myQuat.qk - myQuat.qj * myQuat.qr) / (sqi + sqj + sqk + sqr));
			myRollPitchYaw.yaw = atan2(2.0 * (myQuat.qi * myQuat.qj + myQuat.qk * myQuat.qr), (sqi - sqj - sqk + sqr));

			// scale to degrees
			myRollPitchYaw.roll *= 180.0/3.14159265;
			myRollPitchYaw.pitch *= 180.0/3.14159265;
			myRollPitchYaw.yaw *= 180.0/3.14159265;

			myRPY.roll = myRollPitchYaw.roll;
			myRPY.pitch = myRollPitchYaw.pitch;
			myRPY.yaw = myRollPitchYaw.yaw;

			// copy over accuracy estimate
			myRPY.acc =  myQuat.acc;

			// convert to 16-bit values for transmission on CAN bus
			// CAN scale factor -> 400deg maps to 65536 count
			// need to offset by 200 deg as data is +/-180 degs
			myRPYScaled.roll = myRPY.roll + 200;
			myRPYScaled.pitch = myRPY.pitch + 200;
			myRPYScaled.yaw = myRPY.yaw + 200;
			myRPYScaled.roll *= 163.84;
			myRPYScaled.pitch *= 163.84;
			myRPYScaled.yaw *= 163.84;
			myRPYScaled.acc = myRPY.acc * 655.36;

			// Output to CAN bus
			// First clear any pending messages
			ClearCANPendingMessage(&hcan, &canTxHeader, canTxMsg, &canMailbox);
			// reformat Roll Pitch Yaw from floats to 16 bit scaled values
			tmp_uint16 = (uint16_t) myRPYScaled.roll;
			canTxMsg[0] = (uint8_t) (tmp_uint16 >> 8);
			canTxMsg[1] = (uint8_t) (tmp_uint16 & 0xff);
			tmp_uint16 = (uint16_t) myRPYScaled.pitch;
			canTxMsg[2] = (uint8_t) (tmp_uint16 >> 8);
			canTxMsg[3] = (uint8_t) (tmp_uint16 & 0xff);
			tmp_uint16 = (uint16_t) myRPYScaled.yaw;
			canTxMsg[4] = (uint8_t) (tmp_uint16 >> 8);
			canTxMsg[5] = (uint8_t) (tmp_uint16 & 0xff);
			tmp_uint16 = (uint16_t) myRPYScaled.acc;
			canTxMsg[6] = (uint8_t) (tmp_uint16 >> 8);
			canTxMsg[7] = (uint8_t) (tmp_uint16 & 0xff);
			canTxHeader.ExtId = 0x4080440;

			TransmitCANMessage(&hcan, &canTxHeader, canTxMsg, &canMailbox);
			//HAL_Delay(1);
		}



		/*
	// Verbose output
	sprintf(outputString, "Pri BNO085 Euler angles: %f, %f, %f; Red BNO085 Euler angles: %f, %f, %f\r\n",
			myRPY.roll, myRPY.pitch, myRPY.yaw, myRPYRed.roll, myRPYRed.pitch, myRPYRed.yaw);
	transmissionStatus = HAL_UART_Transmit(&huart2, (uint8_t*)outputString, strlen(outputString), HAL_MAX_DELAY);
	*/
		tickEndCycle = HAL_GetTick();

	// Numbers only output
	sprintf(outputString, "%f, %f, %f, %f, %f, %f, %f, %f, %i, %i\r\n", myQuat.qi, myQuat.qj, myQuat.qk, myQuat.qr,
			myRPY.roll, myRPY.pitch, myRPY.yaw, myRPY.acc, (tickEndCycle-tickStartCycle), (tickStartCycle-tickLastStartCycle));
	transmissionStatus = HAL_UART_Transmit(&huart2, (uint8_t*)outputString, strlen(outputString), HAL_MAX_DELAY);

	  // Turn LED off
	  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 8;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  SET_BIT(hcan.Instance->MCR, CAN_MCR_NART);

  /* USER CODE END CAN_Init 2 */

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
  hi2c1.Init.Timing = 0x00300F38;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 32;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 19000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  //htim6.Init.Prescaler = 32;
  //htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  //htim6.Init.Period = 38000;
  //if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  //{
  //  Error_Handler();
  //}

  /* USER CODE END TIM6_Init 2 */

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
  huart1.Init.BaudRate = 921600;
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint8_t getARVRQuat(const ARVRStabRotVectReport_t* inputRep, Quaternion_t* quatStruct)
//uint8_t getARVRQuat(uint8_t *inputRep, Quaternion_t *quatStruct)
{

	if (inputRep->reportID != 0x28)
	{
		return 0;
	}
	quatStruct->qi = inputRep->quat_i * SCALE_Q(14);
	quatStruct->qj = inputRep->quat_j * SCALE_Q(14);
	quatStruct->qk = inputRep->quat_k * SCALE_Q(14);
	quatStruct->qr = inputRep->quat_r * SCALE_Q(14);
	quatStruct->acc = inputRep->accEstim * SCALE_Q(12);

	return 1;

}

void canConfig(void)
{
	  // Configure CAN filter
	  canFltrCfg.FilterMode = CAN_FILTERMODE_IDLIST;
	  //canFltrCfg.FilterMode = CAN_FILTERMODE_IDMASK;
	  canFltrCfg.FilterActivation = CAN_FILTER_ENABLE;
	  canFltrCfg.FilterBank = 0;
	  canFltrCfg.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	  canFltrCfg.FilterIdHigh = (0x07EC << 5);
	  canFltrCfg.FilterIdLow = 0x0000;
	  //canFltrCfg.FilterMaskIdHigh = 0x0000;
	  canFltrCfg.FilterMaskIdHigh = (0x07E8 << 5);
	  canFltrCfg.FilterMaskIdLow = 0x0000;
	  canFltrCfg.FilterScale = CAN_FILTERSCALE_32BIT;

	  // Push filter configuration to CAN state machine
	  if (HAL_CAN_ConfigFilter(&hcan, &canFltrCfg) != HAL_OK)
	  {
		  Error_Handler();
	  }

	  // Configure message to be sent
	  canTxHeader.DLC = 8;
	  //canTxHeader.ExtId = 0x4040440;
	  //canTxHeader.StdId = 0x1;
	  canTxHeader.IDE = CAN_ID_EXT;
	  //canTxHeader.IDE = CAN_ID_STD;
	  canTxHeader.RTR = CAN_RTR_DATA;

	  // Put CAN state machine in ready mode (out of sleep and out of init)
	  if (HAL_CAN_Start(&hcan) != HAL_OK)
	  {
		  Error_Handler();
	  }

	  // if both MSR bit 0 (INAK) and MSR bit 1 (SLAK) are at zero, then CAN state machine is in ready mode
	  while (((hcan.Instance->MSR & (SET << 0)) == (SET << 0)) || ((hcan.Instance->MSR & (SET << 1)) == (SET << 1)));

}

void ClearCANPendingMessage(CAN_HandleTypeDef *pCANHandle, CAN_TxHeaderTypeDef *pHeader, uint8_t *pTxData, uint32_t *pTxMailbox)
{
	// Before sending new message, verify status of CAN engine
	// If there is a transmit error CAN->TSR->TERR0 == 1
	// Abort previous request by setting CAN->TSR->ABRQ0
	if ((pCANHandle->Instance->TSR & CAN_TSR_TERR0) == CAN_TSR_TERR0)
	{
		// request to abort previous tx
		SET_BIT(pCANHandle->Instance->TSR, CAN_TSR_ABRQ0);
		// set CAN master reset
	    SET_BIT(pCANHandle->Instance->MCR, CAN_MCR_RESET);
	    // force clear TERR0
	    CLEAR_BIT(pCANHandle->Instance->TSR, CAN_TSR_TERR0);
	    // reset can error status register?
	    //pCANHandle->Instance->TSR = 0;
		// go to initialization mode
	    //SET_BIT(pCANHandle->Instance->MCR, CAN_MCR_INRQ);
	    // go to normal mode (by clearing init flag
	    //CLEAR_BIT(pCANHandle->Instance->MCR, CAN_MCR_INRQ);
	    // go to normal mode (by clearing sleep flag
	    CLEAR_BIT(pCANHandle->Instance->MCR, CAN_MCR_SLEEP);

	}
}

void TransmitCANMessage(CAN_HandleTypeDef *pCANHandle, CAN_TxHeaderTypeDef *pHeader, uint8_t *pTxData, uint32_t *pTxMailbox)
{

    // enable no automatic retransmission bit (CAN message will only be sent once regardless of result)
    SET_BIT(pCANHandle->Instance->MCR, CAN_MCR_NART);
	  // Queue next CAN message for car OBD PID readout
	  if (HAL_CAN_AddTxMessage(pCANHandle, pHeader, pTxData, pTxMailbox) != HAL_OK)
	  {
		  Error_Handler();
	  }

	  // Wait for message to make it out on the CAN bus
	  //while (HAL_CAN_IsTxMessagePending(pCANHandle, *pTxMailbox) == 1);
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
