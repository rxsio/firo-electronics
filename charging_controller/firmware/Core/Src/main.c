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
#include "stm32f1xx_ll_adc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEMPSENSOR_TYP_AVGSLOPE 4300   // Average slope in µV/°C (4.3 mV/°C)
#define TEMPSENSOR_TYP_CALX_V 1430     // Calibration voltage in mV (1.43V)
#define TEMPSENSOR_CALX_TEMP 25        // Calibration temperature in °C
#define VREFANALOG_VOLTAGE 3300        // Reference voltage in mV (3.3V)
#define ADC_RESOLUTION 4096            // 12-bit ADC resolution
#define TEMPERATURE_TRESHOLD 65
#define SENSE_CHARGING GPIO_PIN_SET
#define SENSE_NOT_CHARGING GPIO_PIN_RESET
#define BATTERY_CONNECTED GPIO_PIN_SET
#define BATTERY_DISCONNECTED GPIO_PIN_RESET
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

// TIM2 is used for user timeouts in SetTimeout and ClearTimeout
// TIM3 is used for ADC sampling

// Hardcoded can node id for this device
const uint8_t NODE_ID = 0x60;

/**
 * @brief CAN command and response IDs
 * 
 * These enums represent the command and response IDs used in data field in CAN communication.
 */
typedef enum {
    CAN_CMD_ATTEMPT_CHARGING       = 0x00,
    CAN_CMD_STOP_CHARGING          = 0x01,

    // Responses are assigned IDs from upper half of the command IDs range 
    // although there is no particular benefit or reason behind this
    CAN_RESP_CHARGING_SUCCESS      = 0x08,
    CAN_RESP_CHARGING_FAIL         = 0x09,
    CAN_RESP_BUS_OFF_RECOVERY      = 0x0A,
    CAN_RESP_OVERHEATED			       = 0x0B,
    CAN_RESP_OVERHEAT_RECOVERY	   = 0x0C,
    CAN_RESP_FATAL_ERROR_RECOVERY  = 0x0D,
    CAN_RESP_STATUS     		       = 0x0F

} CAN_CommandID;

// CAN Header buffer for received CAN commands
// Consider moving this to HAL_CAN_RxFifo0MsgPendingCallback
CAN_RxHeaderTypeDef   RxHeader;

// CAN data buffer for received CAN commands
// Also consider moving this to HAL_CAN_RxFifo0MsgPendingCallback
uint8_t               RxData[8];

// Error flags
// "volatile" because they are set in interrupts and used elsewhere
volatile uint8_t CAN_BUS_OFF = 0;
volatile uint8_t OVERHEAT = 0;


// Buffer for ADC measurements
uint32_t adc_buffer[1];

// ADC measurements converted to temperature in degrees
// Consider moving this to HAL_ADC_ConvCpltCallback
int8_t temperature;


// Function pointer for user callback used in timeouts methods
// Don't use it directly. Use SetTimeout and ClearTimeout
void (*_TimeoutCallback)(void) = NULL;  

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void DMA_TransferCompleteCallback(DMA_HandleTypeDef *hdma);

HAL_StatusTypeDef SendCanMessage(const uint8_t command_id);
void RecoveryTimeout();

void SetTimeout(void (*callback)(void), uint32_t time_ms);
void ClearTimeout(void (*callback)(void));

void StartChargingSequence();
void ChargingCheckTimeout();
void CompleteChargingSequence();
void DisconnectBattery();

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
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  CAN_FilterTypeDef canfilterconfig;
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 0;
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;

  // In IDLIST mode, property names can be misleading
  // (FilterIdHigh, FilterIdLow, FilterMaskIdHigh, and FilterMaskIdLow).
  // For CAN_FILTERSCALE_16BIT, each property is just a standard 11-bit ID to whitelist.
  // For CAN_FILTERSCALE_32BIT, FilterIdHigh and FilterIdLow together represent one extended 29-bit ID,
  // and FilterMaskIdHigh and FilterMaskIdLow together represent the second extended ID.
  // For better understanding, refer to the memory layout in the MCU datasheet.
  // In our case, we are only using standard IDs.
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDLIST;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_16BIT;


  // Standard 11-bit IDs must be shifted left by 5 bits to align with the left 11 bits of the 16-bit filter registers.
  // The right 5 bits can be set to 0 since they are used for extended 29-bit IDs, except for the RTR bit.
  // The RTR (Remote Transmission Request) bit can be set by appending | 0x10.
  // RTR messages are used to request temperature
  canfilterconfig.FilterIdHigh = (NODE_ID + (CAN_CMD_ATTEMPT_CHARGING << 7)) << 5;  // 1st 11-bit ID
  canfilterconfig.FilterIdLow = (NODE_ID + (CAN_CMD_STOP_CHARGING << 7)) << 5;  // 2nd 11-bit ID
  canfilterconfig.FilterMaskIdHigh = ((NODE_ID + (CAN_RESP_STATUS << 7)) << 5) | 0x10;  // 3rd 11-bit ID
  canfilterconfig.FilterMaskIdLow = (NODE_ID + (CAN_CMD_ATTEMPT_CHARGING << 7)) << 5;  // 4th 11-bit ID - not used

  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

  // Activate CAN notifications, so our interrupt handlers are actually called
  // Some of those notifications can probably be omitted, but this would require careful testing
  if (HAL_CAN_ActivateNotification(&hcan, \
		    CAN_IT_TX_MAILBOX_EMPTY     | \
		    CAN_IT_RX_FIFO0_MSG_PENDING | \
		    CAN_IT_RX_FIFO0_FULL        | \
		    CAN_IT_RX_FIFO0_OVERRUN     | \
		    CAN_IT_RX_FIFO1_MSG_PENDING | \
		    CAN_IT_RX_FIFO1_FULL        | \
		    CAN_IT_RX_FIFO1_OVERRUN     | \
		    CAN_IT_ERROR_WARNING        | \
		    CAN_IT_ERROR_PASSIVE        | \
		    CAN_IT_BUSOFF               | \
		    CAN_IT_LAST_ERROR_CODE      | \
		    CAN_IT_ERROR                ) != HAL_OK) {
    Error_Handler();
  }
  HAL_CAN_Start(&hcan);

  // Checks if the reset was caused by a software reset (fatal error recovery)
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) {
    // Inform the user about the fatal error
    SendCanMessage(CAN_RESP_FATAL_ERROR_RECOVERY);
  }
  // Clear the reset flags
  __HAL_RCC_CLEAR_RESET_FLAGS();

  HAL_ADC_Start_DMA(&hadc1, adc_buffer, 1);

  // Start the timer for ADC sampling
  if (HAL_TIM_Base_Start(&htim3) != HAL_OK) {
      // Starting Error
      Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 1;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CHARGE_PULSE_GPIO_Port, CHARGE_PULSE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BATTERY_CUTOFF_GPIO_Port, BATTERY_CUTOFF_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CHARGE_PULSE_Pin */
  GPIO_InitStruct.Pin = CHARGE_PULSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CHARGE_PULSE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BATTERY_CUTOFF_Pin */
  GPIO_InitStruct.Pin = BATTERY_CUTOFF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BATTERY_CUTOFF_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CHARGE_SENSE_Pin */
  GPIO_InitStruct.Pin = CHARGE_SENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CHARGE_SENSE_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * Handles the pending CAN messages by calling 
 * the appropriate function based on the command ID.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (CAN_BUS_OFF) {
    // Ignore incoming messages if in error state
    // We are unable to respond in this state anyway
    return;
  }
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
    Error_Handler();
  }
  if(RxHeader.RTR == CAN_RTR_REMOTE){
    // Receiver request for temperature measurement
    
    uint8_t charging = 0x00;
    if(HAL_GPIO_ReadPin(CHARGE_SENSE_GPIO_Port, CHARGE_SENSE_Pin) == SENSE_CHARGING){
      charging = 0xFF;
    }
    
	  uint32_t TxMailbox;
	  uint8_t TxData[2] = {*(uint8_t *) &temperature, charging};
    const CAN_TxHeaderTypeDef TxHeader = {
      .IDE = CAN_ID_STD,
      .StdId = NODE_ID + (CAN_RESP_STATUS << 7),  
      .RTR = CAN_RTR_DATA,  
      .DLC = 2             
    };
	  HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox);
	  return;
  }
  if(OVERHEAT){
    // Ignore commands if the device is overheated
	  return;
  }
  switch (RxHeader.StdId >> 7) {
      case CAN_CMD_ATTEMPT_CHARGING:
    	  StartChargingSequence();
         break;
      case CAN_CMD_STOP_CHARGING:
    	  DisconnectBattery();
    	  break;
      default:
          // Unknown command
          break;
  }
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
  // clears bus off error flag after a successful CAN transmission.
  CAN_BUS_OFF = 0;
}


/**
 * Handle CAN errors, specifically the bus-off error.
 * Retract the probe and set a timeout for recovery.
 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    uint32_t error = HAL_CAN_GetError(hcan);

    if (error & HAL_CAN_ERROR_BOF) {
        CAN_BUS_OFF = 1;

        // Abort sending any messages
        HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX0);
        HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX1);
        HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX2);

        SetTimeout(RecoveryTimeout, 500);
    }

    // Todo: Are there any other errors we have to handle?
}

/**
 * Call the user-defined timeout callback set by SetTimeout function.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
      if (_TimeoutCallback != NULL) {
    	  void (*_CurrentCallback)(void)  = _TimeoutCallback;
        _TimeoutCallback();

        // Clear the callback unless user already set new one
        // TODO: If new callback points to the same function, it will be cleared as well
        // This is not yet a problem, but it might be a potential bug
        // Better solution is desirable
        if(_CurrentCallback !=_TimeoutCallback){
          return;
        }
        _TimeoutCallback = NULL;
      }
      // Stop the timer
      HAL_TIM_Base_Stop_IT(&htim2);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

	// Process the ADC data
	temperature = __LL_ADC_CALC_TEMPERATURE_TYP_PARAMS(
					TEMPSENSOR_TYP_AVGSLOPE,
					TEMPSENSOR_TYP_CALX_V,
					TEMPSENSOR_CALX_TEMP,
					VREFANALOG_VOLTAGE,
					adc_buffer[0],
					ADC_RESOLUTION);

	// Check temperature and take action if needed
	if (OVERHEAT)
	{
    // TODO: Consider adding hysteresis to prevent rapid toggling and saturating the CAN bus
		if(temperature <= TEMPERATURE_TRESHOLD){
			OVERHEAT = 0;
			SendCanMessage(CAN_RESP_OVERHEAT_RECOVERY);
		}
	}
	else if(temperature > TEMPERATURE_TRESHOLD){
		OVERHEAT = 1;
		SendCanMessage(CAN_RESP_OVERHEATED);
	}
}

/**
 * @brief Sets a timeout callback
 * 
 * Sets a user-defined callback to be called after a specified time in ms.
 */
void SetTimeout(void (*callback)(void), uint32_t time_ms) {
    // Stop timer If enabled
    HAL_TIM_Base_Stop_IT(&htim2);

    _TimeoutCallback = callback;
    // Update the timer period
    __HAL_TIM_SET_AUTORELOAD(&htim2, time_ms - 1);
    // Reset the counters
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    // Start the timer with interrupt
    HAL_TIM_Base_Start_IT(&htim2);
}


/**
 * @brief Clears a timeout callback
 * 
 * Clear the user-defined timeout callback if it matches the saved callback.
 */
void ClearTimeout(void (*callback)(void)) {
  if (_TimeoutCallback == callback) {
      HAL_TIM_Base_Stop_IT(&htim2);
      _TimeoutCallback = NULL;  // Clear the callback
  }
}
/**
 * @brief Sends a CAN message
 * 
 * Send a CAN message with the specified command ID.
 * If the CAN bus is off, send a recovery message instead.
 */
HAL_StatusTypeDef SendCanMessage(const CAN_CommandID command_id) {
    uint32_t TxMailbox;

    // Abort pending messages in all transmit mailboxes
    HAL_CAN_AbortTxRequest(&hcan, CAN_TX_MAILBOX0);
    HAL_CAN_AbortTxRequest(&hcan, CAN_TX_MAILBOX1);
    HAL_CAN_AbortTxRequest(&hcan, CAN_TX_MAILBOX2);

    if (!CAN_BUS_OFF) {
      // Add the new message to the transmit mailbox
      const CAN_TxHeaderTypeDef TxHeader = {
        .IDE = CAN_ID_STD,
        .StdId = NODE_ID + (command_id << 7),  
        .RTR = CAN_RTR_DATA,  
        .DLC = 0             
      };
      return HAL_CAN_AddTxMessage(&hcan, &TxHeader, &command_id, &TxMailbox);
    } else {
      // Send recovery message if in error state
      const CAN_CommandID recovery_id = CAN_RESP_BUS_OFF_RECOVERY;
      const CAN_TxHeaderTypeDef TxHeader = {
        .IDE = CAN_ID_STD,
        .StdId = NODE_ID + (recovery_id << 7),  
        .RTR = CAN_RTR_DATA,  
        .DLC = 0             
      };
      return HAL_CAN_AddTxMessage(&hcan, &TxHeader, &recovery_id, &TxMailbox);
    }
}

/**
 * @brief Timeout callback for recovery
 * 
 * Send a recovery message/
 */
void RecoveryTimeout() {
  // Data actually doesn't matter in case of bus off error
  // SendCanMessage will automatically replace any message with recovery message
  SendCanMessage(CAN_RESP_BUS_OFF_RECOVERY);
}


/**
 * @brief Timeout callback for charging check
 * 
 * Check the charging status and send a success or failure message.
 */
void ChargingCheckTimeout() {
  if (HAL_GPIO_ReadPin(CHARGE_SENSE_GPIO_Port, CHARGE_SENSE_Pin) == SENSE_CHARGING) {
      SendCanMessage(CAN_RESP_CHARGING_SUCCESS);
      HAL_GPIO_WritePin(BATTERY_CUTOFF_GPIO_Port, BATTERY_CUTOFF_Pin, BATTERY_CONNECTED);
  } else {
      SendCanMessage(CAN_RESP_CHARGING_FAIL);
  }
}

/**
 * @brief Completes the charging sequence
 * 
 * Reset the charge pulse pin and set a timeout for checking the charging status.
 */
void CompleteChargingSequence() {
  HAL_GPIO_WritePin(CHARGE_PULSE_GPIO_Port, CHARGE_PULSE_Pin, GPIO_PIN_RESET);
  SetTimeout(ChargingCheckTimeout, 50);
}

/**
 * @brief Starts the charging sequence
 * 
 * Set the pulse pin to initiate charging and start a timeout to complete the charging sequence
 * after 400 milliseconds.
 */
void StartChargingSequence() {
  HAL_GPIO_WritePin(CHARGE_PULSE_GPIO_Port, CHARGE_PULSE_Pin, GPIO_PIN_SET);
  DisconnectBattery();
  SetTimeout(CompleteChargingSequence, 300);
}

void DisconnectBattery(){
  HAL_GPIO_WritePin(BATTERY_CUTOFF_GPIO_Port, BATTERY_CUTOFF_Pin, BATTERY_DISCONNECTED);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    // Disable interrupts to prevent any further operation
    __disable_irq();

    // 1. Disable pulse pin
    HAL_GPIO_WritePin(CHARGE_PULSE_GPIO_Port, CHARGE_PULSE_Pin, GPIO_PIN_RESET); 
    // 2. Reset the system immediately as a last resort to recover from unhandled or fatal errors
    HAL_NVIC_SystemReset();

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
