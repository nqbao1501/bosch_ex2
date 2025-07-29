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
#include "dcm.h"
#include "dcm_rdbi.h"
#include "dcm_wdbi.h"
#include "dcm_seca.h"
#include <stdbool.h>

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

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t uart3_receive;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
CAN_TxHeaderTypeDef CAN1_pHeader;
CAN_RxHeaderTypeDef CAN1_pHeaderRx;
CAN_FilterTypeDef CAN1_sFilterConfig;
CAN_TxHeaderTypeDef CAN2_pHeader;
CAN_RxHeaderTypeDef CAN2_pHeaderRx;
CAN_FilterTypeDef CAN2_sFilterConfig;
uint32_t CAN1_pTxMailbox;
uint32_t CAN2_pTxMailbox;

uint16_t NumBytesReq = 0;
uint8_t  REQ_BUFFER  [4096];
uint8_t  REQ_1BYTE_DATA;

uint8_t CAN1_DATA_TX[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t CAN1_DATA_RX[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t CAN2_DATA_TX[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t CAN2_DATA_RX[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

uint16_t Num_Consecutive_Tester;
uint8_t  Flg_Consecutive = 0;

uint8_t msg_counter = 0;
uint8_t prev_msg_counter = 0;

uint16_t ECU_ID = 0x0078;
uint16_t Tester_ID = 0x00A2;

uint8_t block_size = 0x08;
uint8_t ST_min = 0x19; //25 in decimal

uint8_t consecutive_sequence_number = 0;



typedef enum{
	STATE_PREPARING_FOR_CAN2_TRANSMISSION,
	STATE_CAN2_TRANSMISSION,
	STATE_READING_CAN1_RECEPTION,
	STATE_PREPARING_FOR_CAN1_TRANSMISSION,
	STATE_CAN1_TRANSMISSION,
	STATE_PREPARING_FOR_CAN2_FLOW_CONTROL,
	STATE_PREPARING_FOR_CAN1_CONSECUTIVE_FRAME,
	STATE_PREPARING_FOR_CAN2_FIRST_FRAME,
	STATE_PREPARING_FOR_CAN1_FLOW_CONTROL,
	STATE_PREPARING_FOR_CAN2_CONSECUTIVE_FRAME,
	STATE_PREPARING_FOR_CAN1_SINGLE_FRAME
}SystemState;

SystemState currentState = STATE_PREPARING_FOR_CAN2_TRANSMISSION;
unsigned int TimeStamp;
// maximum characters send out via UART is 30
char bufsend[32]="XXX: D1 D2 D3 D4 D5 D6 D7 D8  ";
bool falsecounter = false;
bool falsesum = false;


#define UART_RX_BUFFER_SIZE 100
uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
uint8_t uart_rx_index = 0;
uint8_t uart_byte;
uint8_t parsed_values_from_uart_count = 0;
uint8_t parsed_values_from_uart[20];

uint8_t seed[6];
uint8_t key[6];
uint8_t key_from_user[6];
uint8_t SID;

bool seed_sent;
bool security_access_granted;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void MX_CAN1_Setup();
void MX_CAN2_Setup();
void USART3_SendString(uint8_t *ch);
void PrintCANLog(uint16_t CANID, uint8_t * CAN_Frame);

void delay(uint16_t delay);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Đây là callback, chạy sau khi node 1 hoặc node 2 nhận dữ liệu

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData);
    if (hcan->Instance == CAN1) {
        if (currentState == STATE_CAN2_TRANSMISSION){
        	for (uint8_t i = 0; i < 8; i++){
        		CAN1_DATA_RX[i] = rxData[i];
        	}
        if ((CAN1_DATA_RX[0] >> 4) == 0x00) currentState = STATE_READING_CAN1_RECEPTION;
        if ((CAN1_DATA_RX[0] >> 4) == 0x01) currentState = STATE_PREPARING_FOR_CAN1_FLOW_CONTROL;
        if ((CAN1_DATA_RX[0] >> 4) == 0x02) currentState = STATE_PREPARING_FOR_CAN1_SINGLE_FRAME;
        if ((CAN1_DATA_RX[0] >> 4) == 0x03) currentState = STATE_PREPARING_FOR_CAN1_CONSECUTIVE_FRAME;

    }
    }
    if (hcan->Instance == CAN2) {
        if (currentState == STATE_CAN1_TRANSMISSION){
        	for (uint8_t i = 0; i < 8; i++){
        		CAN2_DATA_RX[i] = rxData[i];
        	}
        if ((CAN2_DATA_RX[0] >> 4) == 0x00) currentState = STATE_PREPARING_FOR_CAN2_TRANSMISSION;
        if ((CAN2_DATA_RX[0] >> 4) == 0x01) currentState = STATE_PREPARING_FOR_CAN2_FLOW_CONTROL;
        if ((CAN2_DATA_RX[0] >> 4) == 0x02) currentState = STATE_PREPARING_FOR_CAN2_TRANSMISSION;
        if ((CAN2_DATA_RX[0] >> 4) == 0x03) currentState = STATE_PREPARING_FOR_CAN2_CONSECUTIVE_FRAME;

        }

    }
}

void set_LED (bool LED_State) {
	if (LED_State) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
}

// button interupt IG_OFF -> IG_ON
uint32_t last_debounce_time = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_1) {
    if (HAL_GetTick() - last_debounce_time > 200) // 200ms debounce
    {
      last_debounce_time = HAL_GetTick();

      USART3_SendString((uint8_t *)"IG OFF ");
      MX_CAN1_Setup();
      MX_CAN2_Setup();
      USART3_SendString((uint8_t *)"-> IG ON \n");
    }
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint16_t i,j = 0;
	uint16_t Consecutive_Cntr = 0;
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  MX_CAN1_Setup();
  MX_CAN2_Setup();
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
  HAL_UART_Receive_IT(&huart3, &REQ_1BYTE_DATA, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // Example Function to print can message via uart



  while (1)
  {
	  set_LED(security_access_granted);
	  switch (currentState){
			case STATE_PREPARING_FOR_CAN2_TRANSMISSION:
				//Từ các giá trị lấy được ở UART, xây dựng CAN_frame
				if (NumBytesReq != 0){
					HAL_Delay(200);

					if (REQ_BUFFER[0] == 0x27 && REQ_BUFFER[1] == 0x02 && NumBytesReq == 8){
		  				currentState = STATE_PREPARING_FOR_CAN2_FIRST_FRAME;
		  				break;
					}

					prepare_CAN_TX_frame(CAN2_DATA_TX, REQ_BUFFER, NumBytesReq);
					//Gửi CAN_frame
					currentState = STATE_CAN2_TRANSMISSION;
					CAN2_SendMessage(CAN2_DATA_TX);
					PrintCANLog(CAN2_pHeader.StdId, CAN2_DATA_TX);
					memset(REQ_BUFFER, 0, 20);
					NumBytesReq = 0;
				}

				break;

	  		case STATE_CAN2_TRANSMISSION:
	  			break;

	  		case STATE_PREPARING_FOR_CAN2_FIRST_FRAME:
	  			currentState = STATE_CAN2_TRANSMISSION;

				//Từ các giá trị lấy được ở UART, xây dựng CAN_frame
				prepare_CAN_First_Frame(CAN2_DATA_TX, REQ_BUFFER, NumBytesReq);
				//Gửi CAN_frame
				CAN2_SendMessage(CAN2_DATA_TX);
				PrintCANLog(CAN2_pHeader.StdId, CAN2_DATA_TX);
				break;

	  		case STATE_PREPARING_FOR_CAN2_FLOW_CONTROL:
	  			currentState = STATE_CAN2_TRANSMISSION;
	  			prepare_CAN_Flow_Control_Frame (CAN2_DATA_TX);
	  			CAN2_SendMessage(CAN2_DATA_TX);
	  			PrintCANLog(CAN2_pHeader.StdId, CAN2_DATA_TX);
	  			break;

	  		case STATE_PREPARING_FOR_CAN2_CONSECUTIVE_FRAME:
	  			currentState = STATE_CAN2_TRANSMISSION;
	  			prepare_CAN_Consecutive_Frames(CAN2_DATA_TX, &REQ_BUFFER[6], 2);
				memset(REQ_BUFFER, 0, 20);
				NumBytesReq = 0;
	  			break;

	  		case STATE_READING_CAN1_RECEPTION:
	  			/*
	  			 * Đọc CAN_Frame nhận được
	  			 * CAN_Frame[0] chứa số phần tử ở đằng sau
	  			 * CAN_Frame[1] chứa mã lệnh (SID) -> so sánh mã lệnh này có phải 1 trong 3 mã lệnh 22 27 2E không,
	  			 * 	-> Nếu không: reset state machine và break
	  			 * 	-> Nếu có: Đưa vào phương trình phân tích lệnh 22 27 2E
	  			 */
	  			switch (CAN1_DATA_RX[1]) {
					case 0x22:
						currentState = STATE_PREPARING_FOR_CAN1_TRANSMISSION;
						SID_22_Practice();
						break;
					case 0x27:
						currentState = STATE_PREPARING_FOR_CAN1_TRANSMISSION;
						SID_27_Practice();
						break;
					case 0x2E:
						currentState = STATE_PREPARING_FOR_CAN1_TRANSMISSION;
						SID_2E_Practice();
						break;
					default:
						currentState = STATE_PREPARING_FOR_CAN2_TRANSMISSION;
						break;
				}
	  			break;

	  		case STATE_PREPARING_FOR_CAN1_CONSECUTIVE_FRAME:
	  			if (((CAN1_DATA_RX[0]) & 0x0F) == 0x02) currentState = STATE_PREPARING_FOR_CAN2_TRANSMISSION;
	  			if (((CAN1_DATA_RX[0]) & 0x0F) == 0x00) {currentState = STATE_CAN1_TRANSMISSION;
	  				SID_27_Practice();
	  			}
	  			break;

	  		case STATE_PREPARING_FOR_CAN1_FLOW_CONTROL:
	  			currentState = STATE_CAN1_TRANSMISSION;
	  			SID_27_Practice();
				CAN1_SendMessage(CAN1_DATA_TX);
				PrintCANLog(CAN1_pHeader.StdId, CAN1_DATA_TX);
				break;

	  		case STATE_PREPARING_FOR_CAN1_SINGLE_FRAME:
	  			currentState = STATE_CAN1_TRANSMISSION;
	  			SID_27_Practice();
	  			CAN1_SendMessage(CAN1_DATA_TX);
	  		    PrintCANLog(CAN1_pHeader.StdId, CAN1_DATA_TX);
	  		    break;

	  		case STATE_PREPARING_FOR_CAN1_TRANSMISSION:
	  			currentState = STATE_CAN1_TRANSMISSION;
	  			CAN1_SendMessage(CAN1_DATA_TX);
	  			PrintCANLog(CAN1_pHeader.StdId, CAN1_DATA_TX);
	  			break;

	  		case STATE_CAN1_TRANSMISSION:
	  			break;

	  	}
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
  RCC_OscInitStruct.PLL.PLLN = 80;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 5;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
	//Cần config các setting của filter ở đây


  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC4 PC5 PC6
                           PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void MX_CAN1_Setup()
{
  	CAN1_sFilterConfig.SlaveStartFilterBank = 14;
	CAN1_sFilterConfig.FilterBank = 0;
	CAN1_sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN1_sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN1_sFilterConfig.FilterIdHigh = Tester_ID << 5;
	CAN1_sFilterConfig.FilterIdLow = 0x0000;
	CAN1_sFilterConfig.FilterMaskIdHigh = 0x7FF << 5;
	CAN1_sFilterConfig.FilterMaskIdLow = 0x0000;
	CAN1_sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	CAN1_sFilterConfig.FilterActivation = ENABLE;

	HAL_CAN_ConfigFilter(&hcan1, &CAN1_sFilterConfig);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	//Cần config header của CAN1
    CAN1_pHeader.StdId = ECU_ID;                   // Set TX message ID
    CAN1_pHeader.IDE = CAN_ID_STD;                // Use standard ID
    CAN1_pHeader.RTR = CAN_RTR_DATA;              // Sending data, not a request
    CAN1_pHeader.DLC = 8;                         // 8 bytes of data
    CAN1_pHeader.TransmitGlobalTime = DISABLE;    // No timestamping
}
void MX_CAN2_Setup()
{
	CAN2_sFilterConfig.FilterBank = 14;
	CAN2_sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN2_sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN2_sFilterConfig.FilterIdHigh = ECU_ID << 5;
	CAN2_sFilterConfig.FilterIdLow = 0x0000;
	CAN2_sFilterConfig.FilterMaskIdHigh = 0x7FF << 5;
	CAN2_sFilterConfig.FilterMaskIdLow = 0x0000;
	CAN2_sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	CAN2_sFilterConfig.FilterActivation = ENABLE;

	HAL_CAN_ConfigFilter(&hcan2, &CAN2_sFilterConfig);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

	//Cần config header của CAN2
    CAN2_pHeader.StdId = Tester_ID;                // Set TX message ID
    CAN2_pHeader.IDE = CAN_ID_STD;                // Use standard ID
    CAN2_pHeader.RTR = CAN_RTR_DATA;              // Sending data, not a request
    CAN2_pHeader.DLC = 8;                         // 8 bytes of data
    CAN2_pHeader.TransmitGlobalTime = DISABLE;    // No timestamping
}

void USART3_SendString(uint8_t *ch)
{
   while(*ch!=0)
   {
      HAL_UART_Transmit(&huart3, ch, 1,HAL_MAX_DELAY);
      ch++;
   }
}
void PrintCANLog(uint16_t CANID, uint8_t *CAN_Frame)
{
    char bufsend[64];  // Safe enough size
    uint16_t i = 0;

    // Format timestamp
    int len = sprintf(bufsend, "%d ", TimeStamp);

    // Format CAN ID (always 3 digits, uppercase hex)
    len += sprintf(bufsend + len, "%03X: ", CANID & 0x7FF);  // mask to 11 bits

    // Format 8 bytes of CAN data
    for (i = 0; i < 8; i++) {
        len += sprintf(bufsend + len, "%02X ", CAN_Frame[i]);
    }

    // End with CRLF
    bufsend[len++] = '\r';
    bufsend[len++] = '\n';
    bufsend[len] = '\0';

    // Send over UART
    USART3_SendString((uint8_t *)bufsend);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	REQ_BUFFER[NumBytesReq] = REQ_1BYTE_DATA;
	NumBytesReq++;
	HAL_UART_Receive_IT(&huart3, &REQ_1BYTE_DATA, 1);

	//REQ_BUFFER[7] = NumBytesReq;
}
void delay(uint16_t delay)
{
	HAL_Delay(delay);
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
