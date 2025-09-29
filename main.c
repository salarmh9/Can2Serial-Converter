/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "string.h"
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

FDCAN_HandleTypeDef hfdcan1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart7;
DMA_HandleTypeDef hdma_uart7_rx;
DMA_HandleTypeDef hdma_uart7_tx;

/* USER CODE BEGIN PV */
//=========================== Blink_Led =============================================
uint16_t Blink_Timer       =0;
uint16_t Blink_Timeout     =100;
//========================= RX_USART_DMA Initial =========================================
#define   RX_USART_BUFFER_SIZE                              200 // its for Boudrate =921600 in 1ms
uint8_t   RX_USART_Receive_Buffer[RX_USART_BUFFER_SIZE]     ={0};
//=========================== RX_USART_Receive ===========================================
#define   RX_USART_HEADER_0                                 0xAA
#define   RX_USART_HEADER_1                                 0x55
#define   RX_USART_FOOTER_0                                 0x81
#define   RX_USART_PATTERN_SIZE                             16
uint8_t   RX_USART_Last_Data_Rec_Address                    =0;
uint8_t   RX_USART_Current_Data_Rec_Address                 =0;
uint8_t   RX_USART_Data_Input[RX_USART_BUFFER_SIZE]         ={0};
uint8_t   RX_USART_Num_Of_Data                              =0;
uint8_t   RX_USART_Protocol_State                  		    =0;
uint8_t   RX_USART_Protocol_Counter                			=0;
uint8_t   RX_USART_Pattern[RX_USART_PATTERN_SIZE]           ={0};
uint16_t  RX_USART_Checkdigit                               =0;
uint8_t   RX_USART_Data_Refresh                             =0;
#define   USART_RX_INBOX_NUM                                40
typedef   struct{
	int8_t   Transmit_Permision;
	uint8_t  Data[16];
}rx_usart_msg_t;
uint8_t   U_RX_Inbox_Cnt                                    =0;
 rx_usart_msg_t   USART_Rx_Inbox[USART_RX_INBOX_NUM]        ={0};
#define                       USART_BOX_EMPTY                -1
 //=============================== CAN_Transmit ==========================================
 FDCAN_TxHeaderTypeDef        TxHeader;
 uint8_t                      Can_Tx_Free_Level             =0;
 uint8_t                      Can_Tx_Msg_Index              =0;
 uint8_t                      CN_TX_Sendbox_Cnt             =0;

 //=============================== CAN_Receive ===========================================
#define                       CAN_RX_INBOX_NUM             10
#define                       CAN_BOX_EMPTY                -1
 FDCAN_RxHeaderTypeDef        RxHeader;
 uint8_t                      CAN_Rx_Data[8]               ={0};
 uint8_t                      CAN_Rx_fill_level            =0;
 typedef   struct{
	uint8_t   Header1;
	uint8_t   Header2;
	uint16_t  ID;
	uint8_t  DLC;
	uint8_t  Data[8];
	uint16_t Checksum;
	uint8_t  Footer;
 } __attribute__((packed)) 	can_tx_packet_t;
typedef  union{
	 uint8_t           Direct_Arr[16];
	 can_tx_packet_t   Indirect_Str;
}can_tx_packet_u;

typedef   struct{
	int8_t              turn;
	can_tx_packet_u     Data;
}Can_Send_Box_t;

Can_Send_Box_t               CAN_Send_Box[CAN_RX_INBOX_NUM] ={0};
uint8_t                      CN_RX_Inbox_Cnt                 =0;
//=========================== TX_USART_Transmit ===========================================
uint8_t                      USART_TX_Sendbox_Cnt            =0;
#define                      USART_PROTOCOL_SIZE             16
uint8_t                      U_Tx_Buff[USART_PROTOCOL_SIZE]  ={0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM6_Init(void);
static void MX_UART7_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
	   void Blink_Led         (void);
       void RX_USART_Receive  (void);
       void CAN_Transmit      (void);
       void CAN_Receive       (void);
       void TX_USART_Transmit (void);
       void Write_Msg_Box     (uint8_t);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_FDCAN1_Init();
  MX_TIM6_Init();
  MX_UART7_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  for(uint8_t y=0;y<USART_RX_INBOX_NUM;y++)
	  USART_Rx_Inbox[y].Transmit_Permision =USART_BOX_EMPTY;
//======================================================================= RX_USART =============//
  	UART7->CR1          &= ~(USART_CR1_UE);      // for  cash fix problem in dma use
   	UART7->CR3          |= (USART_CR3_DMAR) | (USART_CR3_ONEBIT);
  	UART7->CR1          |= USART_CR1_UE;

    DMA1_Stream0->PAR	 = (uint32_t) &UART7->RDR;
    DMA1_Stream0->M0AR   = (uint32_t) &RX_USART_Receive_Buffer;
    DMA1_Stream0->NDTR   = RX_USART_BUFFER_SIZE;
    DMA1_Stream0->CR    |=  DMA_SxCR_EN ;
//=========================================================================== CAN ==============//
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;

    if(HAL_FDCAN_Start(&hfdcan1)!= HAL_OK)
	{
			Error_Handler();
	}

    //**************************
    //-----------------------------------------------
    FDCAN_FilterTypeDef               Filter_0;
    Filter_0.IdType                 = FDCAN_STANDARD_ID;
    Filter_0.FilterIndex            = 0;
    Filter_0.FilterType             = FDCAN_FILTER_RANGE;
    Filter_0.FilterConfig           = FDCAN_FILTER_TO_RXFIFO0;
    Filter_0.FilterID1              = 0x000;
    Filter_0.FilterID2              = 0x7FF;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &Filter_0) != HAL_OK)
    {
        Error_Handler();
    }
//====================================================================== CAN_Receive ===========//
for(uint8_t i=0;i<CAN_RX_INBOX_NUM;i++)
	CAN_Send_Box[i].turn = CAN_BOX_EMPTY;
//================================================================== TX_USART_Transmit =========//
//HAL_DMA_Start_IT(&hdma_uart7_tx, &U_Tx_Buff[0], &UART7->TDR, USART_PROTOCOL_SIZE);
DMA1_Stream1->CR   &= ~(DMA_SxCR_EN);
DMA1->LIFCR        |= DMA_LIFCR_CTCIF1;
DMA1_Stream1->PAR	= (uint32_t) &UART7->TDR;
DMA1_Stream1->M0AR  = (uint32_t) &U_Tx_Buff[0] ;
DMA1_Stream1->NDTR  = USART_PROTOCOL_SIZE;
DMA1_Stream1->CR   |= DMA_SxCR_TCIE;
DMA1_Stream1->CR   |= DMA_SxCR_EN;

UART7->CR3         |= USART_CR3_DMAT;
//========================================================================== TIMER =============//
  	HAL_TIM_Base_Start_IT  (&htim6);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;//FDCAN_MODE_NORMAL //FDCAN_MODE_EXTERNAL_LOOPBACK
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 3;
  hfdcan1.Init.NominalSyncJumpWidth = 10;
  hfdcan1.Init.NominalTimeSeg1 = 69;
  hfdcan1.Init.NominalTimeSeg2 = 10;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 6;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 6;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

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
  htim6.Init.Prescaler = 240-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000-1;
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

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 921600;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMAMUX1_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMAMUX1_OVR_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMAMUX1_OVR_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	  if (htim->Instance == htim6.Instance)
  {
		  Blink_Led         ();
		  RX_USART_Receive  ();
		  CAN_Transmit      ();
		  CAN_Receive       ();
		  TX_USART_Transmit ();
  }

}


//================================================================
//name        : Blink_Led
//subject     : Blink a led test on board
//last modify : 1402/05/26
//================================================================

void Blink_Led (void){
 Blink_Timer++;
 if      (Blink_Timer  <=  Blink_Timeout)
		GPIOD->BSRR |= GPIO_BSRR_BS7;
 else if (Blink_Timer  <= (2*Blink_Timeout) )
		GPIOD->BSRR |= GPIO_BSRR_BR7;
 else
	 Blink_Timer=0;

}

//=========================================================================================== Serial To CAN

void RX_USART_Receive (void)
{

	RX_USART_Current_Data_Rec_Address = RX_USART_BUFFER_SIZE - DMA1_Stream0->NDTR;

	if( RX_USART_Current_Data_Rec_Address != RX_USART_Last_Data_Rec_Address)
	{
		if(RX_USART_Current_Data_Rec_Address > RX_USART_Last_Data_Rec_Address)
		{
		  RX_USART_Num_Of_Data = RX_USART_Current_Data_Rec_Address - RX_USART_Last_Data_Rec_Address;
		  memcpy(RX_USART_Data_Input,(RX_USART_Receive_Buffer + RX_USART_Last_Data_Rec_Address) ,RX_USART_Num_Of_Data);
		}
		else
		{
			RX_USART_Num_Of_Data = RX_USART_BUFFER_SIZE - RX_USART_Last_Data_Rec_Address + RX_USART_Current_Data_Rec_Address;
		  memcpy(RX_USART_Data_Input  ,(RX_USART_Receive_Buffer + RX_USART_Last_Data_Rec_Address)  ,RX_USART_BUFFER_SIZE - RX_USART_Last_Data_Rec_Address);
			memcpy(RX_USART_Data_Input + (RX_USART_BUFFER_SIZE - RX_USART_Last_Data_Rec_Address) ,RX_USART_Receive_Buffer  ,RX_USART_Current_Data_Rec_Address);
		}
		RX_USART_Last_Data_Rec_Address=RX_USART_Current_Data_Rec_Address;
	 for(uint8_t i=0;i<RX_USART_Num_Of_Data;i++)
     {
        switch (RX_USART_Protocol_State)
		{

         case 0:
					  if (RX_USART_Data_Input[i] == RX_USART_HEADER_0)
					  {
                         RX_USART_Protocol_State=1;
					     RX_USART_Pattern[RX_USART_Protocol_Counter]=RX_USART_Data_Input[i];
                         RX_USART_Protocol_Counter++;
                      }
		 break;

         case 1:
					  if (RX_USART_Data_Input[i] == RX_USART_HEADER_1)
					  {
                          RX_USART_Protocol_State=2;
						  RX_USART_Pattern[RX_USART_Protocol_Counter]=RX_USART_Data_Input[i];
						  RX_USART_Protocol_Counter++;
                      }
					  else if(RX_USART_Data_Input[i] == RX_USART_HEADER_0)
					  {
						  RX_USART_Protocol_State=1;
						  RX_USART_Protocol_Counter=1;
					  }
					  else
					  {
						  RX_USART_Protocol_State=0;
						  RX_USART_Protocol_Counter=0;
					   }
		  break;

          case 2:
						RX_USART_Pattern[RX_USART_Protocol_Counter] = RX_USART_Data_Input[i];
                        RX_USART_Protocol_Counter++;
						if(RX_USART_Protocol_Counter>=RX_USART_PATTERN_SIZE)
						{
								RX_USART_Protocol_State    =0;
								RX_USART_Protocol_Counter  =0;
								//-----------------------------------------------------
								RX_USART_Checkdigit	= 0;
								for (uint8_t j=0 ;j<RX_USART_PATTERN_SIZE-1-2 ;j++)
										RX_USART_Checkdigit  += RX_USART_Pattern[j];
								if ( (  (uint8_t)RX_USART_Checkdigit == RX_USART_Pattern[13])&&((uint8_t)(RX_USART_Checkdigit>>8) == RX_USART_Pattern[14]))
								{
									for(uint8_t i=0;i<USART_RX_INBOX_NUM;i++)
								    {
										if(USART_Rx_Inbox[i].Transmit_Permision == USART_BOX_EMPTY)
										{
											 memcpy(USART_Rx_Inbox[i].Data , RX_USART_Pattern, RX_USART_PATTERN_SIZE);
											 memset(RX_USART_Pattern, 0x00, RX_USART_PATTERN_SIZE);
											 USART_Rx_Inbox[i].Transmit_Permision = U_RX_Inbox_Cnt;
											 U_RX_Inbox_Cnt++;
											 if(U_RX_Inbox_Cnt >= USART_RX_INBOX_NUM) U_RX_Inbox_Cnt=0;
											 break;
										}

								     }
								}
				        }
		  break;
		  default:
            RX_USART_Protocol_State = 0;
          break;
	     }
	   }
     memset(RX_USART_Data_Input, 0x00, RX_USART_BUFFER_SIZE);
	}
RX_USART_Data_Refresh=0;

}

void CAN_Transmit      (void)
{
	Can_Tx_Free_Level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
	for(uint8_t i=0; (i<USART_RX_INBOX_NUM)&&(Can_Tx_Free_Level!=0) ;i++)
    {
			if(USART_Rx_Inbox[i].Transmit_Permision == CN_TX_Sendbox_Cnt)
			{
			 //***************************************************
			 uint16_t     can_tx_id  = ((uint16_t)( USART_Rx_Inbox[i].Data[3])<<8)|(USART_Rx_Inbox[Can_Tx_Msg_Index].Data[2]);
			 uint8_t      can_tx_dlc = USART_Rx_Inbox[i].Data[4];
			 uint8_t      can_tx_payload[8] = {0};
			 memcpy(can_tx_payload,&USART_Rx_Inbox[i].Data[5],8);
			 memset(&USART_Rx_Inbox[i].Data[0],0x00,16);
			 TxHeader.Identifier = can_tx_id;
			 TxHeader.DataLength = can_tx_dlc;

			 if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&TxHeader, &can_tx_payload[0]) != HAL_OK)
			 {
			 		Error_Handler();
			 }
			 //***************************************************
			 USART_Rx_Inbox[i].Transmit_Permision = USART_BOX_EMPTY;
			 CN_TX_Sendbox_Cnt++;
			 if(CN_TX_Sendbox_Cnt>=USART_RX_INBOX_NUM) CN_TX_Sendbox_Cnt=0;

			}
    }

}

//========================================================================================== CAN To Serial
void CAN_Receive       (void)
{
	CAN_Rx_fill_level = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0);
	for (uint32_t i = 0; i < CAN_Rx_fill_level; i++)
	{
		if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, CAN_Rx_Data) == HAL_OK)
		{
			for(uint8_t j=0;j<CAN_RX_INBOX_NUM;j++)
			{
                if(CAN_Send_Box[j].turn == CAN_BOX_EMPTY)
                	{
                	Write_Msg_Box(j);
                	break;
                	}
			}

		}
		memset(&CAN_Rx_Data[0],0x00,8);
	}
}

void Write_Msg_Box(uint8_t index)
{
	uint16_t    Checksum =0;

	CAN_Send_Box[index].Data.Indirect_Str.Header1 = 0xAA;
	CAN_Send_Box[index].Data.Indirect_Str.Header2 = 0x55;
	CAN_Send_Box[index].Data.Indirect_Str.ID  = (RxHeader.Identifier)&0x7ff;
	CAN_Send_Box[index].Data.Indirect_Str.DLC = RxHeader.DataLength;
	memcpy(CAN_Send_Box[index].Data.Indirect_Str.Data , CAN_Rx_Data,RxHeader.DataLength);
	for(uint8_t k=0;k<13;k++)
		  Checksum +=CAN_Send_Box[index].Data.Direct_Arr[k];
	CAN_Send_Box[index].Data.Indirect_Str.Checksum =Checksum;    /// ????
	CAN_Send_Box[index].Data.Indirect_Str.Footer =0x81;
	//************************************************************
	CAN_Send_Box[index].turn = CN_RX_Inbox_Cnt++;
	if(CN_RX_Inbox_Cnt >= (CAN_RX_INBOX_NUM)  )
		CN_RX_Inbox_Cnt=0;
}

void TX_USART_Transmit (void)
{
	for(uint8_t j=0;j<CAN_RX_INBOX_NUM;j++)
	{
        if(CAN_Send_Box[j].turn == USART_TX_Sendbox_Cnt)
        	{
        	    memcpy(U_Tx_Buff,CAN_Send_Box[j].Data.Direct_Arr,sizeof(U_Tx_Buff));
				DMA1->LIFCR        |= DMA_LIFCR_CTCIF1;
				DMA1_Stream1->PAR	= (uint32_t) &UART7->TDR;
				DMA1_Stream1->M0AR  = (uint32_t) &U_Tx_Buff[0] ;
				DMA1_Stream1->NDTR  = USART_PROTOCOL_SIZE;
				DMA1_Stream1->CR   |= DMA_SxCR_TCIE;
				DMA1_Stream1->CR   |= DMA_SxCR_EN;
				CAN_Send_Box[j].turn= CAN_BOX_EMPTY;
				++USART_TX_Sendbox_Cnt;
				if(USART_TX_Sendbox_Cnt>=(CAN_RX_INBOX_NUM))	USART_TX_Sendbox_Cnt=0;
        	    break;
        	}

     }
}





/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
