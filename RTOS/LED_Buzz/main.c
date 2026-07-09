/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "string.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <queue.h>
#include <semphr.h>
#include <LED_M.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUZZ_Pin       GPIO_PIN_14
#define BUZZ_GPIO_Port GPIOF
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ETH_TxPacketConfig TxConfig;
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

ETH_HandleTypeDef heth;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

#define ACS_SENS_MV_PER_A   122   // (±3.3V 변종)
static float g_zero_raw = 0;
static float raw_buf[5] = {0};
static uint8_t idx = 0;

static LED_M_Handle_t led_matrix1 = { .spi = SPI1, .cs_port = GPIOD, .cs_pin = (1U << 14) };
static LED_M_Handle_t led_matrix2 = { .spi = SPI4, .cs_port = GPIOE, .cs_pin = (1U << 9) };

static uint8_t matrix_is_on = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void CurrentTask(void *argument);
void Buzz_Task(void *argument);
void LED_M_Task(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <stdio.h>

typedef enum
{
    SYS_IDLE = 0,
    SYS_IDENTIFY,
    SYS_AUTHENTICATE,
    SYS_NEGOTIATE,
    SYS_ACTIVE,
    SYS_FAULT,
    SYS_QUARANTINE
} SystemState_t;

typedef enum
{
    MODULE_NONE = 0,
    MODULE_GENERAL,      // Module A: 일반 배송 모듈
    MODULE_COLD_CHAIN,   // Module B: 냉장 배송 모듈
    MODULE_UNKNOWN       // 등록되지 않았거나 알 수 없는 모듈
} ModuleType_t;

typedef enum
{
    FAULT_NONE = 0,
    FAULT_IDENTIFY_TIMEOUT,
    FAULT_AUTH_FAIL,
    FAULT_AUTH_TIMEOUT,
    FAULT_POWER_REJECT,
    FAULT_NEGOTIATE_TIMEOUT,
    FAULT_POWER_VIOLATION_MAX
} FaultCode_t;

typedef struct
{
    /* System State */
    SystemState_t system_state;
    ModuleType_t module_type;
    FaultCode_t latest_fault;

    uint32_t module_id;      // Identify 응답 Payload의 모듈 고유 식별값

    uint8_t dock_detected;
    uint8_t auth_result;
    uint8_t power_granted;
    uint8_t module_function_enabled;

    /* Driving */
    float target_speed_rpm;
    float current_speed_rpm;
    uint16_t motor_pwm_duty;

    /* Power Policy */
    float requested_power_w;       // Module이 선언한 필요 전력
    float granted_power_w;         // Base가 허용한 최대 전력
    float reported_power_w;        // Module이 CAN으로 보고한 현재 사용 전력
    uint8_t power_violation_count; // 누적 위반 횟수

    /* Module A */
    float pressure_value;

    /* Module B */
    float target_temp_c;
    float current_temp_c;
    uint8_t peltier_pwm;
    uint8_t fan_pwm;

    /* Warning */
    uint8_t warning_flag;
    uint8_t sleep_flag;
} SystemSharedData_t;

/* CAN Queue Handle */
extern QueueHandle_t g_can_rx_queue;
extern QueueHandle_t g_can_tx_queue;

/* UART Queue Handle */
extern QueueHandle_t g_uart_rx_queue;
extern QueueHandle_t g_uart_tx_queue;

/* Shared Data Mutex */
extern SemaphoreHandle_t g_system_data_mutex;

/* Shared System Data */
SystemSharedData_t g_system_data;

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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */

  RCC->AHB1ENR |= (0x01 << 0) | (0x01 << 1) | (0x01 << 2) | (0x01 << 3) | (0x01 << 4) | (0x01 << 5);   /* GPIOA~F clock enable */
  GPIOA->MODER |= (0x03 << (3 * 2));      /* PA3 analog mode */

  RCC->APB2ENR |= (0x01 << 8);            /* ADC1 clock enable */
  ADC->CCR |= (0x01 << 16);

  ADC1->SQR3 |= (0x03 << 0);              /* first conversion = channel 3 */
  ADC1->SMPR2 &= ~(0x07 << (3 * 3));
  ADC1->SMPR2 |=  (0x04 << (3 * 3));       /* sample time, channel 3 */

  ADC1->CR2 |= (0x01 << 0);               /* ADON */

  /* ---- SPI1: PA5=SCK, PA7=MOSI, AF5 (원래 그대로, OSPEEDR 추가 안 함) ---- */
  GPIOA->MODER &= ~((0x03 << (5 * 2)) | (0x03 << (7 * 2)));
  GPIOA->MODER |=  ((0x02 << (5 * 2)) | (0x02 << (7 * 2)));   /* AF mode */
  GPIOA->AFR[0] &= ~((0x0F << (5 * 4)) | (0x0F << (7 * 4)));
  GPIOA->AFR[0] |=  ((0x05 << (5 * 4)) | (0x05 << (7 * 4)));  /* AF5 = SPI1 */

  RCC->APB2ENR |= (0x01 << 12);   /* SPI1 clock enable */

  SPI1->CR1 = 0;
  SPI1->CR1 |= (0x01 << 2);       /* MSTR = master mode */
  SPI1->CR1 |= (0x01 << 9) | (0x01 << 8); /* SSM=1, SSI=1: software NSS, avoid MODF */
  SPI1->CR1 |= (0x04 << 3);       /* BR[2:0] = /32 prescaler */
  SPI1->CR1 |= (0x01 << 6);       /* SPE = enable */

  /* ---- SPI4: PE12=SCK(CLK), PE14=MOSI(DIN), AF5 ---- */
  GPIOE->MODER &= ~((0x03 << (12 * 2)) | (0x03 << (14 * 2)));
  GPIOE->MODER |=  ((0x02 << (12 * 2)) | (0x02 << (14 * 2)));   /* AF mode */
  GPIOE->AFR[1] &= ~((0x0F << ((12 - 8) * 4)) | (0x0F << ((14 - 8) * 4)));
  GPIOE->AFR[1] |=  ((0x05 << ((12 - 8) * 4)) | (0x05 << ((14 - 8) * 4))); /* AF5 = SPI4 */

  /* CS: PE9 as GPIO output */
  GPIOE->MODER &= ~(0x03 << (9 * 2));
  GPIOE->MODER |=  (0x01 << (9 * 2));

  RCC->APB2ENR |= (0x01 << 13);   /* SPI4 clock enable */

  SPI4->CR1 = 0;
  SPI4->CR1 |= (0x01 << 2);       /* MSTR = master mode */
  SPI4->CR1 |= (0x01 << 9) | (0x01 << 8); /* SSM=1, SSI=1: software NSS */
  SPI4->CR1 |= (0x04 << 3);       /* BR = /32 */
  SPI4->CR1 |= (0x01 << 6);       /* SPE = enable */

  /* PD14: CS output for LED_M SPI1 */
  GPIOD->MODER &= ~(0x03 << (14 * 2));
  GPIOD->MODER |=  (0x01 << (14 * 2));   /* General purpose output */

  /* PF14(BUZZ) - GPIOF clock enable already done above, but MX_GPIO_Init() ran
     before that clock was on, so redo the mode setting here */
  GPIOF->MODER &= ~(0x03 << (14 * 2));
  GPIOF->MODER |=  (0x01 << (14 * 2));   /* General purpose output */

  ADC1->CR2 |= ADC_CR2_ADON;
  HAL_Delay(1);   // ADC 안정화 대기 (tSTAB)

  float acc = 0;
  for (int i = 0; i < 100; i++) {
	ADC1->CR2 |= ADC_CR2_SWSTART;
	while (!(ADC1->SR & ADC_SR_EOC));
	acc += ADC1->DR;
	HAL_Delay(10);
  }
  g_zero_raw = acc / 100;
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  xTaskCreate(CurrentTask, "Task current", 1000, NULL, 1, NULL);
  xTaskCreate(Buzz_Task, "Task Buzz", 1000, NULL, 1, NULL);
  xTaskCreate(LED_M_Task, "Task LED_M", 1000, NULL, 1, NULL);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */

  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /*Configure GPIO pin : BUZZ_Pin (PF14) */
  GPIO_InitStruct.Pin = BUZZ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZ_GPIO_Port, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void CurrentTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	uint32_t acc = 0;
	const int N = 100;
	for (int i = 0; i < N; i++) {
	  ADC1->CR2 |= ADC_CR2_SWSTART;
	  while (!(ADC1->SR & ADC_SR_EOC));
	  acc += ADC1->DR;
	}
	float raw = (float)acc / (float)N;
	raw_buf[idx] = raw;
	idx = (idx + 1) % 5;

	float raw_avg = 0.0f;

	for(int i=0;i<5;i++)
	    raw_avg += raw_buf[i];

	raw_avg /= 5.0f;

	float d_raw =  -raw_avg + g_zero_raw;
	if (d_raw >= -4 && d_raw <= 4)
	{
		d_raw = 0;
	}

	float current_mA =
	    (d_raw * 3300.0f * 1000.0f) /
	    (4095.0f * ACS_SENS_MV_PER_A);

	int current10 = (int)(current_mA * 10.0f);
	int raw10 = (int)(raw_avg  * 10.0f);
	int d10   = (int)(d_raw * 10.0f);

	char msg[64];


	//int len = snprintf(msg, sizeof(msg), "raw=%d.%d d=%d.%d I=%d.%d mA\r\n", raw10 / 10, abs(raw10 % 10), d10 / 10, abs(d10 % 10), current10 / 10, abs(current10 % 10));
	int raw_i = raw10 / 10;
	int raw_f = raw10 % 10;
	if (raw_f < 0) raw_f = -raw_f;

	int d_i = d10 / 10;
	int d_f = d10 % 10;
	if (d_f < 0) d_f = -d_f;

	int cur_i = current10 / 10;
	int cur_f = current10 % 10;
	if (cur_f < 0) cur_f = -cur_f;

	int len = snprintf(msg, sizeof(msg), "raw=%d.%d d=%d.%d I=%d.%d mA\r\n", raw_i, raw_f, d_i, d_f, cur_i, cur_f);

	HAL_UART_Transmit(&huart3, (uint8_t *)msg, len, HAL_MAX_DELAY);
	HAL_GPIO_TogglePin(GPIOB, LD1_Pin);
	osDelay(200);
  }
  /* USER CODE END 5 */
}

void Buzz_Task(void *argument)
{
  for (;;)
  {
    uint8_t get_sleep_flag = g_system_data.sleep_flag;
    get_sleep_flag = 5; /* debug override */

    if (get_sleep_flag >= 3)
    {
      HAL_GPIO_TogglePin(GPIOB, LD3_Pin);
      HAL_GPIO_TogglePin(GPIOB, LD2_Pin);

      uint32_t buzz_on_ms;
      uint32_t gap_ms;

      switch (get_sleep_flag)
      {
        case 3:  buzz_on_ms = 50; gap_ms = 350; break;
        case 4:  buzz_on_ms = 50; gap_ms = 150; break;
        case 5:
        default: buzz_on_ms = 50; gap_ms = 50;  break;
      }

      HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
      osDelay(buzz_on_ms);
      HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
      osDelay(gap_ms);
    }
    else
    {
      HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
      osDelay(50);
    }
  }
}

void LED_M_Task(void *argument)
{
  LED_M_Init(&led_matrix1);
  LED_M_Init(&led_matrix2);

  uint8_t led_is_on = 0;

  for (;;)
  {
    uint8_t get_sleep_flag = g_system_data.sleep_flag;
    get_sleep_flag = 5; /* debug override */

    if (get_sleep_flag >= 4)
    {
      uint32_t on_ms, off_ms;

      if (get_sleep_flag >= 5)
      {
        on_ms = 300;
        off_ms = 300;
      }
      else /* flag == 4 */
      {
        on_ms = 73;
        off_ms = 73;
      }

      if (get_sleep_flag >= 5)
      {
        LED_M_DisplayPattern(&led_matrix1, LED_M_PATTERN_WARNING_5_L);
        LED_M_DisplayPattern(&led_matrix2, LED_M_PATTERN_WARNING_5_R);
      }
      else
      {
        LED_M_DisplayPattern(&led_matrix1, LED_M_PATTERN_WARNING_4);
        LED_M_DisplayPattern(&led_matrix2, LED_M_PATTERN_WARNING_4);
      }
      led_is_on = 1;
      osDelay(on_ms);

      LED_M_ClearDisplay(&led_matrix1);
      LED_M_ClearDisplay(&led_matrix2);
      led_is_on = 0;
      osDelay(off_ms);
    }
    else
    {
      if (led_is_on)
      {
        LED_M_ClearDisplay(&led_matrix1);
        LED_M_ClearDisplay(&led_matrix2);
        led_is_on = 0;
      }
      osDelay(50);
    }
  }
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	osDelay(1);
  }
  /* USER CODE END 5 */
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
#ifdef USE_FULL_ASSERT
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
