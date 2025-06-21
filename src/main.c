/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Головний файл програми
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * Це програмне забезпечення ліцензовано згідно з умовами, які можна знайти у файлі LICENSE
 * у кореневому каталозі цього програмного забезпечення.
 * Якщо файл LICENSE відсутній, програмне забезпечення надається "ЯК Є".
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "USER_DEFINES\user_defines.h"
#include "RGB/my_fnc_rgb.h"
#include "RGB/ARGB.h"
#include "BUZZER/buzzer.h" // підключення заголовочного файлу
#include "NRF2_4Mhz/NRF24.h"
#include "ADC/adc.h"
#include "USER_DEFINES/print_log.h"
#include "USER_DEFINES\power_info.h"
#include <math.h>
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

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim4_ch2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Приватні змінні */
// змінні для NRF модуля
char str1[64] = {0};

uint8_t buf1[20] = {0};

// uint8_t dt_reg=0;

uint8_t retr_cnt, dt;

uint16_t i = 1, retr_cnt_full;

/* USER CODE END PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
// if (htim->Instance == TIM1) {
//  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8); // Змінити стан піна PB0

//}
//}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Визначення станів живлення
PowerInfo power_info;
// Прототип функції
void update_power_state(bool is_external_power_present);
/* USER CODE END 0 */

/**
 * @brief  Точка входу в програму
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Налаштування MCU --------------------------------------------------------*/

  /* Скидання всіх периферійних пристроїв, ініціалізація Flash та Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Налаштування системного тактування */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Ініціалізація всіх налаштованих периферійних пристроїв */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
    HAL_TIM_Base_Start_IT(&htim1); // Запуск таймера у режимі переривань
  /* USER CODE END 2 */
    ARGB_Init();                   // Ініціалізація ARGB
  NRF24_ini();                   // Ініціалізація NRF24L01

  ARGB_FillRGB(255, 0, 0); // Заповнення ARGB червоним кольором
  ARGB_Show();
  BUZZER_Go(TBUZ_100, TICK_1);
  printf("Im radio pedals v2.0!\n"); // Вивід повідомлення в UART
  HAL_Delay(500);                    // Затримка 0.5 секунда
  printf("NRF24L01 test\n");         // Вивід повідомлення в UART
  NRF24_DebugInfo();                 // Вивід інформації про NRF24
  blink_green_with_period(1, 100);   // Блимати зеленим 1 разів з періодом 100 мс
  print_log();                       // Вивід інформації про стан живлення
  /* Нескінченний цикл */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    blink_tick(); // Виклик функції для керування блиманням LED
    // Зчитування напруги на зовнішній батареї через дільник
    float ext_voltage = adc(ADC_CHANNEL_0, EXT_BATTERY_DIVIDER_COEFF, "BAT_EXTERNAL");

    // Якщо напруга на "ADC" більше порогу — вважаємо, що підключена зовнішня батарея
    // усі порогові напруги записані в файлі src\USER_DEFINES\user_defines.h
    // і можуть корегуватися при налаштуванні після прошивки плати для точнішого визначення напруги
    bool ext_power = (ext_voltage > EXT_POWER_PRESENT_THRESHOLD);

    update_power_state(ext_power);

  if (power_info.power_state == POWER_EXTERNAL) {
      // Дії при живленні від зовнішньої батареї
      HAL_GPIO_WritePin(GPIOB, LEFT_REM_Pin, HAL_GPIO_ReadPin(GPIOB, LEFT_Pin));//
      HAL_GPIO_WritePin(GPIOB, RIGHT_REM_Pin, HAL_GPIO_ReadPin(GPIOB, RIGHT_Pin));
  } else if (power_info.power_state == POWER_INTERNAL_BATTERY) {
      // Дії при живленні від внутрішньої батареї
      if (HAL_GPIO_ReadPin(GPIOB, LEFT_Pin) != GPIO_PIN_RESET ||
          HAL_GPIO_ReadPin(GPIOB, RIGHT_Pin) != GPIO_PIN_RESET) {
          HAL_GPIO_WritePin(GPIOB, LEFT_REM_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOB, RIGHT_REM_Pin, GPIO_PIN_RESET);
      }
  }
  
  








  }
}

/**
 * @brief Налаштування системного тактування
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Ініціалізація генераторів тактових імпульсів згідно з параметрами RCC_OscInitTypeDef */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Ініціалізація шин CPU, AHB та APB */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief Ініціалізація ADC1
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

  /** Загальна конфігурація */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Налаштування регулярного каналу */
  sConfig.Channel = ADC_CHANNEL_1;
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
 * @brief Ініціалізація SPI2
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* Конфігурація SPI2 */
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */
}

/**
 * @brief Ініціалізація TIM1
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 56;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 236;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
}

void update_power_state(bool is_external_power_present)
{
  static PowerState prev_power_state = POWER_INTERNAL_BATTERY;
  static BatteryState prev_battery_state = BATTERY_GOOD;
  static float prev_voltage = 0.0f;

  if (is_external_power_present)
  {
    power_info.power_state = POWER_EXTERNAL;
    float ext_voltage = adc(ADC_CHANNEL_0, EXT_BATTERY_DIVIDER_COEFF, "BAT_EXTERNAL");
    power_info.voltage = ext_voltage;
    if (ext_voltage > EXT_BATTERY_GOOD_THRESHOLD)
      power_info.battery_state = BATTERY_GOOD;
    else if (ext_voltage > EXT_BATTERY_MEDIUM_THRESHOLD)
      power_info.battery_state = BATTERY_MEDIUM;
    else if (ext_voltage > EXT_BATTERY_BAD_THRESHOLD)
      power_info.battery_state = BATTERY_BAD;
    else
      power_info.battery_state = BATTERY_VERY_BAD;
  }
  else
  {
    power_info.power_state = POWER_INTERNAL_BATTERY;
    float int_voltage = adc(ADC_CHANNEL_1, INT_BATTERY_DIVIDER_COEFF, "BAT_ADC");
    power_info.voltage = int_voltage;
    if (int_voltage > INT_BATTERY_GOOD_THRESHOLD)
      power_info.battery_state = BATTERY_GOOD;
    else if (int_voltage > INT_BATTERY_MEDIUM_THRESHOLD)
      power_info.battery_state = BATTERY_MEDIUM;
    else if (int_voltage > INT_BATTERY_BAD_THRESHOLD)
      power_info.battery_state = BATTERY_BAD;
    else
      power_info.battery_state = BATTERY_VERY_BAD;
  }

  // Логирование только при изменении состояния или если напряжение изменилось на >= 0.1В
  if (power_info.power_state != prev_power_state ||
      power_info.battery_state != prev_battery_state ||
      fabsf(power_info.voltage - prev_voltage) >= 0.1f)
  {
    print_log();
    prev_power_state = power_info.power_state;
    prev_battery_state = power_info.battery_state;
    prev_voltage = power_info.voltage;
  }
}

/**
 * @brief Ініціалізація TIM4
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);
}

/**
 * @brief Ініціалізація USART1
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
 * @brief Увімкнення тактування контролера DMA
 */
static void MX_DMA_Init(void)
{

  /* Увімкнення тактування DMA контролера */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* Ініціалізація переривань DMA */
  /* Налаштування переривання DMA1_Channel4_IRQn */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}

/**
 * @brief Ініціалізація GPIO
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* Увімкнення тактування портів GPIO */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Встановити рівень виходу для пінів CSN та CE */
  HAL_GPIO_WritePin(GPIOB, CSN_Pin | CE_Pin, GPIO_PIN_RESET);

  /* Встановити рівень виходу для піна BUZZER */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

  // Налаштування PA0 та PA1 як аналогових входів
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Налаштування інших пінів як входів
  GPIO_InitStruct.Pin = LEFT_Pin | RIGHT_Pin | BOTH_Pin | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Налаштування пінів керування драйверами педалей як виходів
  GPIO_InitStruct.Pin = RIGHT_REM_Pin | LEFT_REM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Налаштування пінів : CSN_Pin CE_Pin */
  GPIO_InitStruct.Pin = CSN_Pin | CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Налаштування піна : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Ця функція виконується у випадку виникнення помилки.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* Користувач може додати свою реалізацію для повідомлення про стан помилки HAL */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Повідомляє ім'я вихідного файлу та номер рядка,
 *         де виникла помилка assert_param.
 * @param  file: вказівник на ім'я вихідного файлу
 * @param  line: номер рядка, де виникла помилка
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* Користувач може додати свою реалізацію для повідомлення імені файлу та номера рядка,
     наприклад: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
