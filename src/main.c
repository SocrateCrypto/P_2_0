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
#include "NRF2_4Mhz/NRF24_reg_addresses.h"
#include "NRF2_4Mhz/nrf24_transmit.h"
#include <string.h>
#include <stdbool.h>
#include "ADC/adc.h"
#include "RGB/my_fnc_rgb.h"
#include "USER_DEFINES/print_log.h"
#include "USER_DEFINES\power_info.h"
#include "Timer/timer.h"
#include "Debounce/anti_debounce.h"
#include <math.h>
#include "USER_DEFINES/radio_settings.h"
static uint8_t rx_radio_addr[5] = RADIO_ADDRESS;
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

uint8_t buf1[20] = {"maza facking"}; // буфер для відправки даних через NRF24L01

// uint8_t dt_reg=0;

uint8_t retr_cnt, dt;

uint16_t i = 1, retr_cnt_full;

// NRF24 змінні
#define PLD_S 32
uint8_t tx_addr[5] = {0x45, 0x55, 0x67, 0x10, 0x21};//
uint8_t dataT[PLD_S];
uint32_t nrf_counter = 0; // Счетчик сообщений NRF

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

// Функції-обгортки для таймерів
void buzzer_action(void);
void blink_green_action(void);

// Переменные для управления привязкой
typedef enum {
    TRANSMITTER_NORMAL = 0,    // Обычный режим работы
    TRANSMITTER_BINDING        // Режим привязки
} transmitter_state_t;

static transmitter_state_t transmitter_state = TRANSMITTER_NORMAL;
static uint32_t both_button_press_start = 0;  // Время начала нажатия кнопки both
static uint8_t both_button_was_pressed = 0;   // Флаг что кнопка была нажата
static uint32_t binding_start_time = 0;       // Время начала режима привязки

#define BOTH_BUTTON_HOLD_TIME 4000    // 5 секунд удержания для входа в привязку
#define BINDING_TIMEOUT 30000         // 30 секунд таймаут привязки
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

  ARGB_FillRGB(255, 0, 0); // Заповнення ARGB червоним кольором
  ARGB_Show();
  BUZZER_Go(TBUZ_100, TICK_1);
  printf("Im radio pedals v2.0!\n"); // Вивід повідомлення в UART
  HAL_Delay(500);                    // Затримка 0.5 секунда
  printf("NRF24L01 test\n");         // Вивід повідомлення в UART

  // Ініціалізація NRF24 (тільки якщо потрібно)
  nrf24_radio_init();
  // Вивід інформації про NRF24
  blink_green_with_period(1, 100); // Блимати зеленим 1 разів з періодом 100 мс
  print_log();                     // Вивід інформації про стан живлення
  /* Нескінченний цикл */
  /* USER CODE BEGIN WHILE */

  // Глобальный тикер для таймера
  TimerTicker buzzer_ticker = {0};

  // Глобальний тикер для blink_green_with_period
  TimerTicker blink_green_ticker = {0};

  DebounceButton left_btn, right_btn, both_btn;
  // Ініціалізація кнопок з антидребезгом

  debounce_init(&left_btn, GPIOB, LEFT_Pin, 10); // 8 циклов для устойчивости
  debounce_init(&right_btn, GPIOB, RIGHT_Pin, 10);
  debounce_init(&both_btn, GPIOB, BOTH_Pin, 10); // Кнопка BOTH

  while (1)
  {

    blink_tick(); // Виклик функції тікера для керування блиманням LED
    // Зчитування напруги на зовнішній батареї через дільник
    float ext_voltage = adc(ADC_CHANNEL_0, EXT_BATTERY_DIVIDER_COEFF, "BAT_EXTERNAL");

    // Якщо напруга на "ADC" більше порогу — вважаємо, що підключена зовнішня батарея
    // усі порогові напруги записані в файлі src\USER_DEFINES\user_defines.h
    // і можуть корегуватися при налаштуванні  плати для точнішого визначення напруги
    bool ext_power = (ext_voltage > EXT_POWER_PRESENT_THRESHOLD);

    update_power_state(ext_power);
    uint8_t left_state = debounce_read(&left_btn);   // антидребезг
    uint8_t right_state = debounce_read(&right_btn); // антидребезг
    uint8_t both_state = debounce_read(&both_btn);   // антидребезг кнопки BOTH
    if (power_info.power_state == POWER_EXTERNAL)
    {
      // Дії при живленні від зовнішньої батареї
      // Інвертуємо вихід який йде на основну плату , якщо його стан збігається зі станом входу.
      // Це потрібно, тому що педаль (кнопка) притягує вхід до землі (лог. 0),
      // а у відпущеному стані вхід підтягнутий до плюса (лог. 1).
      // Сигнал передається на оптопару, де потрібна інверсія.

      if (left_state == HAL_GPIO_ReadPin(GPIOB, LEFT_REM_Pin))
      {
        HAL_GPIO_WritePin(GPIOB, LEFT_REM_Pin, !left_state);

        printf("Left REM Pin: %d\n", HAL_GPIO_ReadPin(GPIOB, LEFT_REM_Pin));
      }
      if (right_state == HAL_GPIO_ReadPin(GPIOB, RIGHT_REM_Pin))
      {
        HAL_GPIO_WritePin(GPIOB, RIGHT_REM_Pin, !right_state);
        printf("Right REM Pin: %d\n", HAL_GPIO_ReadPin(GPIOB, RIGHT_REM_Pin));
      }
    }
    else if (power_info.power_state == POWER_INTERNAL_BATTERY)
    {
      // Дії при живленні від внутрішньої батареї - використовуємо радіопередачу
      // при внутрішній батареї лінія LEFT_REM RIGHT_REM не повинна працювати і
      // на всяк випадок перевіримо чи вона в стані "нуль" якщо ні то притисткаємо
      if (HAL_GPIO_ReadPin(GPIOB, LEFT_REM_Pin) != GPIO_PIN_RESET)
      {
        HAL_GPIO_WritePin(GPIOB, LEFT_REM_Pin, GPIO_PIN_RESET);
        printf("Left REM Pin: %d\n", HAL_GPIO_ReadPin(GPIOB, LEFT_REM_Pin));
      }
      if (HAL_GPIO_ReadPin(GPIOB, RIGHT_REM_Pin) != GPIO_PIN_RESET)
      {
        HAL_GPIO_WritePin(GPIOB, RIGHT_REM_Pin, GPIO_PIN_RESET);
        printf("Right REM Pin: %d\n", HAL_GPIO_ReadPin(GPIOB, RIGHT_REM_Pin));
      }

      // ============================================================
      // ЛОГИКА УПРАВЛЕНИЯ ПРИВЯЗКОЙ
      // ============================================================
      uint32_t current_time = HAL_GetTick();
      
      // Проверяем логику удержания кнопки both для входа в режим привязки
      if (transmitter_state == TRANSMITTER_NORMAL) {
        if (both_state == GPIO_PIN_RESET) { // Кнопка both нажата
          if (!both_button_was_pressed) {
            // Только что нажали кнопку - запоминаем время
            both_button_press_start = current_time;
            both_button_was_pressed = 1;
          } else {
            // Кнопка удерживается - проверяем время
            if (current_time - both_button_press_start >= BOTH_BUTTON_HOLD_TIME) {
              // Удерживали 5 секунд - входим в режим привязки
              transmitter_state = TRANSMITTER_BINDING;
              binding_start_time = current_time;
              nrf24_enter_binding_mode();
              printf("ENTERING BINDING MODE - Hold 5 sec detected!\r\n");
              BUZZER_Go(TBUZ_50, TICK_1);
              // Начинаем постоянное мигание в режиме привязки (желтый, 500ms)
              start_binding_blink(500);
              
              both_button_was_pressed = 0; // Сбрасываем флаг
            }
          }
        } else {
          // Кнопка both отпущена
          both_button_was_pressed = 0;
        }
        
        // В обычном режиме передаем данные педалей
        nrf24_transmit_pedal_data(left_state, right_state, both_state);
        
        // --- Автоматическое переключение в RX для приёма команд ---
        nrf24_stop_listen(); // Гарантируем выход из RX перед сменой режима
        ce_low();
        nrf24_flush_rx(); // <--- ОЧИСТКА RX FIFO ПЕРЕД ПРИЁМОМ
        nrf24_open_rx_pipe(0, rx_radio_addr); // Используем адрес из дефайна
        nrf24_listen();
        ce_high();
        HAL_Delay(3); // Оптимальна затримка для прийому (підібрати при необхідності)
        
        char rx_buf[32] = {0};
        static char last_rx_cmd[32] = {0}; // Для фильтрации повторов
        static uint32_t last_long_beep_time = 0; // Время последнего срабатывания long_beep
        // Обрабатываем все команды, которые накопились в RX FIFO
        while (nrf24_data_available()) {
            if (nrf24_receive_command(rx_buf, sizeof(rx_buf))) {
                // Проверка на переполнение RX FIFO (ошибка приёма)
                if (nrf24_rx_fifo_full()) {
                    printf("[NRF RX] RX FIFO FULL! Flushing FIFO...\r\n");
                    nrf24_flush_rx();
                }
                uint32_t now = HAL_GetTick();
                if (strcmp(rx_buf, last_rx_cmd) != 0) {
                    printf("[NRF RX] Received command: '%s'\r\n", rx_buf);
                    if (strcmp(rx_buf, "angle_agiust_mode") == 0) {
                        printf("[NRF RX] angle_agiust_mode command received\r\n");
                        BUZZER_Go(TBUZ_50, TICK_10);
                        blink_blue_with_period(100, 100);
                    } else if (strcmp(rx_buf, "angle_agiust_exit") == 0) {
                        printf("[NRF RX] angle_agiust_exit command received\r\n");
                        BUZZER_Go(TBUZ_50, TICK_1);
                        blink_red_with_period(2, 100);
                        blink_blue_with_period(1, 100);
                    } else if (strcmp(rx_buf, "long_beep") == 0) {
                        if (now - last_long_beep_time > 1000) {
                            printf("[NRF RX] long_beep command received\r\n");
                            BUZZER_Go(TBUZ_1000, TICK_1); // 1 секунда (20*50мс)
                            last_long_beep_time = now;
                        } else {
                            printf("[NRF RX] long_beep ignored (debounce)\r\n");
                        }
                    }
                    strncpy(last_rx_cmd, rx_buf, sizeof(last_rx_cmd)-1);
                    last_rx_cmd[sizeof(last_rx_cmd)-1] = '\0';
                } else {
                    // Повтор команди, игнорируем
                    if (strcmp(rx_buf, "long_beep") == 0) {
                        // Для long_beep — фильтр по времени
                        uint32_t now = HAL_GetTick();
                        if (now - last_long_beep_time > 1000) {
                            printf("[NRF RX] long_beep command received (after timeout)\r\n");
                            BUZZER_Go(TBUZ_1000, TICK_1); // 1 секунда (20*50мс)
                            last_long_beep_time = now;
                        } else {
                            printf("[NRF RX] Duplicate long_beep ignored (debounce)\r\n");
                        }
                    } else {
                        printf("[NRF RX] Duplicate command '%s' ignored\r\n", rx_buf);
                    }
                }
            }
        }
        nrf24_stop_listen(); // Возвращаемся в режим передачи
        ce_low();
        nrf24_open_tx_pipe(rx_radio_addr); // Для возврата в TX тоже используем этот адрес
        ce_high();
        
      } else if (transmitter_state == TRANSMITTER_BINDING) {
        // В режиме привязки
        printf("BINDING MODE ACTIVE - binding_mode=%d, sending bind packets...\r\n", nrf24_is_binding_mode());
        
        // Отправляем пакеты привязки
        nrf24_binding_loop();
        
        // Проверяем, не вышли ли мы из режима привязки (после успешного получения BIND_OK)
        if (!nrf24_is_binding_mode()) {
          printf("*** BINDING SUCCESSFUL! Exiting binding mode ***\r\n");
          transmitter_state = TRANSMITTER_NORMAL;
          
          // Останавливаем постоянное мигание и показываем успех
          stop_binding_blink();
          blink_green_with_period(3, 100); // Индикация успешной привязки
        }
        // Проверяем таймаут привязки
        else if (current_time - binding_start_time >= BINDING_TIMEOUT) {
          printf("BINDING TIMEOUT - exiting binding mode\r\n");
          transmitter_state = TRANSMITTER_NORMAL;
          nrf24_exit_binding_mode();
          
          // Останавлием постоянное мигание и показываем ошибку
          stop_binding_blink();
          blink_red_with_period(5, 50); // Индикация таймаута
        }
      }
    }
    if (power_info.battery_state == BATTERY_GOOD)
    {
      // Дії при хорошому рівні заряду батареї
    }
    else if (power_info.battery_state == BATTERY_MEDIUM)
    {
      // Вызов BUZZER_Go каждые 300 секунд
      call_every_seconds(&buzzer_ticker, 300, buzzer_action);
    }
    else if (power_info.battery_state == BATTERY_BAD || power_info.battery_state == BATTERY_VERY_BAD)
    {
      // Дії при поганому або дуже поганому рівні заряду батареї
      // Вызов blink_green_with_period каждые 30 секунд
      call_every_seconds(&blink_green_ticker, 30, blink_green_action); //
    }
    else
    {
      // Невідомий стан батареї
      //  вивести повідомлення про помилку або попередження
      printf("Unknown battery state!\n");
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
  GPIO_InitStruct.Pull = GPIO_PULLUP; // Використовуємо підтяжку до живлення
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

// Функція-обёртка для BUZZER_Go
void buzzer_action()
{
  BUZZER_Go(TBUZ_50, TICK_2);
}

// Функція-обёртка для blink_green_with_period
void blink_green_action()
{
  blink_red_with_period(2, 50);
  BUZZER_Go(TBUZ_200, TICK_1);
}

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
