/**
 ******************************************************************************
 * @file           : nrf24_transmit.c
 * @brief          : Функции для передачи данных педалей через NRF24L01
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "nrf24_transmit.h"
#include "NRF24.h"
#include "NRF24_reg_addresses.h"
#include "../RGB/my_fnc_rgb.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* Private defines -----------------------------------------------------------*/
#define PEDAL_TRANSMIT_PERIOD_MS 500  // Период передачи при удержании педали

/* External variables --------------------------------------------------------*/
extern uint8_t dataT[PLD_S];
extern uint32_t nrf_counter;

/* Private variables ---------------------------------------------------------*/
static uint8_t prev_left_state = GPIO_PIN_SET;
static uint8_t prev_right_state = GPIO_PIN_SET;
static uint8_t prev_both_state = GPIO_PIN_SET;
static uint8_t prev_left_right_both_state = GPIO_PIN_SET;

// Таймеры для периодической передачи (в тиках HAL_GetTick())
static uint32_t left_pressed_timer = 0;
static uint32_t right_pressed_timer = 0;
static uint32_t left_right_both_pressed_timer = 0;

/**
 * @brief  Инициализация радиомодуля NRF24L01
 * @param  None
 * @retval None
 */
void nrf24_radio_init(void)
{
  printf("Initializing NRF24L01...\n");

  csn_high();
  ce_high();
  HAL_Delay(5);
  ce_low();

  nrf24_init();
  nrf24_stop_listen();

  nrf24_auto_ack_all(auto_ack); // Включаем автоподтверждение для всех каналов
  nrf24_en_ack_pld(disable);
  nrf24_dpl(disable);

  nrf24_set_crc(en_crc, _1byte); // Включаем CRC для лучшего определения ошибок

  nrf24_tx_pwr(_0dbm);
  nrf24_data_rate(_1mbps);
  nrf24_set_channel(90);
  nrf24_set_addr_width(5);

  nrf24_set_rx_dpl(0, disable);
  nrf24_set_rx_dpl(1, disable);
  nrf24_set_rx_dpl(2, disable);
  nrf24_set_rx_dpl(3, disable);
  nrf24_set_rx_dpl(4, disable);
  nrf24_set_rx_dpl(5, disable);

  nrf24_pipe_pld_size(0, PLD_S);

  nrf24_auto_retr_delay(4);
  nrf24_auto_retr_limit(10);

  // Настройка адреса передачи
  uint8_t tx_addr[5] = {0x45, 0x55, 0x67, 0x10, 0x21};
  nrf24_open_tx_pipe(tx_addr);
  nrf24_open_rx_pipe(0, tx_addr);
  ce_high();

  printf("NRF24L01 initialized successfully\n");
}

/**
 * @brief  Передача данных педалей через NRF24L01
 * @param  left_state: Состояние левой педали (GPIO_PIN_RESET - нажата, GPIO_PIN_SET - отпущена)
 * @param  right_state: Состояние правой педали (GPIO_PIN_RESET - нажата, GPIO_PIN_SET - отпущена)
 * @param  both_state: Состояние кнопки BOTH (GPIO_PIN_RESET - нажата, GPIO_PIN_SET - отпущена)
 * @retval None
 */
void nrf24_transmit_pedal_data(uint8_t left_state, uint8_t right_state, uint8_t both_state)
{
  bool need_transmit = false;
  
  // Определяем состояние одновременного нажатия левой и правой педалей
  uint8_t left_right_both_state = (left_state == GPIO_PIN_RESET && right_state == GPIO_PIN_RESET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
  
  // Проверяем изменения состояний
  bool left_changed = (left_state != prev_left_state);
  bool right_changed = (right_state != prev_right_state);
  bool both_changed = (both_state != prev_both_state);
  bool left_right_both_changed = (left_right_both_state != prev_left_right_both_state);
  
  // ПРИОРИТЕТ ПЕРЕДАЧ:
  // 1. Одновременное нажатие left+right (блокирует отдельные передачи left и right)
  // 2. Отдельные нажатия left или right (только если нет одновременного нажатия)
  // 3. Кнопка BOTH (независимо от других)
  
  // Логика для одновременного нажатия левой и правой педалей (ВЫСШИЙ ПРИОРИТЕТ)
  if (left_right_both_changed) {
    need_transmit = true;
    if (left_right_both_state == GPIO_PIN_RESET) { // Обе педали нажаты
      left_right_both_pressed_timer = HAL_GetTick(); // Запускаем таймер
    }
  } else if (left_right_both_state == GPIO_PIN_RESET) { // Обе педали удерживаются нажатыми
    // Проверяем, прошло ли 500 мс с последней передачи
    if ((HAL_GetTick() - left_right_both_pressed_timer) >= PEDAL_TRANSMIT_PERIOD_MS) {
      need_transmit = true;
      left_right_both_pressed_timer = HAL_GetTick(); // Сбрасываем таймер
    }
  }
  
  // Логика для левой педали (только если правая не нажата одновременно)
  if (!need_transmit && left_changed && left_right_both_state == GPIO_PIN_SET) {
    need_transmit = true;
    if (left_state == GPIO_PIN_RESET) { // Педаль нажата
      left_pressed_timer = HAL_GetTick(); // Запускаем таймер
    }
  } else if (!need_transmit && left_state == GPIO_PIN_RESET && left_right_both_state == GPIO_PIN_SET) { // Педаль удерживается нажатой (но не одновременно с правой)
    // Проверяем, прошло ли 500 мс с последней передачи
    if ((HAL_GetTick() - left_pressed_timer) >= PEDAL_TRANSMIT_PERIOD_MS) {
      need_transmit = true;
      left_pressed_timer = HAL_GetTick(); // Сбрасываем таймер
    }
  }
  
  // Логика для правой педали (только если левая не нажата одновременно)
  if (!need_transmit && right_changed && left_right_both_state == GPIO_PIN_SET) {
    need_transmit = true;
    if (right_state == GPIO_PIN_RESET) { // Педаль нажата
      right_pressed_timer = HAL_GetTick(); // Запускаем таймер
    }
  } else if (!need_transmit && right_state == GPIO_PIN_RESET && left_right_both_state == GPIO_PIN_SET) { // Педаль удерживается нажатой (но не одновременно с левой)
    // Проверяем, прошло ли 500 мс с последней передачи
    if ((HAL_GetTick() - right_pressed_timer) >= PEDAL_TRANSMIT_PERIOD_MS) {
      need_transmit = true;
      right_pressed_timer = HAL_GetTick(); // Сбрасываем таймер
    }
  }
  
  // Логика для кнопки BOTH (только при изменении)
  if (!need_transmit && both_changed) {
    need_transmit = true;
  }

  // Передаем данные, если есть необходимость
  if (need_transmit) {
    // Очищаем буфер
    memset(dataT, 0, sizeof(dataT));

    // Формируем сообщение с состоянием педалей
    char message[PLD_S];
    snprintf(message, sizeof(message), "L:%d,R:%d,B:%d,LR:%d #%lu",
             (left_state == GPIO_PIN_RESET) ? 1 : 0,
             (right_state == GPIO_PIN_RESET) ? 1 : 0,
             (both_state == GPIO_PIN_RESET) ? 1 : 0,
             (left_right_both_state == GPIO_PIN_RESET) ? 1 : 0,
             nrf_counter++);

    // Копируем сообщение в буфер для передачи
    strncpy((char *)dataT, message, PLD_S - 1);
    dataT[PLD_S - 1] = '\0'; // Гарантируем null-terminator

    // Очищаем флаги статуса перед передачей
    uint8_t clear_flags = 0x70; // Очищаем TX_DS, MAX_RT, RX_DR
    nrf24_w_reg(STATUS, &clear_flags, 1);

    // Отправляем данные через NRF24
    uint8_t val = nrf24_transmit(dataT, PLD_S);
    
    // Читаем статус регистр для детальной диагностики
    uint8_t status_reg = nrf24_r_reg(STATUS, 1);

    // Выводим статус передачи через UART
    printf("NRF TX: %s | Status: %d | StatusReg: 0x%02X\n", message, val, status_reg);
    
    // Детальный анализ StatusReg
    bool tx_ds = (status_reg & 0x20) != 0;   // Бит 5 - TX_DS (успешная передача)
    bool max_rt = (status_reg & 0x10) != 0;  // Бит 4 - MAX_RT (достигнуто максимум повторов)
    bool rx_dr = (status_reg & 0x01) != 0;   // Бит 0 - RX_DR (данные получены)
    
    printf("TX_DS:%d MAX_RT:%d RX_DR:%d\n", tx_ds, max_rt, rx_dr);
    
    // Передача не удалась, если:
    // 1. MAX_RT установлен (достигнуто максимум повторов)
    // 2. TX_DS не установлен (передача не подтверждена)
    // 3. Функция nrf24_transmit вернула ошибку
    if (max_rt || !tx_ds || val != 0) {
      blink_red(1); // Мигнуть красным один раз
      printf("TX FAILED! Reason: ");
      if (max_rt) printf("MAX_RT ");
      if (!tx_ds) printf("NO_TX_DS ");
      if (val != 0) printf("FUNC_ERROR ");
      printf("\n");
    }

    // Сохраняем текущее состояние для следующего сравнения
    prev_left_state = left_state;
    prev_right_state = right_state;
    prev_both_state = both_state;
    prev_left_right_both_state = left_right_both_state;
  }
}
