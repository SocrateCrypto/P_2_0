/**
 ******************************************************************************
 * @file           : nrf24_transmit.c
 * @brief          : Функции для передачи данных педалей через NRF24L01
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "nrf24_transmit.h"
#include "NRF24.h"
#include <string.h>  // Для memcpy
#include "NRF24_reg_addresses.h"
#include "../RGB/my_fnc_rgb.h"
#include <stdio.h>
#include <string.h>
#include "main.h" // Для HAL_GetTick()
#include <stdbool.h>
#include "BUZZER/buzzer.h" // підключення заголовочного файлу
#include "../USER_DEFINES/radio_settings.h" // Пользовательские настройки радиомодуля
/* Private defines -----------------------------------------------------------*/
#define PEDAL_TRANSMIT_PERIOD_MS 250  // Период передачи при удержании педали

/* Адреса для привязки и работы */
static uint8_t my_addr[5] = RADIO_ADDRESS;      // Уникальный адрес этого пульта
static uint8_t bind_addr[5] = BIND_ADDRESS;     // Стандартный адрес для привязки
static uint8_t work_addr[5] = RADIO_ADDRESS;    // Текущий рабочий адрес

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

// Переменные для режима привязки
static uint8_t binding_mode = 0;      // Флаг режима привязки
static uint32_t last_bind_send = 0;   // Время последней отправки пакета привязки
static uint32_t binding_start_time = 0; // Время начала режима привязки

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

  nrf24_auto_ack_all(1); // Включаем автоподтверждение для всех каналов
  nrf24_en_ack_pld(disable);
  nrf24_dpl(disable);

  nrf24_set_crc(1, 0); // Включаем CRC для лучшего определения ошибок

  nrf24_tx_pwr(_0dbm); // Устанавливаем максимальную мощность передачи
  nrf24_data_rate(_250kbps);
  nrf24_set_channel(76);
  nrf24_set_addr_width(5);

  nrf24_set_rx_dpl(0, disable);
  nrf24_set_rx_dpl(1, disable);
  nrf24_set_rx_dpl(2, disable);
  nrf24_set_rx_dpl(3, disable);
  nrf24_set_rx_dpl(4, disable);
  nrf24_set_rx_dpl(5, disable);

  nrf24_pipe_pld_size(0, PLD_S);

  nrf24_auto_retr_delay(15);
  nrf24_auto_retr_limit(15); // Максимум 15 попыток передачи

  // Настройка адреса передачи (используем рабочий адрес по умолчанию)
  nrf24_open_tx_pipe(work_addr);
  nrf24_open_rx_pipe(0, work_addr);
  ce_high();

  printf("NRF24L01 initialized successfully\n");
  printf("Working address: 0x%02X%02X%02X%02X%02X\n", 
         work_addr[0], work_addr[1], work_addr[2], work_addr[3], work_addr[4]);
  printf("My unique address: 0x%02X%02X%02X%02X%02X\n",
         my_addr[0], my_addr[1], my_addr[2], my_addr[3], my_addr[4]);
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
  // Проверяем, что мы не в режиме привязки
  if (binding_mode) {
    printf("WARNING: nrf24_transmit_pedal_data called while in binding mode! Ignoring.\r\n");
    return;
  }
  
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
     // blink_red_with_period(1, 30); // Блимати зеленим 1 разів з періодом 100 мс // Мигнуть красным один раз
      printf("TX FAILED! Reason: ");
      if (max_rt) {
        printf("MAX_RT ");
        blink_red_with_period(1, 30);
      }
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

/* ============================================================================
 * ФУНКЦИИ ПРИВЯЗКИ ПЕРЕДАТЧИКА
 * ============================================================================ */

/**
 * @brief Расчет контрольной суммы для пакета привязки
 * @param packet Указатель на пакет привязки
 * @retval Контрольная сумма
 */
static uint8_t calculate_bind_checksum(const bind_packet_t* packet) {
    uint8_t checksum = packet->packet_type;
    for(int i = 0; i < 5; i++) {
        checksum += packet->remote_address[i];
    }
    return checksum;
}

/**
 * @brief Вход в режим привязки передатчика
 * @retval None
 */
void nrf24_enter_binding_mode(void) {
    binding_mode = 1;
    binding_start_time = HAL_GetTick(); // Запоминаем время начала привязки
    last_bind_send = 0; // Сбрасываем таймер для немедленной отправки
    
    printf("Transmitter entering binding mode...\r\n");
    printf("Sending bind packets to address: 0x%02X%02X%02X%02X%02X\r\n",
           bind_addr[0], bind_addr[1], bind_addr[2], bind_addr[3], bind_addr[4]);
    
    // Переключаемся на адрес привязки
    nrf24_stop_listen();
    ce_low();
    
    // Включаем auto ACK для привязки
    nrf24_auto_ack_all(1);
    
    // Очищаем буферы
    nrf24_flush_tx();
    nrf24_flush_rx();
    
    // Настраиваем bind-адрес для TX и RX
    nrf24_open_tx_pipe(bind_addr);
    nrf24_open_rx_pipe(0, bind_addr);
    
    // Очищаем флаги статуса
    uint8_t clear_flags = 0x70; // Очищаем TX_DS, MAX_RT, RX_DR
    nrf24_w_reg(STATUS, &clear_flags, 1);
    
    ce_high();
    
    printf("Binding mode initialized - TX/RX cycling enabled\r\n");
}

/**
 * @brief Выход из режима привязки передатчика
 * @retval None
 */
void nrf24_exit_binding_mode(void) {
    binding_mode = 0;
    
    printf("Transmitter exiting binding mode\r\n");
    
    // Обновляем рабочий адрес на новый адрес привязки
    memcpy(work_addr, my_addr, 5);
    
    // Возвращаемся на НОВЫЙ рабочий адрес (после привязки)
    nrf24_stop_listen();
    ce_low();
    
    // Включаем обратно auto ACK для нормальной работы
    nrf24_auto_ack_all(1);
    
    nrf24_open_tx_pipe(work_addr);
    nrf24_open_rx_pipe(0, work_addr);
    
    // Очищаем буферы и флаги
    uint8_t clear_flags = 0x70;
    nrf24_w_reg(STATUS, &clear_flags, 1);
    
    ce_high();
    // Убираем звуковой сигнал - он уже есть в nrf24_check_bind_response()
    printf("Returned to NEW working address: 0x%02X%02X%02X%02X%02X (Auto ACK enabled)\r\n",
           work_addr[0], work_addr[1], work_addr[2], work_addr[3], work_addr[4]);
}

/**
 * @brief Проверка режима привязки
 * @retval 1 - в режиме привязки, 0 - в обычном режиме
 */
uint8_t nrf24_is_binding_mode(void) {
    return binding_mode;
}

/**
 * @brief Отправка пакета привязки
 * @retval None
 */
void nrf24_send_bind_packet(void) {
    if (!binding_mode) {
        printf("ERROR: nrf24_send_bind_packet called but binding_mode is FALSE!\r\n");
        return; // Не в режиме привязки
    }
    
    bind_packet_t bind_packet;
    
    // Заполняем пакет
    bind_packet.packet_type = BIND_PACKET_TYPE;
    memcpy(bind_packet.remote_address, my_addr, 5);
    bind_packet.checksum = calculate_bind_checksum(&bind_packet);
    
    printf("Creating bind packet: type=0x%02X, addr=0x%02X%02X%02X%02X%02X, checksum=0x%02X\r\n",
           bind_packet.packet_type,
           bind_packet.remote_address[0], bind_packet.remote_address[1], 
           bind_packet.remote_address[2], bind_packet.remote_address[3], bind_packet.remote_address[4],
           bind_packet.checksum);
    
    // Убеждаемся, что мы в режиме передачи
    nrf24_stop_listen();
    ce_low();
    
    // Устанавливаем bind-адрес для передачи
    nrf24_open_tx_pipe(bind_addr);
    
    // Очищаем буферы перед отправкой
    nrf24_flush_tx();
    
    // Очищаем флаги статуса перед передачей
    uint8_t clear_flags = 0x70;
    nrf24_w_reg(STATUS, &clear_flags, 1);
    
    // Отправляем пакет привязки
    uint8_t result = nrf24_transmit((uint8_t*)&bind_packet, sizeof(bind_packet));
    
    // Читаем статус для диагностики
    uint8_t status_reg = nrf24_r_reg(STATUS, 1);
    bool tx_ds = (status_reg & 0x20) != 0;
    bool max_rt = (status_reg & 0x10) != 0;
    
    printf("Bind packet sent | Result: %d | TX_DS:%d MAX_RT:%d\r\n", result, tx_ds, max_rt);
    
    if (max_rt) {
        printf("Bind packet: Max retries reached\r\n");
        blink_red_with_period(1, 50);
    } else if (tx_ds) {
        printf("Bind packet: Transmission successful!\r\n");
        blink_green_with_period(1, 50);
    }
}

/**
 * @brief Функция цикла привязки (вызывается из основного цикла)
 * @retval None
 */
void nrf24_binding_loop(void) {
    static uint32_t last_bind_cycle_time = 0;
    static uint32_t tx_send_time = 0;
    static uint8_t binding_state = 0; // 0 - готов к TX, 1 - отправил TX, ждем в RX
    uint32_t current_time = HAL_GetTick();
    
    if (!binding_mode) {
        return;
    }
    
    // Проверяем таймаут привязки
    if ((current_time - binding_start_time) >= BIND_TIMEOUT_MS) {
        printf("Bind timeout reached! Reloading radio module and exiting binding mode...\r\n");
        
        // Перезагружаем радиомодуль
        nrf24_radio_init();
        BUZZER_Go(TBUZ_100, TICK_1);
        // Выходим из режима привязки
        nrf24_exit_binding_mode();
        return;
    }
    
    // Цикл TX-RX каждые BIND_TX_INTERVAL_MS миллисекунд
    if (binding_state == 0) {
        // Состояние: готов к отправке TX
        if ((current_time - last_bind_cycle_time) >= BIND_TX_INTERVAL_MS) {
            printf("BIND: Sending TX packet...\r\n");
            
            // Отправляем bind пакет
            nrf24_send_bind_packet();
            
            // КРИТИЧЕСКОЕ ИСПРАВЛЕНИЕ: Настраиваем RX pipe на bind_addr для получения ответа
            printf("BIND: Setting RX pipe to bind_addr for response...\r\n");
            nrf24_open_rx_pipe(0, bind_addr);
            
            // Переключаемся в режим приема для ожидания ответа
            nrf24_listen();
            printf("BIND: Switched to RX mode on bind_addr, waiting for response...\r\n");
            
            // Переходим в состояние ожидания ответа
            binding_state = 1;
            tx_send_time = current_time;
        }
    } else if (binding_state == 1) {
        // Состояние: ждем ответ в RX режиме
        
        // Проверяем, есть ли ответ от приемника
        if (nrf24_check_bind_response()) {
            printf("BIND: Response received! Exiting binding mode...\r\n");
            // Убираем дополнительные пищания - они уже есть в nrf24_check_bind_response()
            nrf24_exit_binding_mode();
            return;
        }
        
        // Проверяем таймаут ожидания ответа
        if ((current_time - tx_send_time) >= BIND_RX_WAIT_MS) {
            printf("BIND: RX timeout, preparing for next TX cycle...\r\n");
            
            // Выходим из режима приема, готовимся к следующей отправке
            nrf24_stop_listen();
            
            // Переходим обратно в состояние готовности к TX
            binding_state = 0;
            last_bind_cycle_time = current_time;
        }
    }
}

/**
 * @brief Расчет контрольной суммы для ответа привязки
 * @param packet Указатель на пакет ответа
 * @retval Контрольная сумма
 */
static uint8_t calculate_bind_response_checksum(const bind_response_t* packet) {
    uint8_t checksum = packet->packet_type;
    for(int i = 0; i < 5; i++) {
        checksum += packet->receiver_address[i];
    }
    return checksum;
}

/**
 * @brief Проверка ответа BIND_OK от приемника
 * @retval 1 - получен корректный ответ, 0 - ответа нет или ошибка
 */
uint8_t nrf24_check_bind_response(void) {
    if (!binding_mode) {
        return 0; // Не в режиме привязки
    }
    
    // Проверяем наличие данных в RX буфере
    uint8_t status_reg = nrf24_r_reg(STATUS, 1);
    bool rx_dr = (status_reg & 0x40) != 0;  // Бит 6 - RX_DR (данные получены)
    
    if (!rx_dr) {
        return 0; // Нет данных
    }
    
    // Читаем данные
    uint8_t rx_data[32];
    if (nrf24_data_available()) {
        nrf24_receive(rx_data, 32);
        
        // Очищаем флаг RX_DR
        uint8_t clear_rx_dr = 0x40;
        nrf24_w_reg(STATUS, &clear_rx_dr, 1);
        
        // Проверяем, что это ответ на привязку
        bind_response_t* response = (bind_response_t*)rx_data;
        
        if (response->packet_type == BIND_RESPONSE_TYPE) {
            printf("BIND_OK response received!\r\n");
            printf("Response type: 0x%02X\r\n", response->packet_type);
            printf("Response address: 0x%02X%02X%02X%02X%02X\r\n",
                   response->receiver_address[0], response->receiver_address[1],
                   response->receiver_address[2], response->receiver_address[3],
                   response->receiver_address[4]);
            
            // Проверяем контрольную сумму
            uint8_t calculated_checksum = calculate_bind_response_checksum(response);
            if (calculated_checksum != response->checksum) {
                printf("BIND_OK checksum error: calc=0x%02X, recv=0x%02X\r\n", 
                       calculated_checksum, response->checksum);
                return 0;
            }
            
            printf("*** BIND_OK VALID! Switching to new address ***\r\n");
             BUZZER_Go(TBUZ_50, TICK_10);
            
            // Обновляем рабочий адрес на адрес из my_addr
            memcpy(work_addr, my_addr, 5);
            
            printf("New working address: 0x%02X%02X%02X%02X%02X\r\n",
                   work_addr[0], work_addr[1], work_addr[2], work_addr[3], work_addr[4]);
            
            return 1; // Успешно получен корректный ответ
        } else {
            printf("Non-BIND_OK packet received in binding mode (type=0x%02X)\r\n", response->packet_type);
        }
    }
    
    return 0;
}
