/**
 * @file nrf24_transmit.h
 * @brief Функции для передачи данных педалей через NRF24L01
 * @author Radio Pedals Project
 * @date 2025
 */

#ifndef NRF24_TRANSMIT_H
#define NRF24_TRANSMIT_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "main.h"

/* Определения для передачи данных */
#define PLD_S 32  // Размер payload для совместимости с main.c

/* Константы для привязки */
#define BIND_CHANNEL 76                                    // Канал для привязки (тот же что и рабочий)
#define BIND_PACKET_TYPE 0xBB                             // Тип пакета привязки
#define BIND_RESPONSE_TYPE 0xBC                           // Тип ответа на привязку "BIND_OK"
#define BIND_TIMEOUT_MS 30000                             // Таймаут режима привязки (30 секунд)
#define BIND_TX_INTERVAL_MS 20                          // Интервал между отправками bind-пакетов
#define BIND_RX_WAIT_MS 100                               // Время ожидания ответа после отправки

/* Структура пакета привязки */
typedef struct {
    uint8_t packet_type;     // 0xBB = Bind request
    uint8_t remote_address[5]; // Адрес пульта для связи
    uint8_t checksum;        // Простая контрольная сумма
} bind_packet_t;

/* Структура ответа на привязку */
typedef struct {
    uint8_t packet_type;     // 0xBC = Bind response "BIND_OK"
    uint8_t receiver_address[5]; // Адрес приемника (подтверждение)
    uint8_t checksum;        // Простая контрольная сумма
} bind_response_t;

/**
 * @brief Инициализация NRF24 модуля для передачи
 * @retval None
 */
void nrf24_radio_init(void);

/**
 * @brief Передача состояния педалей через NRF24
 * @param left_state Состояние левой педали (GPIO_PIN_RESET - нажата, GPIO_PIN_SET - отпущена)
 * @param right_state Состояние правой педали (GPIO_PIN_RESET - нажата, GPIO_PIN_SET - отпущена)  
 * @param both_state Состояние кнопки BOTH (GPIO_PIN_RESET - нажата, GPIO_PIN_SET - отпущена)
 * @retval None
 */
void nrf24_transmit_pedal_data(uint8_t left_state, uint8_t right_state, uint8_t both_state);

/**
 * @brief Функции привязки передатчика
 */
void nrf24_enter_binding_mode(void);
void nrf24_exit_binding_mode(void);
uint8_t nrf24_is_binding_mode(void);
void nrf24_send_bind_packet(void);
void nrf24_binding_loop(void);  // Функция для вызова из основного цикла
uint8_t nrf24_check_bind_response(void);  // Проверка ответа BIND_OK
uint8_t nrf24_receive_command(char *buf, size_t len);

#endif /* NRF24_TRANSMIT_H */
