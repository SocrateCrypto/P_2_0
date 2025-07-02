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
#include "main.h"

/* Определения для передачи данных */
#define PLD_S 32  // Размер payload для совместимости с main.c

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

#endif /* NRF24_TRANSMIT_H */
