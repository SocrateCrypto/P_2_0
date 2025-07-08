#include "nrf24_transmit.h"
#include "NRF24.h"
#include <string.h>
#include <stdbool.h>

/**
 * @brief Приём команды по радио (строка, максимум 32 байта)
 * @param buf Буфер для принятой строки
 * @param len Размер буфера
 * @retval 1 если принято сообщение, 0 если нет
 */
uint8_t nrf24_receive_command(char *buf, size_t len) {
    if (!nrf24_data_available())
        return 0;
    uint8_t rx_data[32] = {0};
    nrf24_receive(rx_data, 32);
    // Копируем только строку (до нуля или конца буфера)
    strncpy(buf, (char*)rx_data, len-1);
    buf[len-1] = '\0';
    return 1;
}
