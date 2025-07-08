#include "NRF24.h"
#include "NRF24_reg_addresses.h"
#include <stdint.h>

// Проверка: RX FIFO FULL
uint8_t nrf24_rx_fifo_full(void) {
    uint8_t reg_dt = nrf24_r_reg(FIFO_STATUS, 1);
    return (reg_dt & (1 << RX_FULL)) ? 1 : 0;
}
