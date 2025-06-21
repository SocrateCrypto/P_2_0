#include "timer.h"
#include "stm32f1xx_hal.h" // Для HAL_GetTick()


// Функция, вызывающая callback каждые interval_sec секунд
void call_every_seconds(TimerTicker *ticker, uint32_t interval_sec, TimerCallback callback)
{
    uint32_t current_tick = HAL_GetTick() / 1000; // HAL_GetTick возвращает миллисекунды

    if ((current_tick - ticker->last_tick) >= interval_sec) {
        ticker->last_tick = current_tick;
        if (callback) {
            callback();
        }
    }
}

