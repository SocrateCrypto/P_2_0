#pragma once
#include <stdint.h>

// Тип для функции-обработчика
typedef void (*TimerCallback)(void);

// Структура для хранения состояния таймера
typedef struct {
    uint32_t last_tick;
} TimerTicker;

// Функция, вызывающая callback каждые interval_sec секунд
void call_every_seconds(TimerTicker *ticker, uint32_t interval_sec, TimerCallback callback);
