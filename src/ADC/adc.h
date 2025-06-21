#pragma once
#include <stdint.h>

// Прототип функции для чтения ADC и преобразования в вольты
// channel — номер канала ADC (например, ADC_CHANNEL_0)
// k — коэффициент делителя напряжения
// name — строка для идентификации (например, "BAT_ADC")
float adc(uint32_t channel, float k, const char* name);