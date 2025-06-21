#include "adc.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <stdlib.h>

#define ADC_PRINT_ENABLE   // закомментируйте для отключения вывода

// Функция для чтения значения с аналогового пина и преобразования в вольты с учетом коэффициента делителя
float adc(uint32_t channel, float k, const char* name) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;

    extern ADC_HandleTypeDef hadc1;

    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    uint32_t raw = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    float voltage = (float)raw * 3.3f / 4095.0f;
    float real_voltage = voltage * k;

#ifdef ADC_PRINT_ENABLE
    static uint32_t lastTick = 0;
    static uint32_t period = 0;
    uint32_t now = HAL_GetTick();
    if (period == 0 || now - lastTick >= period) {
        // Рандомный период от 2000 до 3000 мс
        period = 2000 + rand() % 1001;
       // printf("%s:  %.3f V\r\n", name, real_voltage);
        lastTick = now;
    }
#endif

    return real_voltage;
}