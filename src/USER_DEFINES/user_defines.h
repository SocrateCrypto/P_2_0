#pragma once

// Коефіцієнти дільників напруги
#define EXT_BATTERY_DIVIDER_COEFF   11.56f   // коефіцієнт дільника для зовнішньої батареї (ADC_CHANNEL_0)
#define INT_BATTERY_DIVIDER_COEFF   1.5f     // коефіцієнт дільника для внутрішньої батареї (ADC_CHANNEL_1)

// Пороги визначення рівня заряду зовнішньої батареї (В)
#define EXT_BATTERY_GOOD_THRESHOLD      12.0f
#define EXT_BATTERY_MEDIUM_THRESHOLD    11.0f
#define EXT_BATTERY_BAD_THRESHOLD       10.0f

// Пороги визначення рівня заряду внутрішньої батареї (В)
#define INT_BATTERY_GOOD_THRESHOLD      4.0f
#define INT_BATTERY_MEDIUM_THRESHOLD    3.5f
#define INT_BATTERY_BAD_THRESHOLD       3.3f

// Поріг визначення наявності зовнішнього живлення (В)
#define EXT_POWER_PRESENT_THRESHOLD     5.0f