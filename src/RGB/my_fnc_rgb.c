#include "my_fnc_rgb.h"
#include "stm32f1xx_hal.h"

int remaining_calls_red = 0;
int remaining_calls_green = 0;
int remaining_calls_blue = 0;
int is_active_red = 0;
int is_active_green = 0;
int is_active_blue = 0;

int period_red = 500;
int period_green = 500;
int period_blue = 500;

// Переменные для постоянного мигания в режиме привязки
static int binding_blink_active = 0;
static int binding_blink_period = 500;
static int binding_blink_state = 0;

void blink_green(int count_green) {
    blink_green_with_period(count_green, 500); // 500 мс по умолчанию
}

void blink_green_with_period(int count_green, int period) {
    remaining_calls_green = count_green;
    period_green = period;
    is_active_green = 1;
}

void blink_red(int count_red) {
    blink_red_with_period(count_red, 500);
}

void blink_red_with_period(int count_red, int period) {
    remaining_calls_red = count_red;
    period_red = period;
    is_active_red = 1;
}

void blink_blue(int count_blue) {
    blink_blue_with_period(count_blue, 500);
}

void blink_blue_with_period(int count_blue, int period) {
    remaining_calls_blue = count_blue;
    period_blue = period;
    is_active_blue = 1;
}

// Функция для начала постоянного мигания в режиме привязки
void start_binding_blink(int period) {
    binding_blink_active = 1;
    binding_blink_period = period;
    binding_blink_state = 0;
}

// Функция для остановки постоянного мигания в режиме привязки
void stop_binding_blink(void) {
    binding_blink_active = 0;
    binding_blink_state = 0;
    // Выключаем светодиод
    ARGB_Clear();
    ARGB_Show();
}

void blink_tick()
{
    static uint32_t last_red_time = 0;
    static uint32_t last_green_time = 0;
    static uint32_t last_blue_time = 0;
    static uint32_t last_binding_time = 0;
    uint32_t current_time = HAL_GetTick();

    // ПРИОРИТЕТ: Постоянное мигание в режиме привязки
    if (binding_blink_active && current_time - last_binding_time >= binding_blink_period)
    {
        last_binding_time = current_time;
        if (binding_blink_state)
        {
            ARGB_FillRGB(255, 255, 0); // Желтый цвет для режима привязки
        }
        else
        {
            ARGB_Clear();
        }
        binding_blink_state = !binding_blink_state;
        ARGB_Show();
        return; // Прерываем выполнение, чтобы не было конфликтов с другими LED
    }

    if (remaining_calls_red > 0 && current_time - last_red_time >= period_red)
    {
        last_red_time = current_time;
        if (is_active_red)
        {
            ARGB_FillRGB(255, 0, 0);
        }
        else
        {
            ARGB_Clear();
            remaining_calls_red--;
        }
        is_active_red = !is_active_red;
        ARGB_Show();
    }

    if (remaining_calls_green > 0 && current_time - last_green_time >= period_green)
    {
        last_green_time = current_time;
        if (is_active_green)
        {
            ARGB_FillRGB(0, 255, 0);
        }
        else
        {
            ARGB_Clear();
            remaining_calls_green--;
        }
        is_active_green = !is_active_green;
        ARGB_Show();
    }

    if (remaining_calls_blue > 0 && current_time - last_blue_time >= period_blue)
    {
        last_blue_time = current_time;
        if (is_active_blue)
        {
            ARGB_FillRGB(0, 0, 255);
        }
        else
        {
            ARGB_Clear();
            remaining_calls_blue--;
        }
        is_active_blue = !is_active_blue;
        ARGB_Show();
    }
}