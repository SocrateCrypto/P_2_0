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

void blink_tick()
{
    static uint32_t last_red_time = 0;
    static uint32_t last_green_time = 0;
    static uint32_t last_blue_time = 0;
    uint32_t current_time = HAL_GetTick();

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