# pragma once

#include "ARGB.h"  // include header file
void blink_green(int count_green);
void blink_green_with_period(int count_green, int period);

void blink_red(int count_red);
void blink_red_with_period(int count_red, int period);

void blink_blue(int count_blue);
void blink_blue_with_period(int count_blue, int period);

// Функции для постоянного мигания во время привязки
void start_binding_blink(int period);
void stop_binding_blink(void);

void blink_tick( );
