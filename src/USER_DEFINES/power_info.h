#pragma once

typedef enum {
    POWER_INTERNAL_BATTERY,
    POWER_EXTERNAL
} PowerState;

typedef enum {
    BATTERY_GOOD,
    BATTERY_MEDIUM,
    BATTERY_BAD,
    BATTERY_VERY_BAD
} BatteryState;

typedef struct {
    PowerState power_state;
    BatteryState battery_state;
    float voltage;
} PowerInfo;

extern PowerInfo power_info;