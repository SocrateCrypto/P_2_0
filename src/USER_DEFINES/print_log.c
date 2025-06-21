#include <stdio.h>
#include "power_info.h"
#include "adc.h"
#include "user_defines.h"


void print_log()
{
    if (power_info.power_state == POWER_EXTERNAL) {
        printf("State: EXTERNAL BATTERY\r\n");
    } else {
        printf("State: INTERNAL BATTERY\r\n");
    }
    
    switch (power_info.battery_state) {
        case BATTERY_GOOD:
            printf("Battery: GOOD\r\n");
            break;
        case BATTERY_MEDIUM:
            printf("Battery: MEDIUM\r\n");
            break;
        case BATTERY_BAD:
            printf("Battery: BAD\r\n");
            break;
        case BATTERY_VERY_BAD:
            printf("Battery: VERY BAD\r\n");
            break;
        default:
            printf("Battery: UNKNOWN\r\n");
            break;
    }
    printf("Voltage: %.2f V\r\n", power_info.voltage);
}