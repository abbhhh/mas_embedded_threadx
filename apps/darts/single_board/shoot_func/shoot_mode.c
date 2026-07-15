#include "shoot_mode.h"

#include "bsp_dwt.h"

#include <math.h>
#include <stddef.h>

static uint64_t delay_start_us;
static bool     delay_active;
static const float position_tolerance_rad = 5.0f;

bool shoot_mode_3508_is_arrived(const DJI_Motor_t *motor, float target_angle_rad)
{
    if (motor == NULL) return false;

    return fabsf(motor->base.measure.total_angle - target_angle_rad) <= position_tolerance_rad &&
           fabsf(motor->base.measure.speed_rad) < 10.0f;
}

bool shoot_mode_delay_ms(uint32_t delay_ms)
{
    uint64_t now_us = BSP_DWT_GetTimeline_us();

    if (!delay_active)
    {
        delay_start_us = now_us;
        delay_active   = true;
    }

    if (now_us - delay_start_us < (uint64_t)delay_ms * 1000ULL) return false;

    delay_active = false;
    return true;
}
