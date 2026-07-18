/*
 * @Author: abbhhh 804433588@qq.com
 * @Date: 2026-07-14 14:21:58
 * @LastEditors: abbhhh 804433588@qq.com
 * @LastEditTime: 2026-07-15 22:54:30
 * @FilePath: \mas_embedded_threadx\apps\darts\single_board\shoot_func\shoot_mode.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "shoot_mode.h"

#include "bsp_dwt.h"
#include "bsp_gpio.h"

#include <math.h>
#include <stddef.h>

static uint64_t delay_start_us;
static bool     delay_active;
static const float position_tolerance_rad = 0.2f;
static volatile bool limit_switch_event;

static void shoot_mode_limit_switch_callback(void) { limit_switch_event = true; }

bool shoot_mode_3508_is_arrived(const DJI_Motor_t *motor, float target_angle_rad)
{
    if (motor == NULL) return false;

    return fabsf(motor->base.measure.total_angle - target_angle_rad) <= position_tolerance_rad &&
           fabsf(motor->base.measure.speed_rad) < 0.2f;
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

bool shoot_mode_limit_switch_init(uint16_t pin)
{
    return BSP_GPIO_EXTI_Register(pin, shoot_mode_limit_switch_callback) != 0xFFU;
}

bool shoot_mode_limit_switch_take_event(void)
{
    if (!limit_switch_event) return false;

    limit_switch_event = false;
    return true;
}
