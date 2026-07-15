#include "motor_dji.h"
#include "bsp_dwt.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>

extern bool shoot_mode_3508_is_arrived(const DJI_Motor_t *motor, float target_angle_rad);

uint64_t BSP_DWT_GetTimeline_us(void) { return 0U; }

int main(void)
{
    DJI_Motor_t motor = {0};

    motor.base.controller.ref      = -100.0f;
    motor.base.measure.total_angle = 15.0f;
    motor.base.measure.speed_rad   = 9.9f;
    assert(shoot_mode_3508_is_arrived(&motor, 10.0f));

    motor.base.measure.speed_rad = 10.0f;
    assert(!shoot_mode_3508_is_arrived(&motor, 10.0f));

    motor.base.measure.speed_rad = -10.1f;
    assert(!shoot_mode_3508_is_arrived(&motor, 10.0f));

    motor.base.measure.speed_rad = 0.0f;
    motor.base.measure.total_angle = 15.01f;
    assert(!shoot_mode_3508_is_arrived(&motor, 10.0f));

    assert(!shoot_mode_3508_is_arrived(NULL, 10.0f));
    return 0;
}
