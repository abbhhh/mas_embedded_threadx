#ifndef DARTS_SHOOT_MODE_H
#define DARTS_SHOOT_MODE_H

#include "darts_def.h"
#include "motor_dji.h"

#include <stdbool.h>
#include <stdint.h>

bool shoot_mode_3508_is_arrived(const DJI_Motor_t *motor, float target_angle_rad);
bool shoot_mode_delay_ms(uint32_t delay_ms);
bool shoot_mode_limit_switch_init(uint16_t pin);
bool shoot_mode_limit_switch_take_event(void);

#endif /* DARTS_SHOOT_MODE_H */
