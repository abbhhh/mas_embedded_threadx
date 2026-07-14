#ifndef DARTS_SHOOT_MODE_H
#define DARTS_SHOOT_MODE_H

#include "darts_def.h"

#include <stdint.h>

typedef struct
{
    uint64_t now_us;
    uint8_t  motors_online;
    float    left_angle;
    float    right_angle;
    float    left_speed;
    float    right_speed;
    uint8_t  reload_finished;
    uint8_t  reload_error;
} Shoot_Mode_Feedback_t;

void         shoot_remote_init(void);
shoot_mode_e shoot_remote_update(uint8_t is_upper, uint8_t is_active);

void shoot_mode_init(shoot_mode_e *shoot_mode, loader_mode_e *load_mode);
void shoot_mode_update(const Shoot_Ctrl_Cmd_t *shoot_cmd, const Shoot_Mode_Feedback_t *feedback, shoot_mode_e *shoot_mode, loader_mode_e *load_mode);
void shoot_mode_set_error(shoot_fault_e fault, shoot_mode_e *shoot_mode, loader_mode_e *load_mode);

float         shoot_mode_get_left_home_angle(void);
float         shoot_mode_get_right_home_angle(void);
int8_t        shoot_mode_get_reload_index(void);
uint8_t       shoot_mode_get_salvo_index(void);
shoot_fault_e shoot_mode_get_fault(void);

#endif /* DARTS_SHOOT_MODE_H */
