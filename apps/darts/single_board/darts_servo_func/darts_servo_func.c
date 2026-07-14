/*
 * @Author: abbhhh 804433588@qq.com
 * @Date: 2026-07-13 22:13:00
 * @LastEditors: abbhhh 804433588@qq.com
 * @LastEditTime: 2026-07-14 18:00:59
 * @FilePath: \mas_embedded_threadx\apps\darts\single_board\darts_servo_func\darts_servo_func.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "darts_servo_func.h"
#include "bsp_dwt.h"
#include "darts_def.h"
#include <stddef.h>

#define ARRAY_SIZE(array) ((uint8_t)(sizeof(array) / sizeof((array)[0])))

static Servo_Motor_t               *servo_motors[DARTS_SERVO_COUNT];
static const Darts_Servo_Step_t    *sequence_steps;
static uint8_t                      sequence_step_count;
static uint8_t                      sequence_step_index;
static uint64_t                     step_start_us;
static Darts_Servo_Sequence_State_e sequence_state = DARTS_SERVO_SEQUENCE_IDLE;

static const Darts_Servo_Step_t reload_sequence_0[] = {
    {
        .motor_mask = DARTS_SERVO_MASK(DARTS_SERVO_RISE_L) | DARTS_SERVO_MASK(DARTS_SERVO_RISE_R),
        .target_deg = {[DARTS_SERVO_RISE_L] = 0.0f, [DARTS_SERVO_RISE_R] = 150.0f},
        .wait_ms    = 1000,
    },
    {
        .motor_mask = DARTS_SERVO_MASK(DARTS_SERVO_GRIPPER),
        .target_deg = {[DARTS_SERVO_GRIPPER] = 90.0f},
        .wait_ms    = 500,
    },
    {
        .motor_mask = DARTS_SERVO_MASK(DARTS_SERVO_RISE_L) | DARTS_SERVO_MASK(DARTS_SERVO_RISE_R),
        .target_deg = {[DARTS_SERVO_RISE_L] = 150.0f, [DARTS_SERVO_RISE_R] = 0.0f},
        .wait_ms    = 1000,
    },
};

static const Darts_Servo_Step_t reload_sequence_1[] = {
    {
        .motor_mask = DARTS_SERVO_MASK(DARTS_SERVO_TRAN),
        .target_deg = {[DARTS_SERVO_TRAN] = 0.0f},
        .wait_ms    = 1000,
    },
    {
        .motor_mask = DARTS_SERVO_MASK(DARTS_SERVO_RISE_L) | DARTS_SERVO_MASK(DARTS_SERVO_RISE_R),
        .target_deg = {[DARTS_SERVO_RISE_L] = 0.0f, [DARTS_SERVO_RISE_R] = 150.0f},
        .wait_ms    = 1000,
    },
    {
        .motor_mask = DARTS_SERVO_MASK(DARTS_SERVO_GRIPPER),
        .target_deg = {[DARTS_SERVO_GRIPPER] = 40.0f},
        .wait_ms    = 500,
    },
    {
        .motor_mask = DARTS_SERVO_MASK(DARTS_SERVO_RISE_L) | DARTS_SERVO_MASK(DARTS_SERVO_RISE_R),
        .target_deg = {[DARTS_SERVO_RISE_L] = 150.0f, [DARTS_SERVO_RISE_R] = 0.0f},
        .wait_ms    = 1000,
    },
    {
        .motor_mask = DARTS_SERVO_MASK(DARTS_SERVO_TRAN),
        .target_deg = {[DARTS_SERVO_TRAN] = 135.0f},
        .wait_ms    = 1000,
    },
    {
        .motor_mask = DARTS_SERVO_MASK(DARTS_SERVO_RISE_L) | DARTS_SERVO_MASK(DARTS_SERVO_RISE_R),
        .target_deg = {[DARTS_SERVO_RISE_L] = 0.0f, [DARTS_SERVO_RISE_R] = 150.0f},
        .wait_ms    = 1000,
    },
    {
        .motor_mask = DARTS_SERVO_MASK(DARTS_SERVO_GRIPPER),
        .target_deg = {[DARTS_SERVO_GRIPPER] = 90.0f},
        .wait_ms    = 500,
    },
    {
        .motor_mask = DARTS_SERVO_MASK(DARTS_SERVO_RISE_L) | DARTS_SERVO_MASK(DARTS_SERVO_RISE_R),
        .target_deg = {[DARTS_SERVO_RISE_L] = 150.0f, [DARTS_SERVO_RISE_R] = 0.0f},
        .wait_ms    = 1000,
    },
};

static const Darts_Servo_Step_t reload_sequence_2[] = {
    {
        .motor_mask = DARTS_SERVO_MASK(DARTS_SERVO_TRAN),
        .target_deg = {[DARTS_SERVO_TRAN] = 270.0f},
        .wait_ms    = 1000,
    },
    {
        .motor_mask = DARTS_SERVO_MASK(DARTS_SERVO_RISE_L) | DARTS_SERVO_MASK(DARTS_SERVO_RISE_R),
        .target_deg = {[DARTS_SERVO_RISE_L] = 0.0f, [DARTS_SERVO_RISE_R] = 150.0f},
        .wait_ms    = 1000,
    },
    {
        .motor_mask = DARTS_SERVO_MASK(DARTS_SERVO_GRIPPER),
        .target_deg = {[DARTS_SERVO_GRIPPER] = 40.0f},
        .wait_ms    = 500,
    },
    {
        .motor_mask = DARTS_SERVO_MASK(DARTS_SERVO_RISE_L) | DARTS_SERVO_MASK(DARTS_SERVO_RISE_R),
        .target_deg = {[DARTS_SERVO_RISE_L] = 160.0f, [DARTS_SERVO_RISE_R] = 0.0f},
        .wait_ms    = 1000,
    },
    {
        .motor_mask = DARTS_SERVO_MASK(DARTS_SERVO_TRAN),
        .target_deg = {[DARTS_SERVO_TRAN] = 135.0f},
        .wait_ms    = 500,
    },
    {
        .motor_mask = DARTS_SERVO_MASK(DARTS_SERVO_RISE_L) | DARTS_SERVO_MASK(DARTS_SERVO_RISE_R),
        .target_deg = {[DARTS_SERVO_RISE_L] = 0.0f, [DARTS_SERVO_RISE_R] = 150.0f},
        .wait_ms    = 1000,
    },
    {
        .motor_mask = DARTS_SERVO_MASK(DARTS_SERVO_GRIPPER),
        .target_deg = {[DARTS_SERVO_GRIPPER] = 90.0f},
        .wait_ms    = 500,
    },
    {
        .motor_mask = DARTS_SERVO_MASK(DARTS_SERVO_RISE_L) | DARTS_SERVO_MASK(DARTS_SERVO_RISE_R),
        .target_deg = {[DARTS_SERVO_RISE_L] = 150.0f, [DARTS_SERVO_RISE_R] = 0.0f},
        .wait_ms    = 1000,
    },
};

typedef struct
{
    const Darts_Servo_Step_t *steps;
    uint8_t                   step_count;
} Darts_Reload_Sequence_t;

static const Darts_Reload_Sequence_t reload_sequences[] = {
    {reload_sequence_0, ARRAY_SIZE(reload_sequence_0)},
    {reload_sequence_1, ARRAY_SIZE(reload_sequence_1)},
    {reload_sequence_2, ARRAY_SIZE(reload_sequence_2)},
};

static uint8_t apply_step(const Darts_Servo_Step_t *step)
{
    for (uint8_t i = 0; i < DARTS_SERVO_COUNT; ++i)
    {
        if ((step->motor_mask & DARTS_SERVO_MASK(i)) == 0U) continue;
        if (servo_motors[i] == NULL) return 0U;

        Motor_Servo_Start(servo_motors[i]);
        Motor_Servo_SetRef(servo_motors[i], step->target_deg[i]);
    }

    step_start_us = BSP_DWT_GetTimeline_us();
    return 1U;
}

void Darts_Servo_Sequence_Init(Servo_Motor_t *rise_l, Servo_Motor_t *rise_r, Servo_Motor_t *tran, Servo_Motor_t *gripper, Servo_Motor_t *trigger)
{
    servo_motors[DARTS_SERVO_RISE_L]  = rise_l;
    servo_motors[DARTS_SERVO_RISE_R]  = rise_r;
    servo_motors[DARTS_SERVO_TRAN]    = tran;
    servo_motors[DARTS_SERVO_GRIPPER] = gripper;
    servo_motors[DARTS_SERVO_TRIGGER] = trigger;
    Darts_Servo_Sequence_Cancel();
}

uint8_t Darts_Servo_Sequence_Start(const Darts_Servo_Step_t *steps, uint8_t step_count)
{
    if (steps == NULL || step_count == 0 || sequence_state == DARTS_SERVO_SEQUENCE_RUNNING) return 0;

    sequence_steps      = steps;
    sequence_step_count = step_count;
    sequence_step_index = 0;
    sequence_state      = DARTS_SERVO_SEQUENCE_RUNNING;

    if (!apply_step(&sequence_steps[0]))
    {
        sequence_state = DARTS_SERVO_SEQUENCE_ERROR;
        return 0U;
    }
    return 1U;
}

void Darts_Servo_Sequence_Update(void)
{
    if (sequence_state != DARTS_SERVO_SEQUENCE_RUNNING) return;

    const Darts_Servo_Step_t *step       = &sequence_steps[sequence_step_index];
    uint64_t                  elapsed_us = BSP_DWT_GetTimeline_us() - step_start_us;
    if (elapsed_us < (uint64_t)step->wait_ms * 1000ULL) return;

    ++sequence_step_index;
    if (sequence_step_index >= sequence_step_count)
    {
        sequence_state = DARTS_SERVO_SEQUENCE_FINISHED;
        return;
    }

    if (!apply_step(&sequence_steps[sequence_step_index])) sequence_state = DARTS_SERVO_SEQUENCE_ERROR;
}

void Darts_Servo_Sequence_Cancel(void)
{
    sequence_steps      = NULL;
    sequence_step_count = 0;
    sequence_step_index = 0;
    step_start_us       = 0;
    sequence_state      = DARTS_SERVO_SEQUENCE_IDLE;
}

Darts_Servo_Sequence_State_e Darts_Servo_Sequence_GetState(void) { return sequence_state; }

uint8_t Darts_Servo_Sequence_GetStep(void) { return sequence_step_index; }

uint8_t Darts_Servo_Reload_Get(uint8_t index, const Darts_Servo_Step_t **steps, uint8_t *step_count)
{
    if (steps == NULL || step_count == NULL || index >= ARRAY_SIZE(reload_sequences)) return 0U;

    *steps      = reload_sequences[index].steps;
    *step_count = reload_sequences[index].step_count;
    return 1U;
}
