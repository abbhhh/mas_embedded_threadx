#include "darts_servo_func.h"
#include "bsp_dwt.h"
#include <stddef.h>

static Servo_Motor_t *servo_motors[DARTS_SERVO_COUNT];
static const Darts_Servo_Step_t *sequence_steps;
static uint8_t sequence_step_count;
static uint8_t sequence_step_index;
static uint64_t step_start_us;
static Darts_Servo_Sequence_State_e sequence_state = DARTS_SERVO_SEQUENCE_IDLE;

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

void Darts_Servo_Sequence_Init(Servo_Motor_t *rise_l, Servo_Motor_t *rise_r, Servo_Motor_t *tran, Servo_Motor_t *gripper,
                               Servo_Motor_t *trigger)
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
    if (steps == NULL || step_count == 0U || sequence_state == DARTS_SERVO_SEQUENCE_RUNNING) return 0U;

    sequence_steps      = steps;
    sequence_step_count = step_count;
    sequence_step_index = 0U;
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

    const Darts_Servo_Step_t *step = &sequence_steps[sequence_step_index];
    uint64_t elapsed_us = BSP_DWT_GetTimeline_us() - step_start_us;
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
    sequence_step_count = 0U;
    sequence_step_index = 0U;
    step_start_us       = 0U;
    sequence_state      = DARTS_SERVO_SEQUENCE_IDLE;
}

Darts_Servo_Sequence_State_e Darts_Servo_Sequence_GetState(void) { return sequence_state; }

uint8_t Darts_Servo_Sequence_GetStep(void) { return sequence_step_index; }
