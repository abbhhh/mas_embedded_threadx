#include "darts_servo_func.h"
#include "bsp_dwt.h"
#include <stddef.h>

/* Init() 绑定的舵机对象，索引与 Darts_Servo_Id_e 一一对应。 */
static Servo_Motor_t *servo_motors[DARTS_SERVO_COUNT];

/* 当前序列由调用方提供，序列运行期间步骤数组必须保持有效。 */
static const Darts_Servo_Step_t *sequence_steps;
static uint8_t sequence_step_count;
static uint8_t sequence_step_index;

/* 当前步骤开始时间，用于非阻塞地判断等待时间是否结束。 */
static uint64_t step_start_us;
static Darts_Servo_Sequence_State_e sequence_state = DARTS_SERVO_SEQUENCE_IDLE;

/* 向步骤掩码选中的舵机下发目标角度，并开始本步骤计时。 */
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
    /* 状态机只保存指针，不负责创建或释放舵机对象。 */
    servo_motors[DARTS_SERVO_RISE_L]  = rise_l;
    servo_motors[DARTS_SERVO_RISE_R]  = rise_r;
    servo_motors[DARTS_SERVO_TRAN]    = tran;
    servo_motors[DARTS_SERVO_GRIPPER] = gripper;
    servo_motors[DARTS_SERVO_TRIGGER] = trigger;
    Darts_Servo_Sequence_Cancel();
}

uint8_t Darts_Servo_Sequence_Start(const Darts_Servo_Step_t *steps, uint8_t step_count)
{
    /* 防止空序列以及运行过程中被另一个序列覆盖。 */
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

    /* 未达到本步骤等待时间时立即返回，不阻塞其他控制任务。 */
    if (elapsed_us < (uint64_t)step->wait_ms * 1000ULL) return;

    /* 当前步骤等待完成，切换到下一步骤。 */
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
    /* 只复位状态机；舵机保持最后一次下发的目标值。 */
    sequence_steps      = NULL;
    sequence_step_count = 0U;
    sequence_step_index = 0U;
    step_start_us       = 0U;
    sequence_state      = DARTS_SERVO_SEQUENCE_IDLE;
}

Darts_Servo_Sequence_State_e Darts_Servo_Sequence_GetState(void) { return sequence_state; }

uint8_t Darts_Servo_Sequence_GetStep(void) { return sequence_step_index; }
