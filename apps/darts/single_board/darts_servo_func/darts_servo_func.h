#ifndef DARTS_SERVO_FUNC_H
#define DARTS_SERVO_FUNC_H

#include "motor_servo.h"
#include <stdint.h>

typedef enum
{
    DARTS_SERVO_RISE_L = 0,
    DARTS_SERVO_RISE_R,
    DARTS_SERVO_TRAN,
    DARTS_SERVO_GRIPPER,
    DARTS_SERVO_TRIGGER,
    DARTS_SERVO_COUNT,
} Darts_Servo_Id_e;

#define DARTS_SERVO_MASK(id) ((uint8_t)(1U << (id)))

typedef struct
{
    uint8_t  motor_mask;
    float    target_deg[DARTS_SERVO_COUNT];
    uint32_t wait_ms;
} Darts_Servo_Step_t;

typedef enum
{
    DARTS_SERVO_SEQUENCE_IDLE = 0,
    DARTS_SERVO_SEQUENCE_RUNNING,
    DARTS_SERVO_SEQUENCE_FINISHED,
    DARTS_SERVO_SEQUENCE_ERROR,
} Darts_Servo_Sequence_State_e;

void Darts_Servo_Sequence_Init(Servo_Motor_t *rise_l, Servo_Motor_t *rise_r, Servo_Motor_t *tran, Servo_Motor_t *gripper,
                               Servo_Motor_t *trigger);
uint8_t Darts_Servo_Sequence_Start(const Darts_Servo_Step_t *steps, uint8_t step_count);
void Darts_Servo_Sequence_Update(void);
void Darts_Servo_Sequence_Cancel(void);
Darts_Servo_Sequence_State_e Darts_Servo_Sequence_GetState(void);
uint8_t Darts_Servo_Sequence_GetStep(void);

#endif /* DARTS_SERVO_FUNC_H */
