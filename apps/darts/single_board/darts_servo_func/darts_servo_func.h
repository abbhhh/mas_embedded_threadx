#ifndef DARTS_SERVO_FUNC_H
#define DARTS_SERVO_FUNC_H

#include "motor_servo.h"
#include <stdint.h>

/* 舵机在状态机内部数组中的固定索引。 */
typedef enum
{
    DARTS_SERVO_RISE_L = 0, /* 左抬升舵机 */
    DARTS_SERVO_RISE_R,     /* 右抬升舵机 */
    DARTS_SERVO_TRAN,       /* 转运舵机 */
    DARTS_SERVO_GRIPPER,    /* 夹爪舵机 */
    DARTS_SERVO_TRIGGER,    /* 扳机舵机 */
    DARTS_SERVO_COUNT,      /* 舵机总数，必须放在枚举末尾 */
} Darts_Servo_Id_e;

/* 将舵机索引转换为步骤掩码，可用按位或同时选择多个舵机。 */
#define DARTS_SERVO_MASK(id) ((uint8_t)(1U << (id)))

/*
 * 一条动作步骤。
 * motor_mask 选择本步骤需要更新的舵机；未选中的舵机保持原目标角度。
 * target_deg 按 Darts_Servo_Id_e 索引保存各舵机目标角度。
 * wait_ms 表示发出本步骤命令后，进入下一步骤前需要等待的时间。
 */
typedef struct
{
    uint8_t  motor_mask;
    float    target_deg[DARTS_SERVO_COUNT];
    uint32_t wait_ms;
} Darts_Servo_Step_t;

/* 舵机动作序列运行状态。 */
typedef enum
{
    DARTS_SERVO_SEQUENCE_IDLE = 0, /* 未运行或已取消 */
    DARTS_SERVO_SEQUENCE_RUNNING,  /* 正在等待或执行某一步 */
    DARTS_SERVO_SEQUENCE_FINISHED, /* 所有步骤执行完成 */
    DARTS_SERVO_SEQUENCE_ERROR,    /* 步骤引用了未绑定的舵机 */
} Darts_Servo_Sequence_State_e;

/* 绑定状态机使用的五个舵机，所有指针由调用方负责初始化和保持有效。 */
void Darts_Servo_Sequence_Init(Servo_Motor_t *rise_l, Servo_Motor_t *rise_r, Servo_Motor_t *tran, Servo_Motor_t *gripper, Servo_Motor_t *trigger);

/* 启动动作序列。运行期间不能重复启动，成功返回 1，失败返回 0。 */
uint8_t Darts_Servo_Sequence_Start(const Darts_Servo_Step_t *steps, uint8_t step_count);

/* 周期调用，依据 DWT 时间轴推进到下一步骤；函数本身不会阻塞线程。 */
void Darts_Servo_Sequence_Update(void);

/* 取消当前序列并清除步骤信息，不会主动改变舵机当前目标角度。 */
void Darts_Servo_Sequence_Cancel(void);

/* 查询当前运行状态和步骤索引。 */
Darts_Servo_Sequence_State_e Darts_Servo_Sequence_GetState(void);
uint8_t                      Darts_Servo_Sequence_GetStep(void);

/* 按 0/1/2 获取三套飞镖装填序列。 */
uint8_t Darts_Servo_Reload_Get(uint8_t index, const Darts_Servo_Step_t **steps, uint8_t *step_count);

#endif /* DARTS_SERVO_FUNC_H */
