#include "shoot_func.h"

#include "bsp_dwt.h"
#include "darts_servo_func.h"
#include "dsp/fast_math_functions.h"
#include "module_offline.h"
#include "motor_def.h"
#include "motor_dji.h"
#include "motor_servo.h"
#include "shoot_mode.h"
#include "tim.h"

#include <stddef.h>

#define LOG_TAG "app_shoot"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

static DJI_Motor_t   *friction_l;
static DJI_Motor_t   *friction_r;
static Servo_Motor_t *rise_left;
static Servo_Motor_t *rise_right;
static Servo_Motor_t *transfer;
static Servo_Motor_t *gripper;
static Servo_Motor_t *trigger;

static shoot_mode_e  shoot_mode = shoot_off;
static loader_mode_e load_mode  = load_origin;
static shoot_mode_e  last_shoot_mode;
static loader_mode_e last_load_mode;

void shoot_init(void)
{
    {
    Motor_Init_Config_s friction_config = {
        .offline_init_config =
            {
                .name       = "3508_1", // 设备名称
                .timeout_ms = 100,      // 超时时间
                .beep_times = 1,        // 蜂鸣次数
                .enable     = 1,        // 是否启用离线管理
            },
        .transport = MOTOR_TRANSPORT_CAN,
        .transport_config.can =
            {
                .hcan = BSP_CAN_HANDLE1,
            },
        .controller_init_config = 
            { 
                // --- 外环：位置环 (使用电机内部 total_angle) ---
                .angle_PID = 
                    {
                        .Kp = 7.51f,           
                        .Ki = 0.0f,
                        .Kd = 0.000f,          
                        .MaxOut = 6.0f,       
                        .DeadBand = 0.0f,
                        .Improve = PID_DerivativeFilter |PID_Derivative_On_Measurement,
                        .Derivative_LPF_RC = 0.0035f,
                    },
                .speed_PID = 
                    {
                        .Kp = 0.014f,           
                        .Ki = 0.0005f,
                        .Kd = 0.000f,          
                        .MaxOut = 4.0f,        
                        .DeadBand = 0.0f,
                        .Improve = PID_Integral_Limit,
                        .IntegralLimit = 0.5f,
                    },
            },
        .setting_init_config =
            {
                .angle_feedback_source = 0,
                .speed_feedback_source = 0,
                .loop_type             = SPEED_LOOP,
                .feedback_reverse_flag = 0,
                .algorithm_type        = CONTROL_PID,
            },
        .motor_init_info = {.motor_type = M3508, .gear_ratio = 19, .max_torque = 6, .torque_constant = 0.0016f},
    };
    // 左摩擦轮
    friction_config.transport_config.can.tx_id = 1;
    friction_l                                 = Motor_DJI_Init(&friction_config);
    if (friction_l == NULL)
    {
        LOG_E("friction_l init failed");
        return;
    }
    // 右摩擦轮
    friction_config.transport_config.can.tx_id     = 4; // 右摩擦轮,改txid和方向就行
    friction_config.offline_init_config.name       = "3508_2";
    friction_config.offline_init_config.beep_times = 4;
    friction_r                                     = Motor_DJI_Init(&friction_config);
    if (friction_r == NULL)
    {
        LOG_E("friction_r init failed");
        return;
    }

    Motor_Init_Config_s servo_config = {
        .offline_init_config =
            {
                .name       = "darts_rise_left",
                .timeout_ms = 100,
                .beep_times = 0,
                .enable     = 0,
            },
        .transport = MOTOR_TRANSPORT_PWM,
        .transport_config.pwm =
            {
                .htim    = &htim1,
                .Channel = TIM_CHANNEL_1,
                .dutyx10 = 125,
                .Mode    = PWM_MODE_BLOCKING,
            },
        .controller_init_config = {0},
        .setting_init_config =
            {
                .loop_type      = ANGLE_LOOP,
                .algorithm_type = CONTROL_PID,
                .enableflag     = 0U,
            },
        .motor_init_info =
            {
                .motor_type = SERVO_GENERIC,
                .gear_ratio = 1.0f,
            },
    };

    rise_left = Motor_Servo_Init(&servo_config, 50, 250, 0, 180);
    if (rise_left == NULL)
    {
        LOG_E("rise_left init failed");
        return;
    }

    servo_config.offline_init_config.name     = "darts_rise_right";
    servo_config.transport_config.pwm.Channel = TIM_CHANNEL_2;
    rise_right = Motor_Servo_Init(&servo_config, 50, 250, 0, 180);
    if (rise_right == NULL)
    {
        LOG_E("rise_right init failed");
        return;
    }

    servo_config.offline_init_config.name     = "darts_transfer";
    servo_config.transport_config.pwm.Channel = TIM_CHANNEL_3;
    transfer = Motor_Servo_Init(&servo_config, 50, 250, 0, 270);
    if (transfer == NULL)
    {
        LOG_E("transfer init failed");
        return;
    }

    servo_config.offline_init_config.name     = "darts_gripper";
    servo_config.transport_config.pwm.Channel = TIM_CHANNEL_4;
    gripper = Motor_Servo_Init(&servo_config, 50, 250, 0, 90);
    if (gripper == NULL)
    {
        LOG_E("gripper init failed");
        return;
    }

    servo_config.offline_init_config.name     = "darts_trigger";
    servo_config.transport_config.pwm.htim    = &htim8;
    servo_config.transport_config.pwm.Channel = TIM_CHANNEL_1;
    trigger = Motor_Servo_Init(&servo_config, 50, 250, 0, 90);
    if (trigger == NULL)
    {
        LOG_E("trigger init failed");
        return;
    }

    Motor_Servo_Start(rise_left);
    Motor_Servo_Start(rise_right);
    Motor_Servo_Start(transfer);
    Motor_Servo_Start(gripper);
    Motor_Servo_Start(trigger);
    Motor_Servo_SetRef(rise_left, 160);
    Motor_Servo_SetRef(rise_right, 0);
    Motor_Servo_SetRef(transfer, 135);
    Motor_Servo_SetRef(gripper, 70);
    Motor_Servo_SetRef(trigger, 20);
    Darts_Servo_Sequence_Init(rise_left, rise_right, transfer, gripper, trigger);


    shoot_mode_init(&shoot_mode, &load_mode);
    last_shoot_mode = shoot_mode;
    last_load_mode  = load_mode;

    LOG_I("darts shoot init success");
    }
}
void shoot_func(Shoot_Ctrl_Cmd_t *shoot_cmd)
{
    if (shoot_cmd == NULL || friction_l == NULL || friction_r == NULL || rise_left == NULL || rise_right == NULL || transfer == NULL ||
        gripper == NULL || trigger == NULL)
        return;

    Darts_Servo_Sequence_Update();
    Darts_Servo_Sequence_State_e reload_state = Darts_Servo_Sequence_GetState();
    Shoot_Mode_Feedback_t        feedback     = {
                   .now_us        = BSP_DWT_GetTimeline_us(),
                   .motors_online = Module_Offline_get_device_status(friction_l->base.offline_dev) == STATE_ONLINE &&
                                    Module_Offline_get_device_status(friction_r->base.offline_dev) == STATE_ONLINE,
                   .left_angle      = friction_l->base.measure.total_angle,
                   .right_angle     = friction_r->base.measure.total_angle,
                   .left_speed      = friction_l->base.measure.speed_rad,
                   .right_speed     = friction_r->base.measure.speed_rad,
                   .reload_finished = reload_state == DARTS_SERVO_SEQUENCE_FINISHED,
                   .reload_error    = reload_state == DARTS_SERVO_SEQUENCE_ERROR,
    };
    shoot_mode_update(shoot_cmd, &feedback, &shoot_mode, &load_mode);

    switch (load_mode)
    {
    case load_origin:
        break;

    case load_stop:
        if (shoot_mode != shoot_error && shoot_mode != shoot_finished)
        {
            Motor_DJI_Start(friction_l);
            Motor_DJI_Start(friction_r);
            Motor_DJI_SetRef(friction_l, shoot_mode_get_left_home_angle());
            Motor_DJI_SetRef(friction_r, shoot_mode_get_right_home_angle());
        }
        break;

    case load_cock:
        Motor_DJI_Start(friction_l);
        Motor_DJI_Start(friction_r);
        Motor_DJI_SetRef(friction_l, shoot_mode_get_left_home_angle() - 6.283185307f);
        Motor_DJI_SetRef(friction_r, shoot_mode_get_right_home_angle() + 6.283185307f);
        break;

    case load_return:
        Motor_DJI_Start(friction_l);
        Motor_DJI_Start(friction_r);
        Motor_DJI_SetRef(friction_l, shoot_mode_get_left_home_angle());
        Motor_DJI_SetRef(friction_r, shoot_mode_get_right_home_angle());
        break;

    case load_reload:
        Motor_DJI_Start(friction_l);
        Motor_DJI_Start(friction_r);
        Motor_DJI_SetRef(friction_l, shoot_mode_get_left_home_angle());
        Motor_DJI_SetRef(friction_r, shoot_mode_get_right_home_angle());
        if (last_load_mode != load_reload)
        {
            const Darts_Servo_Step_t *steps        = NULL;
            uint8_t                   step_count   = 0U;
            int8_t                    reload_index = shoot_mode_get_reload_index();
            if (reload_index < 0 || !Darts_Servo_Reload_Get((uint8_t)reload_index, &steps, &step_count) ||
                !Darts_Servo_Sequence_Start(steps, step_count))
            {
                shoot_mode_set_error(shoot_reload_error, &shoot_mode, &load_mode);
            }
        }
        break;
    }

    switch (shoot_mode)
    {
    case shoot_off:
        break;

    case shoot_lock:
        Motor_Servo_SetRef(trigger, 75.0f);
        break;

    case shoot_fire:
        Motor_Servo_SetRef(trigger, 20.0f);
        break;

    case shoot_finished:
        Motor_DJI_Stop(friction_l);
        Motor_DJI_Stop(friction_r);
        break;

    case shoot_error:
        Motor_DJI_Stop(friction_l);
        Motor_DJI_Stop(friction_r);
        Darts_Servo_Sequence_Cancel();
        break;
    }

    if (shoot_mode != last_shoot_mode || load_mode != last_load_mode)
    {
        if (shoot_mode == shoot_error)
            LOG_E("shoot error=%d", shoot_mode_get_fault());
        else
            LOG_I("shoot_mode=%d load_mode=%d salvo=%d", shoot_mode, load_mode, shoot_mode_get_salvo_index());

        last_shoot_mode = shoot_mode;
        last_load_mode  = load_mode;
    }
}
