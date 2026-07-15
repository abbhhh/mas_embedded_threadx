#include "shoot_func.h"

#include "bsp_dwt.h"
#include "darts_def.h"
#include "dsp/fast_math_functions.h"
#include "module_offline.h"
#include "motor_def.h"
#include "motor_dji.h"
#include "motor_servo.h"
#include "shoot_mode.h"
#include "tim.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

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

typedef enum
{
    shoot_step_idle = 0,
    shoot_step_first_cock,
    shoot_step_first_lock_wait,
    shoot_step_first_return,
    shoot_step_first_fire_wait,
    shoot_step_second_cock,
    shoot_step_second_lock_wait,
    shoot_step_second_rise_down_wait,
    shoot_step_second_gripper_wait,
    shoot_step_second_rise_home_wait,
    shoot_step_second_return,
    shoot_step_second_fire_wait,
    shoot_step_third_cock,
    shoot_step_third_lock_wait,
    shoot_step_third_transfer_out_wait,
    shoot_step_third_rise_down_wait,
    shoot_step_third_gripper_wait,
    shoot_step_third_rise_home_wait,
    shoot_step_third_rise_down_again_wait,
    shoot_step_third_gripper_open_wait,
    shoot_step_third_rise_home_final_wait,
    shoot_step_third_return,
    shoot_step_third_fire_wait,
    shoot_step_fourth_cock,
    shoot_step_fourth_lock_wait,
    shoot_step_fourth_transfer_out_wait,
    shoot_step_fourth_rise_down_wait,
    shoot_step_fourth_gripper_wait,
    shoot_step_fourth_rise_home_wait,
    shoot_step_fourth_rise_down_again_wait,
    shoot_step_fourth_gripper_open_wait,
    shoot_step_fourth_rise_home_final_wait,
    shoot_step_fourth_return,
    shoot_step_fourth_fire_wait,
} shoot_step_e;

static shoot_step_e shoot_step = shoot_step_idle;
volatile uint8_t enable_flag = 0;
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
                        .Kp = 4.51f,           
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
                        .Ki = 0.0001f,
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
                .loop_type             = ANGLE_AND_SPEED_LOOP,
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
                .enableflag     = 0,
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

    Motor_DJI_Start(friction_l);
    Motor_DJI_Start(friction_r);
    Motor_Servo_Start(rise_left);
    Motor_Servo_Start(rise_right);
    Motor_Servo_Start(transfer);
    Motor_Servo_Start(gripper);
    Motor_Servo_Start(trigger);
    Motor_Servo_SetRef(rise_left, 140);
    Motor_Servo_SetRef(rise_right, 20);
    Motor_Servo_SetRef(transfer, 135);
    Motor_Servo_SetRef(gripper, 40);
    Motor_Servo_SetRef(trigger, 5.0f);

    LOG_I("darts shoot init success");
    }
}
void shoot_func(Shoot_Ctrl_Cmd_t *shoot_cmd)
{
    if (shoot_cmd == NULL || friction_l == NULL || friction_r == NULL || rise_left == NULL || rise_right == NULL || transfer == NULL ||
        gripper == NULL || trigger == NULL)
        return;

    if (!Module_Offline_get_device_status(friction_l->base.offline_dev) && !Module_Offline_get_device_status(friction_r->base.offline_dev))
    {
        if (shoot_cmd->shoot_mode == shoot_restart)
        {
            if (enable_flag != 0 && shoot_cmd -> shoot_mode == shoot_restart)
            {
                enable_flag = 0;
                shoot_step  = shoot_step_idle;
            }
            return;
        }

        if (shoot_cmd->shoot_mode == shoot_start_1)
        {
            if (enable_flag != 0) return;
            if (shoot_step == shoot_step_idle) shoot_step = shoot_step_first_cock;
        }
        else if (shoot_cmd->shoot_mode == shoot_start_2)
        {
            if (enable_flag != 1) return;
            if (shoot_step == shoot_step_idle) shoot_step = shoot_step_third_cock;
        }
        else
        {
            Motor_DJI_Stop(friction_l);
            Motor_DJI_Stop(friction_r);
            Motor_Servo_Stop(rise_left);
            Motor_Servo_Stop(rise_right);
            Motor_Servo_Stop(transfer);
            Motor_Servo_Stop(gripper);
            Motor_Servo_Stop(trigger);
            shoot_step = shoot_step_idle;
            return;
        }

        switch (shoot_step)
        {
        case shoot_step_first_cock:
            Motor_DJI_SetRef(friction_l, trigger_station);
            Motor_DJI_SetRef(friction_r, -trigger_station);
            if (!shoot_mode_3508_is_arrived(friction_l, trigger_station) || !shoot_mode_3508_is_arrived(friction_r, -trigger_station)) return;
            Motor_Servo_SetRef(trigger, 70);
            shoot_step = shoot_step_first_lock_wait;
            return;

        case shoot_step_first_lock_wait:
            if (!shoot_mode_delay_ms(500)) return;
            shoot_step = shoot_step_first_return;
            return;

        case shoot_step_first_return:
            Motor_DJI_SetRef(friction_l, 0);
            Motor_DJI_SetRef(friction_r, 0);
            if (!shoot_mode_3508_is_arrived(friction_l, 0) || !shoot_mode_3508_is_arrived(friction_r, 0)) return;
            Motor_Servo_SetRef(trigger, 5);
            shoot_step = shoot_step_first_fire_wait;
            return;

        case shoot_step_first_fire_wait:
            if (!shoot_mode_delay_ms(500)) return;
            shoot_step = shoot_step_second_cock;
            return;

        case shoot_step_second_cock:
            Motor_DJI_SetRef(friction_l, trigger_station);
            Motor_DJI_SetRef(friction_r, -trigger_station);
            if (!shoot_mode_3508_is_arrived(friction_l, trigger_station) || !shoot_mode_3508_is_arrived(friction_r, -trigger_station)) return;
            Motor_Servo_SetRef(trigger, 70);
            shoot_step = shoot_step_second_lock_wait;
            return;

        case shoot_step_second_lock_wait:
            if (!shoot_mode_delay_ms(500)) return;
            Motor_Servo_SetRef(rise_left, 20);
            Motor_Servo_SetRef(rise_right, 150);
            shoot_step = shoot_step_second_rise_down_wait;
            return;

        case shoot_step_second_rise_down_wait:
            if (!shoot_mode_delay_ms(500)) return;
            Motor_Servo_SetRef(gripper, 80);
            shoot_step = shoot_step_second_gripper_wait;
            return;

        case shoot_step_second_gripper_wait:
            if (!shoot_mode_delay_ms(300)) return;
            Motor_Servo_SetRef(rise_left, 150);
            Motor_Servo_SetRef(rise_right, 20);
            shoot_step = shoot_step_second_rise_home_wait;
            return;

        case shoot_step_second_rise_home_wait:
            if (!shoot_mode_delay_ms(500)) return;
            shoot_step = shoot_step_second_return;
            return;

        case shoot_step_second_return:
            Motor_DJI_SetRef(friction_l, 0);
            Motor_DJI_SetRef(friction_r, 0);
            if (!shoot_mode_3508_is_arrived(friction_l, 0) || !shoot_mode_3508_is_arrived(friction_r, 0)) return;
            Motor_Servo_SetRef(trigger, 5);
            shoot_step = shoot_step_second_fire_wait;
            return;

        case shoot_step_second_fire_wait:
            if (!shoot_mode_delay_ms(500)) return;
            enable_flag = 1;
            shoot_step  = shoot_step_idle;
            return;

        case shoot_step_third_cock:
            Motor_DJI_SetRef(friction_l, trigger_station);
            Motor_DJI_SetRef(friction_r, -trigger_station);
            if (!shoot_mode_3508_is_arrived(friction_l, trigger_station) || !shoot_mode_3508_is_arrived(friction_r, -trigger_station)) return;
            Motor_Servo_SetRef(trigger, 70);
            shoot_step = shoot_step_third_lock_wait;
            return;

        case shoot_step_third_lock_wait:
            if (!shoot_mode_delay_ms(500)) return;
            Motor_Servo_SetRef(transfer, 45);
            shoot_step = shoot_step_third_transfer_out_wait;
            return;

        case shoot_step_third_transfer_out_wait:
            if (!shoot_mode_delay_ms(700)) return;
            Motor_Servo_SetRef(rise_left, 20);
            Motor_Servo_SetRef(rise_right, 150);
            shoot_step = shoot_step_third_rise_down_wait;
            return;

        case shoot_step_third_rise_down_wait:
            if (!shoot_mode_delay_ms(500)) return;
            Motor_Servo_SetRef(gripper, 45);
            shoot_step = shoot_step_third_gripper_wait;
            return;

        case shoot_step_third_gripper_wait:
            if (!shoot_mode_delay_ms(300)) return;
            Motor_Servo_SetRef(rise_left, 150);
            Motor_Servo_SetRef(rise_right, 20);
            Motor_Servo_SetRef(transfer, 135);
            shoot_step = shoot_step_third_rise_home_wait;
            return;

        case shoot_step_third_rise_home_wait:
            if (!shoot_mode_delay_ms(700)) return;
            Motor_Servo_SetRef(rise_left, 20);
            Motor_Servo_SetRef(rise_right, 150);
            shoot_step = shoot_step_third_rise_down_again_wait;
            return;

        case shoot_step_third_rise_down_again_wait:
            if (!shoot_mode_delay_ms(500)) return;
            Motor_Servo_SetRef(gripper, 80);
            shoot_step = shoot_step_third_gripper_open_wait;
            return;

        case shoot_step_third_gripper_open_wait:
            if (!shoot_mode_delay_ms(300)) return;
            Motor_Servo_SetRef(rise_left, 150);
            Motor_Servo_SetRef(rise_right, 20);
            shoot_step = shoot_step_third_rise_home_final_wait;
            return;

        case shoot_step_third_rise_home_final_wait:
            if (!shoot_mode_delay_ms(500)) return;
            shoot_step = shoot_step_third_return;
            return;

        case shoot_step_third_return:
            Motor_DJI_SetRef(friction_l, 0);
            Motor_DJI_SetRef(friction_r, 0);
            if (!shoot_mode_3508_is_arrived(friction_l, 0) || !shoot_mode_3508_is_arrived(friction_r, 0)) return;
            Motor_Servo_SetRef(trigger, 5);
            shoot_step = shoot_step_third_fire_wait;
            return;

        case shoot_step_third_fire_wait:
            if (!shoot_mode_delay_ms(500)) return;
            shoot_step = shoot_step_fourth_cock;
            return;

        case shoot_step_fourth_cock:
            Motor_DJI_SetRef(friction_l, trigger_station);
            Motor_DJI_SetRef(friction_r, -trigger_station);
            if (!shoot_mode_3508_is_arrived(friction_l, trigger_station) || !shoot_mode_3508_is_arrived(friction_r, -trigger_station)) return;
            Motor_Servo_SetRef(trigger, 70);
            shoot_step = shoot_step_fourth_lock_wait;
            return;

        case shoot_step_fourth_lock_wait:
            if (!shoot_mode_delay_ms(500)) return;
            Motor_Servo_SetRef(transfer, 45);
            shoot_step = shoot_step_fourth_transfer_out_wait;
            return;

        case shoot_step_fourth_transfer_out_wait:
            if (!shoot_mode_delay_ms(700)) return;
            Motor_Servo_SetRef(rise_left, 20);
            Motor_Servo_SetRef(rise_right, 150);
            shoot_step = shoot_step_fourth_rise_down_wait;
            return;

        case shoot_step_fourth_rise_down_wait:
            if (!shoot_mode_delay_ms(500)) return;
            Motor_Servo_SetRef(gripper, 45);
            shoot_step = shoot_step_fourth_gripper_wait;
            return;

        case shoot_step_fourth_gripper_wait:
            if (!shoot_mode_delay_ms(300)) return;
            Motor_Servo_SetRef(rise_left, 150);
            Motor_Servo_SetRef(rise_right, 20);
            Motor_Servo_SetRef(transfer, 135);
            shoot_step = shoot_step_fourth_rise_home_wait;
            return;

        case shoot_step_fourth_rise_home_wait:
            if (!shoot_mode_delay_ms(700)) return;
            Motor_Servo_SetRef(rise_left, 20);
            Motor_Servo_SetRef(rise_right, 150);
            shoot_step = shoot_step_fourth_rise_down_again_wait;
            return;

        case shoot_step_fourth_rise_down_again_wait:
            if (!shoot_mode_delay_ms(500)) return;
            Motor_Servo_SetRef(gripper, 80);
            shoot_step = shoot_step_fourth_gripper_open_wait;
            return;

        case shoot_step_fourth_gripper_open_wait:
            if (!shoot_mode_delay_ms(300)) return;
            Motor_Servo_SetRef(rise_left, 150);
            Motor_Servo_SetRef(rise_right, 20);
            shoot_step = shoot_step_fourth_rise_home_final_wait;
            return;

        case shoot_step_fourth_rise_home_final_wait:
            if (!shoot_mode_delay_ms(500)) return;
            shoot_step = shoot_step_fourth_return;
            return;

        case shoot_step_fourth_return:
            Motor_DJI_SetRef(friction_l, 0);
            Motor_DJI_SetRef(friction_r, 0);
            if (!shoot_mode_3508_is_arrived(friction_l, 0) || !shoot_mode_3508_is_arrived(friction_r, 0)) return;
            Motor_Servo_SetRef(trigger, 5);
            shoot_step = shoot_step_fourth_fire_wait;
            return;

        case shoot_step_fourth_fire_wait:
            if (!shoot_mode_delay_ms(500)) return;
            enable_flag = 2;
            shoot_step  = shoot_step_idle;
            return;

        case shoot_step_idle:
        default:
            return;
        }
    }
    else
    {
        Motor_DJI_Stop(friction_l);
        Motor_DJI_Stop(friction_r);
        Motor_Servo_Stop(rise_left);
        Motor_Servo_Stop(rise_right);
        Motor_Servo_Stop(transfer);
        Motor_Servo_Stop(gripper);
        Motor_Servo_Stop(trigger);
        shoot_step = shoot_step_idle;
    }
}
