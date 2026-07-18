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
static float          friction_l_home_angle;
static float          friction_r_home_angle;
static bool           friction_home_captured;

typedef enum
{
    shoot_step_idle = 0,
    shoot_step_1,
    shoot_step_2,
    shoot_step_3,
    shoot_step_4,
    shoot_step_5,
    shoot_step_6,
    shoot_step_7,
    shoot_step_8,
    shoot_step_9,
    shoot_step_10,
    shoot_step_11,
    shoot_step_12,
    shoot_step_13,
    shoot_step_14,
    shoot_step_15,
    shoot_step_16,
    shoot_step_17,
    shoot_step_18,
    shoot_step_19,
    shoot_step_20,
    shoot_step_21,
    shoot_step_22,
    shoot_step_23,
    shoot_step_24,
    shoot_step_25,
    shoot_step_26,
    shoot_step_27,
    shoot_step_28,
    shoot_step_29,
    shoot_step_30,
    shoot_step_31,
    shoot_step_32,
    shoot_step_33,
    shoot_step_34,
    shoot_step_35,
    shoot_step_36,
    shoot_step_37,
    shoot_step_38,
    shoot_step_39,
    shoot_step_40,
    shoot_step_41,
    shoot_step_42,
    shoot_step_43,
    shoot_step_44,
    shoot_step_45,
    shoot_step_46,
    shoot_step_47,
    shoot_step_48,
    shoot_step_49,
    shoot_step_50,
    shoot_step_51,
    shoot_step_52,
    shoot_step_53,
    shoot_step_54,
    shoot_step_55,
    shoot_step_56,
    shoot_step_57,
    shoot_step_58,
    shoot_step_59,
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
                .speed_PID = 
                    {
                        .Kp = 0.0095f,           
                        .Ki = 0.0004f,
                        .Kd = 0.000f,          
                        .MaxOut = 4.0f,        
                        .DeadBand = 0.0f,
                        .Improve = PID_Integral_Limit,
                        .IntegralLimit = 0.3f,
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

    Motor_Servo_Start(rise_left);
    Motor_Servo_Start(rise_right);
    Motor_Servo_Start(transfer);
    Motor_Servo_Start(gripper);
    Motor_Servo_Start(trigger);
    Motor_Servo_SetRef(rise_left, left_rise_sation);
    Motor_Servo_SetRef(rise_right, right_rise_sation);
    Motor_Servo_SetRef(transfer, transfer_left);
    Motor_Servo_SetRef(gripper, gripper_in);
    Motor_Servo_SetRef(trigger, trigger_fire );

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
        if (!friction_home_captured)
        {
            friction_l_home_angle = friction_l->base.measure.total_angle;
            friction_r_home_angle = friction_r->base.measure.total_angle;
            friction_home_captured = true;
        }

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
            if (shoot_step == shoot_step_idle) shoot_step = shoot_step_13;
        }
        else if (shoot_cmd->shoot_mode == shoot_start_2)
        {
            if (enable_flag != 1) return;
            if (shoot_step == shoot_step_idle) shoot_step = shoot_step_28;
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

        Motor_DJI_Start(friction_l);
        Motor_DJI_Start(friction_r);
        Motor_Servo_Start(rise_left);
        Motor_Servo_Start(rise_right);
        Motor_Servo_Start(transfer);
        Motor_Servo_Start(gripper);
        Motor_Servo_Start(trigger);

        switch (shoot_step)
        {

        case shoot_step_1:
            if (!shoot_mode_delay_ms(1000)) return;
            shoot_step = shoot_step_3;
            return;

        case shoot_step_3:
            Motor_DJI_SetRef(friction_l, friction_high_speed);
            Motor_DJI_SetRef(friction_r, -friction_high_speed);
            if (!shoot_mode_delay_ms(friction_time)) return;
            shoot_step = shoot_step_5;
            return;

        case shoot_step_5:
            Motor_DJI_SetRef(friction_l, friction_low_speed);
            Motor_DJI_SetRef(friction_r, -friction_low_speed);
            if (!shoot_mode_delay_ms(friction_low_time)) return;
            shoot_step = shoot_step_7;
            return;
        case shoot_step_7:
            Motor_DJI_SetRef(friction_l, 0);
            Motor_DJI_SetRef(friction_r, 0);
            Motor_Servo_SetRef(trigger, trigger_ready);
            if (!shoot_mode_delay_ms(servo_time)) return;
            shoot_step = shoot_step_9;
            return;
        case shoot_step_9:
            Motor_DJI_SetRef(friction_l, -friction_low_speed);
            Motor_DJI_SetRef(friction_r, friction_low_speed);
            if (!shoot_mode_delay_ms(friction_low_time)) return;
            shoot_step = shoot_step_10;
            return;

        case shoot_step_10:
            Motor_DJI_SetRef(friction_l, -friction_high_speed);
            Motor_DJI_SetRef(friction_r, friction_high_speed);
            if (!shoot_mode_delay_ms(friction_time)) return;
            shoot_step = shoot_step_11;
            return;
        case shoot_step_11:
            Motor_DJI_SetRef(friction_l, 0);
            Motor_DJI_SetRef(friction_r, 0);
            if (!shoot_mode_delay_ms(servo_time)) return;
            shoot_step = shoot_step_12;
            return;
        case shoot_step_12:
            Motor_Servo_SetRef(trigger, trigger_fire);
            if (!shoot_mode_delay_ms(servo_time)) return;
            enable_flag = 2;
            shoot_step  = shoot_step_idle;
            return;
        case shoot_step_13:
            Motor_DJI_SetRef(friction_l, friction_high_speed);
            Motor_DJI_SetRef(friction_r, -friction_high_speed);
            Motor_Servo_SetRef(transfer,transfer_medium);
            if (!shoot_mode_delay_ms(friction_time)) return;
            shoot_step = shoot_step_14;
            return;

        case shoot_step_14:
            Motor_DJI_SetRef(friction_l, friction_low_speed);
            Motor_DJI_SetRef(friction_r, -friction_low_speed);
            if(!shoot_mode_delay_ms(friction_low_time)) return;
            shoot_step = shoot_step_15;
            return;
        case shoot_step_15:
            Motor_Servo_SetRef(rise_left, left_low_sation);
            Motor_Servo_SetRef(rise_right, right_low_sation);
            Motor_DJI_SetRef(friction_l, 0);
            Motor_DJI_SetRef(friction_r, 0);
            if (!shoot_mode_delay_ms(servo_time)) return;
            shoot_step = shoot_step_16;
            return;

        case shoot_step_16:
            Motor_DJI_SetRef(friction_l,-friction_low_speed);
            Motor_DJI_SetRef(friction_r,friction_low_speed);
            if (!shoot_mode_delay_ms(friction_low_time)) return;
            shoot_step = shoot_step_17;
            return;

        case shoot_step_17:
            shoot_step = shoot_step_18;
            return;

        case shoot_step_18:
            Motor_Servo_SetRef(gripper, gripper_out);
            Motor_Servo_SetRef(rise_left, left_rise_sation);
            Motor_Servo_SetRef(rise_right, right_rise_sation);
            if (!shoot_mode_delay_ms(servo_time)) return;
            shoot_step = shoot_step_19;
            return;

        case shoot_step_19:
            Motor_DJI_SetRef(friction_l, -friction_high_speed);
            Motor_DJI_SetRef(friction_r, friction_high_speed);
            if (!shoot_mode_delay_ms(friction_time)) return;
            shoot_step = shoot_step_20;
            return;

        case shoot_step_20:            
            Motor_DJI_SetRef(friction_l, 0);
            Motor_DJI_SetRef(friction_r, 0);
            Motor_Servo_SetRef(transfer,transfer_left);
            Motor_Servo_SetRef(gripper,gripper_out);
            if (!shoot_mode_delay_ms(servo_time)) return;
            shoot_step = shoot_step_21;
            return;

        case shoot_step_21:            
            Motor_DJI_SetRef(friction_l, friction_high_speed);
            Motor_DJI_SetRef(friction_r, -friction_high_speed);
            if (!shoot_mode_delay_ms(friction_time)) return;
            shoot_step = shoot_step_22;
            return;

        case shoot_step_22:
            Motor_DJI_SetRef(friction_l, friction_low_speed);
            Motor_DJI_SetRef(friction_r, -friction_low_speed);
            if (!shoot_mode_delay_ms(friction_low_time)) return;
            shoot_step = shoot_step_23;
            return;

            /*第二发开火*/

        case shoot_step_23:
            Motor_DJI_SetRef(friction_l, 0);
            Motor_DJI_SetRef(friction_r, 0);
            //Motor_Servo_SetRef(trigger, trigger_ready);
            if (!shoot_mode_delay_ms(servo_time)) return;
            shoot_step = shoot_step_24;
            return;
        case shoot_step_24:
            Motor_DJI_SetRef(friction_l, -friction_low_speed);
            Motor_DJI_SetRef(friction_r, friction_low_speed);
            if (!shoot_mode_delay_ms(friction_low_time)) return;
            shoot_step = shoot_step_25;
            return;

        case shoot_step_25:
            Motor_DJI_SetRef(friction_l, -friction_high_speed);
            Motor_DJI_SetRef(friction_r, friction_high_speed);
            if (!shoot_mode_delay_ms(friction_time)) return;
            shoot_step = shoot_step_26;
            return;

        case shoot_step_26:            
            Motor_DJI_SetRef(friction_l, 0);
            Motor_DJI_SetRef(friction_r, 0);
            if (!shoot_mode_delay_ms(servo_time)) return;
            shoot_step = shoot_step_27;
            return;

        case shoot_step_27:
            //Motor_Servo_SetRef(trigger, trigger_fire);
            if (!shoot_mode_delay_ms(servo_time)) return;
            enable_flag = 2;
            shoot_step  = shoot_step_idle;
            return;
            /*第三发蓄力*/

        case shoot_step_28:
            Motor_Servo_SetRef(rise_left,left_low_sation);
            Motor_Servo_SetRef(rise_left,right_low_sation);
            if (!shoot_mode_delay_ms(servo_time)) return;
            shoot_step = shoot_step_29;
            return;
        case shoot_step_29:
            Motor_Servo_SetRef(gripper, gripper_in);
            if (!shoot_mode_delay_ms(servo_time)) return;
            shoot_step = shoot_step_30;
            return;
            /*第三发装载*/

        case shoot_step_30:
            Motor_Servo_SetRef(rise_left,left_rise_sation);
            Motor_Servo_SetRef(rise_left,right_rise_sation);
            Motor_Servo_SetRef(transfer,transfer_medium);
            if (!shoot_mode_delay_ms(servo_time)) return;
            shoot_step = shoot_step_31;
            return;
        case shoot_step_31:
            Motor_DJI_SetRef(friction_l, friction_high_speed);
            Motor_DJI_SetRef(friction_r, -friction_high_speed);
            if (!shoot_mode_delay_ms(friction_time)) return;
            shoot_step = shoot_step_32;
            return;

        case shoot_step_32:
            Motor_DJI_SetRef(friction_l, friction_low_speed);
            Motor_DJI_SetRef(friction_r, -friction_low_speed);
            if (!shoot_mode_delay_ms(friction_low_time)) return;
            shoot_step = shoot_step_33;
            return;

        case shoot_step_33:            
            Motor_DJI_SetRef(friction_l, 0);
            Motor_DJI_SetRef(friction_r, 0);
            Motor_Servo_SetRef(rise_left, left_low_sation);
            Motor_Servo_SetRef(rise_right, right_low_sation);
            if (!shoot_mode_delay_ms(servo_time)) return;
            shoot_step = shoot_step_34;
            return;

        case shoot_step_34:
            Motor_DJI_SetRef(friction_l, -friction_low_speed);
            Motor_DJI_SetRef(friction_r, friction_low_speed);
            if (!shoot_mode_delay_ms(friction_low_time)) return;
            shoot_step = shoot_step_35;
            return;

        case shoot_step_35:
            Motor_Servo_SetRef(rise_left,left_rise_sation);
            Motor_Servo_SetRef(rise_left,right_rise_sation);
            Motor_Servo_SetRef(gripper, gripper_out);
            if (!shoot_mode_delay_ms(servo_time)) return;
            shoot_step = shoot_step_36;
            return;

        case shoot_step_36:
            Motor_DJI_SetRef(friction_l, -friction_high_speed);
            Motor_DJI_SetRef(friction_r, friction_high_speed);
            if (!shoot_mode_delay_ms(friction_time)) return;
            shoot_step = shoot_step_37;
            return;

        case shoot_step_37:
            Motor_DJI_SetRef(friction_l,0);
            Motor_DJI_SetRef(friction_r, 0);
            Motor_Servo_SetRef(transfer,transfer_right);
            if (!shoot_mode_delay_ms(servo_time)) return;
            shoot_step = shoot_step_38;
            return;

        case shoot_step_38:
            Motor_DJI_SetRef(friction_l, friction_high_speed);
            Motor_DJI_SetRef(friction_r, -friction_high_speed);
            if (!shoot_mode_delay_ms(friction_time)) return;
            shoot_step = shoot_step_39;
            return;

        case shoot_step_39:
            Motor_DJI_SetRef(friction_l, friction_low_speed);
            Motor_DJI_SetRef(friction_r, -friction_low_speed);
            if (!shoot_mode_delay_ms(friction_low_time)) return;
            shoot_step = shoot_step_40;
            return;

        case shoot_step_40:
            Motor_DJI_SetRef(friction_l,0);
            Motor_DJI_SetRef(friction_r, 0);
            Motor_Servo_SetRef(rise_left, left_low_sation);
            Motor_Servo_SetRef(rise_right, right_low_sation);
            Motor_Servo_SetRef(trigger,trigger_ready);
            if (!shoot_mode_delay_ms(servo_time)) return;
            shoot_step = shoot_step_41;
            return;

        case shoot_step_41:
            Motor_DJI_SetRef(friction_l, -friction_low_speed);
            Motor_DJI_SetRef(friction_r, friction_low_speed);
            if (!shoot_mode_delay_ms(friction_low_time)) return;
            shoot_step = shoot_step_42;
            return;

        case shoot_step_42:
            Motor_DJI_SetRef(friction_l, -friction_high_speed);
            Motor_DJI_SetRef(friction_r, friction_high_speed);
            if (!shoot_mode_delay_ms(friction_time)) return;
            shoot_step = shoot_step_43;
            return;

        case shoot_step_43:
            Motor_DJI_SetRef(friction_l, 0);
            Motor_DJI_SetRef(friction_r, 0);
            Motor_Servo_SetRef(gripper,gripper_in);
            if (!shoot_mode_delay_ms(servo_time)) return;
            shoot_step = shoot_step_44;
            return;
            /*第三发开火*/

        case shoot_step_44:
            Motor_Servo_SetRef(trigger,trigger_fire);
            if (!shoot_mode_delay_ms(servo_time)) return;
            shoot_step = shoot_step_45;
            return;

        case shoot_step_45:
            Motor_Servo_SetRef(rise_left,left_rise_sation);
            Motor_Servo_SetRef(rise_left,right_rise_sation);
            Motor_Servo_SetRef(transfer,transfer_medium);
            if (!shoot_mode_delay_ms(servo_time)) return;
            shoot_step = shoot_step_46;
            return;

        case shoot_step_46:            
            Motor_DJI_SetRef(friction_l, friction_high_speed);
            Motor_DJI_SetRef(friction_r, -friction_high_speed);
            if (!shoot_mode_delay_ms(friction_time)) return;
            shoot_step = shoot_step_47;
            return;

        case shoot_step_47:
            Motor_DJI_SetRef(friction_l, friction_low_speed);
            Motor_DJI_SetRef(friction_r, -friction_low_speed);
            if (!shoot_mode_delay_ms(friction_low_time)) return;
            shoot_step = shoot_step_48;
            return;
            /*第四发蓄力*/

        case shoot_step_48:
            Motor_Servo_SetRef(rise_left,left_low_sation);
            Motor_Servo_SetRef(rise_left,right_low_sation);
            Motor_DJI_SetRef(friction_l, 0);
            Motor_DJI_SetRef(friction_r, 0);
            if (!shoot_mode_delay_ms(servo_time)) return;
            shoot_step = shoot_step_49;
            return;
        case shoot_step_49:
            Motor_DJI_SetRef(friction_l, -friction_low_speed);
            Motor_DJI_SetRef(friction_r, friction_low_speed);
            if (!shoot_mode_delay_ms(friction_low_time)) return;
            shoot_step = shoot_step_50;
            return;
            /*第四发装载*/

        case shoot_step_50:
            Motor_Servo_SetRef(rise_left,left_rise_sation);
            Motor_Servo_SetRef(rise_left,right_rise_sation);
            Motor_Servo_SetRef(gripper, gripper_out);
            if (!shoot_mode_delay_ms(servo_time)) return;
            shoot_step = shoot_step_51;
            return;

        case shoot_step_51:
            Motor_DJI_SetRef(friction_l, -friction_high_speed);
            Motor_DJI_SetRef(friction_r, friction_high_speed);
            if (!shoot_mode_delay_ms(friction_time)) return;
            shoot_step = shoot_step_52;
            return;

        case shoot_step_52:
            Motor_DJI_SetRef(friction_l, 0);
            Motor_DJI_SetRef(friction_r, 0);
            if (!shoot_mode_delay_ms(friction_time)) return;
            shoot_step = shoot_step_53;
            return;

        case shoot_step_53:
            Motor_DJI_SetRef(friction_l, friction_high_speed);
            Motor_DJI_SetRef(friction_r, -friction_high_speed);
            if (!shoot_mode_delay_ms(friction_time)) return;
            shoot_step = shoot_step_54;
            return;

        case shoot_step_54:
            Motor_DJI_SetRef(friction_l, friction_low_speed);
            Motor_DJI_SetRef(friction_r, -friction_low_speed);
            if (!shoot_mode_delay_ms(friction_low_time)) return;
            shoot_step = shoot_step_55;
            return;

        case shoot_step_55:
            Motor_DJI_SetRef(friction_l, 0);
            Motor_DJI_SetRef(friction_r, 0);
            Motor_Servo_SetRef(trigger,trigger_ready);
            if (!shoot_mode_delay_ms(servo_time)) return;
            shoot_step = shoot_step_56;
            return;

        case shoot_step_56:
            Motor_DJI_SetRef(friction_l, -friction_low_speed);
            Motor_DJI_SetRef(friction_r, friction_low_speed);
            if (!shoot_mode_delay_ms(friction_low_time)) return;
            shoot_step = shoot_step_57;
            return;

        case shoot_step_57:
            Motor_DJI_SetRef(friction_l, -friction_high_speed);
            Motor_DJI_SetRef(friction_r, friction_high_speed);
            if (!shoot_mode_delay_ms(friction_time)) return;
            shoot_step = shoot_step_58;
            return;

        case shoot_step_58:
            Motor_DJI_SetRef(friction_l, 0);
            Motor_DJI_SetRef(friction_r, 0);
            if (!shoot_mode_delay_ms(servo_time)) return;
            shoot_step = shoot_step_59;
            return;

        case shoot_step_59:
            Motor_Servo_SetRef(trigger,trigger_fire);
            if (!shoot_mode_delay_ms(servo_time)) return;
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
