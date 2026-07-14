#include "shoot_func.h"
#include "darts_def.h"
#include "darts_servo_func.h"
#include "module_offline.h"
#include "motor_def.h"
#include "motor_dji.h"
#include "motor_servo.h"
#include "stm32f4xx_hal_tim.h"
#include "tim.h"
#include "user_lib.h"
#include <stdint.h>
#include <stdlib.h>

#define LOG_TAG "app_shoot"
#define LOG_LVL LOG_LVL_DBG
#include "ulog_def.h"

DJI_Motor_t *friction_l = NULL;
DJI_Motor_t *friction_r = NULL;
DJI_Motor_t *loader     = NULL; // 扳机电机
Servo_Motor_t *rise_l   = NULL;
Servo_Motor_t *rise_r   = NULL;
Servo_Motor_t *tran     = NULL;
Servo_Motor_t *gripper  = NULL;
Servo_Motor_t *trigger  = NULL;

static const Darts_Servo_Step_t reload_sequence[] = {
    {
        .motor_mask = DARTS_SERVO_MASK(DARTS_SERVO_RISE_L) | DARTS_SERVO_MASK(DARTS_SERVO_RISE_R),
        .target_deg = {[DARTS_SERVO_RISE_L] = 125.0f, [DARTS_SERVO_RISE_R] = 25.0f},
        .wait_ms    = 500,
    },
    {
        .motor_mask = DARTS_SERVO_MASK(DARTS_SERVO_GRIPPER),
        .target_deg = {[DARTS_SERVO_GRIPPER] = 30.0f},
        .wait_ms    = 350,
    },
    {
        .motor_mask = DARTS_SERVO_MASK(DARTS_SERVO_GRIPPER),
        .target_deg = {[DARTS_SERVO_GRIPPER] = 110.0f},
        .wait_ms    = 350,
    },
    {
        .motor_mask = DARTS_SERVO_MASK(DARTS_SERVO_RISE_L) | DARTS_SERVO_MASK(DARTS_SERVO_RISE_R),
        .target_deg = {[DARTS_SERVO_RISE_L] = 75.0f, [DARTS_SERVO_RISE_R] = 75.0f},
        .wait_ms    = 500,
    },
};

static loader_mode_e last_load_mode = load_stop;

void shoot_init(void)
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
        .controller_init_config = {.lqr_init =
                                       {
                                           .K         = {0.0011f},
                                           .state_dim = 1,
                                       }},
        .setting_init_config =
            {
                .angle_feedback_source = 0,
                .speed_feedback_source = 0,
                .loop_type             = SPEED_LOOP,
                .feedback_reverse_flag = 0,
                .algorithm_type        = CONTROL_LQR,
            },
        .motor_init_info = {.motor_type = M3508, .gear_ratio = 19, .max_torque = 6, .torque_constant = 0.0016f},
    };
    // 左同步轮
    friction_config.transport_config.can.tx_id = 1;
    friction_l                                 = Motor_DJI_Init(&friction_config);
    if (friction_l == NULL)
    {
        LOG_E("friction_l init failed");
        return;
    }
    // 右同步轮
    friction_config.transport_config.can.tx_id     = 2; // 右同步轮,改txid和方向就行
    friction_config.offline_init_config.name       = "3508_2";
    friction_config.offline_init_config.beep_times = 6;
    friction_r                                     = Motor_DJI_Init(&friction_config);
    if (friction_r == NULL)
    {
        LOG_E("friction_r init failed");
        return;
    }

    // 镖盘电机
    Motor_Init_Config_s loader_config = {
        .offline_init_config =
            {
                .name       = "3508_3", // 设备名称
                .timeout_ms = 100,     // 超时时间
                .beep_times = 7,       // 蜂鸣次数
                .enable     = 1,       // 是否启用离线管理
            },
        .transport = MOTOR_TRANSPORT_CAN,
        .transport_config.can =
            {
                .hcan  = BSP_CAN_HANDLE1,
                .tx_id = 3,
            },
        .controller_init_config = {.lqr_init =
                                       {
                                           .K         = {0.005f}, // 0.0317
                                           .state_dim = 1,
                                       }},
        .setting_init_config =
            {
                .angle_feedback_source = 0,
                .speed_feedback_source = 0,
                .loop_type             = SPEED_LOOP,
                .feedback_reverse_flag = 0,
                .algorithm_type        = CONTROL_LQR,
            },
        .motor_init_info =
            {
                .motor_type      = M3508,
                .gear_ratio      = 19,
                .max_torque      = 6.0f,
                .torque_constant = 0.0016f,
            },
    };
    loader = Motor_DJI_Init(&loader_config);
    if (loader == NULL)
    {
        LOG_E("loader init failed");
        return;
    }

    //左抬升舵机
     Motor_Init_Config_s rise_servo_l_config = {
        .offline_init_config = {
            .name = "rise_servo_l", 
            .timeout_ms = 100,
            .beep_times = 2,
            .enable = 0,
        },
        .transport = MOTOR_TRANSPORT_PWM,
        .transport_config.pwm = {
            .htim = &htim1,
            .Channel = TIM_CHANNEL_1,
            .dutyx10 = 75, /* 7.5% = 1.5 ms，中位角 */
            .Mode = PWM_MODE_BLOCKING,
        },
        .controller_init_config = {0}, /* 舵机内部完成位置闭环 */
        .setting_init_config = {
            .loop_type = ANGLE_LOOP,
            .algorithm_type = CONTROL_PID,
            .enableflag = 0,
        },
        .motor_init_info = {
            .motor_type = SERVO_GENERIC,
            .gear_ratio = 1.0f,
        },
    };

    /* TIM1 当前每计数 10 us：50=0.5 ms，250=2.5 ms */
    rise_l = Motor_Servo_Init(&rise_servo_l_config,50,250,0,180);

    if (rise_l == NULL)
    {
        LOG_E("loader init failed");
        return;
    }
        //右抬升电机
         Motor_Init_Config_s rise_servo_r_config = {
        .offline_init_config = {
            .name = "rise_servo_r", 
            .timeout_ms = 100,
            .beep_times = 2,
            .enable = 0,
        },
        .transport = MOTOR_TRANSPORT_PWM,
        .transport_config.pwm = {
            .htim = &htim1,
            .Channel = TIM_CHANNEL_2,
            .dutyx10 = 75, /* 7.5% = 1.5 ms，中位角 */
            .Mode = PWM_MODE_BLOCKING,
        },
        .controller_init_config = {0}, /* 舵机内部完成位置闭环 */
        .setting_init_config = {
            .loop_type = ANGLE_LOOP,
            .algorithm_type = CONTROL_PID,
            .enableflag = 0,
        },
        .motor_init_info = {
            .motor_type = SERVO_GENERIC,
            .gear_ratio = 1.0f,
        },
    };

    /* TIM1 当前每计数 10 us：50=0.5 ms，250=2.5 ms */
    rise_r = Motor_Servo_Init(&rise_servo_r_config,50,250,0,180);

    if (rise_r == NULL)
    {
        LOG_E("rise_r init failed");
        return;
    }

         Motor_Init_Config_s tran_servo_config = {
        .offline_init_config = {
            .name = "tran_servo", 
            .timeout_ms = 100,
            .beep_times = 2,
            .enable = 0,
        },
        .transport = MOTOR_TRANSPORT_PWM,
        .transport_config.pwm = {
            .htim = &htim1,
            .Channel = TIM_CHANNEL_3,
            .dutyx10 = 75, /* 7.5% = 1.5 ms，中位角 */
            .Mode = PWM_MODE_BLOCKING,
        },
        .controller_init_config = {0}, /* 舵机内部完成位置闭环 */
        .setting_init_config = {
            .loop_type = ANGLE_LOOP,
            .algorithm_type = CONTROL_PID,
            .enableflag = 0,
        },
        .motor_init_info = {
            .motor_type = SERVO_GENERIC,
            .gear_ratio = 1.0f,
        },
    };

    /* TIM1 当前每计数 10 us：50=0.5 ms，250=2.5 ms */
    tran = Motor_Servo_Init(&tran_servo_config,50,250,0,270);

    if (tran == NULL)
    {
        LOG_E("loader init failed");
        return;
    }

         Motor_Init_Config_s gripper_servo_config = {
        .offline_init_config = {
            .name = "gripper_servo", 
            .timeout_ms = 100,
            .beep_times = 2,
            .enable = 0,
        },
        .transport = MOTOR_TRANSPORT_PWM,
        .transport_config.pwm = {
            .htim = &htim1,
            .Channel = TIM_CHANNEL_4,
            .dutyx10 = 110, /* 7.5% = 1.5 ms，中位角 */
            .Mode = PWM_MODE_BLOCKING,
        },
        .controller_init_config = {0}, /* 舵机内部完成位置闭环 */
        .setting_init_config = {
            .loop_type = ANGLE_LOOP,
            .algorithm_type = CONTROL_PID,
            .enableflag = 0,
        },
        .motor_init_info = {
            .motor_type = SERVO_GENERIC,
            .gear_ratio = 1.0f,
        },
    };

    /* TIM1 当前每计数 10 us：50=0.5 ms，250=2.5 ms */
    gripper = Motor_Servo_Init(&gripper_servo_config,50,250,0,90);

    if (gripper == NULL)
    {
        LOG_E("loader init failed");
        return;
    }

         Motor_Init_Config_s trigger_servo_config = {
        .offline_init_config = {
            .name = "trigger_servo", 
            .timeout_ms = 100,
            .beep_times = 2,
            .enable = 0,
        },
        .transport = MOTOR_TRANSPORT_PWM,
        .transport_config.pwm = {
            .htim = &htim8,
            .Channel = TIM_CHANNEL_1,
            .dutyx10 = 75, /* 7.5% = 1.5 ms，中位角 */
            .Mode = PWM_MODE_BLOCKING,
        },
        .controller_init_config = {0}, /* 舵机内部完成位置闭环 */
        .setting_init_config = {
            .loop_type = ANGLE_LOOP,
            .algorithm_type = CONTROL_PID,
            .enableflag = 0,
        },
        .motor_init_info = {
            .motor_type = SERVO_GENERIC,
            .gear_ratio = 1.0f,
        },
    };

    /* TIM1 当前每计数 10 us：50=0.5 ms，250=2.5 ms */
        trigger = Motor_Servo_Init(&trigger_servo_config,50,250,0,90);

    if (trigger == NULL)
    {
        LOG_E("loader init failed");
        return;
    }

    Darts_Servo_Sequence_Init(rise_l, rise_r, tran, gripper, trigger);
}
void shoot_func(Shoot_Ctrl_Cmd_t *shoot_cmd)
{
    Darts_Servo_Sequence_Update();

    if (shoot_cmd == NULL || shoot_cmd->shoot_mode != shoot_on)
    {
        Darts_Servo_Sequence_Cancel();
        last_load_mode = load_stop;
    }
    else if (shoot_cmd->load_mode != load_reload && last_load_mode == load_reload)
    {
        Darts_Servo_Sequence_Cancel();
        last_load_mode = shoot_cmd->load_mode;
    }

    if (!friction_l || !friction_r || !loader || !rise_l || !rise_r || !tran || !gripper || !trigger) return;

    if (shoot_cmd != NULL)
    {
        // 从cmd获取控制数据
        if (!Module_Offline_get_device_status(friction_l->base.offline_dev) && !Module_Offline_get_device_status(friction_r->base.offline_dev) &&
            !Module_Offline_get_device_status(loader->base.offline_dev))
        {
            if (shoot_cmd->shoot_mode == shoot_on)
            {
                Motor_DJI_Start(friction_l);
                Motor_DJI_Start(friction_r);
                Motor_DJI_Start(loader);
                Motor_Servo_Start(rise_l);
                Motor_Servo_Start(rise_r);
                Motor_Servo_Start(tran);
                Motor_Servo_Start(gripper);
                Motor_Servo_Start(trigger);
                    switch (shoot_cmd->load_mode)
                    {
                    case load_stop:
                        Motor_DJI_SetRef(friction_l,0);
                        Motor_DJI_SetRef(friction_r,0);
                        Motor_DJI_SetRef(loader, 0);
                        Motor_Servo_SetRef(rise_l,75);
                        Motor_Servo_SetRef(rise_r,75);
                        Motor_Servo_SetRef(tran,75);
                        Motor_Servo_SetRef(gripper,110);
                        Motor_Servo_SetRef(trigger,75);
                        break;
                    case load_start :
                        Motor_DJI_SetRef(loader, 1800 * RPM_2_RAD_PER_SEC);
                        Motor_Servo_SetRef(rise_l,125);
                        Motor_Servo_SetRef(rise_r,25);
                        Motor_Servo_SetRef(tran,75);
                        Motor_Servo_SetRef(gripper,30);
                        Motor_Servo_SetRef(trigger,75);
                        break;
                    case load_get :
                        Motor_DJI_SetRef(friction_l, 1800 * RPM_2_RAD_PER_SEC);
                        Motor_DJI_SetRef(friction_r, -1800 * RPM_2_RAD_PER_SEC);
                        break;
                    case load_restart :
                        Motor_DJI_SetRef(friction_l, -1800 * RPM_2_RAD_PER_SEC);
                        Motor_DJI_SetRef(friction_r, 1800 * RPM_2_RAD_PER_SEC);
                        Motor_DJI_SetRef(loader, -1800 * RPM_2_RAD_PER_SEC);
                        Motor_Servo_SetRef(rise_l,125);
                        Motor_Servo_SetRef(rise_r,25);
                        Motor_Servo_SetRef(tran,75);
                        Motor_Servo_SetRef(gripper,110);
                        Motor_Servo_SetRef(trigger,75);
                        break;
                    case load_reload :
                        Motor_DJI_SetRef(friction_l, 0);
                        Motor_DJI_SetRef(friction_r, 0);
                        Motor_DJI_SetRef(loader, 0);
                        if (last_load_mode != load_reload)
                        {
                            Darts_Servo_Sequence_Start(reload_sequence,
                                                       (uint8_t)(sizeof(reload_sequence) / sizeof(reload_sequence[0])));
                        }
                        break;
                    case load_ready :
                        Motor_DJI_SetRef(friction_l, 0);
                        Motor_DJI_SetRef(friction_r, 0);
                        Motor_DJI_SetRef(loader, 0);
                        Motor_Servo_SetRef(rise_l,125);
                        Motor_Servo_SetRef(rise_r,25);
                        Motor_Servo_SetRef(tran,75);
                        Motor_Servo_SetRef(gripper,110);
                        Motor_Servo_SetRef(trigger,75);    
                    default:
                        break;
                    }
                    last_load_mode = shoot_cmd->load_mode;
            }
                else // 关闭摩擦轮
                {
                    Motor_DJI_SetRef(friction_l, 0);
                    Motor_DJI_SetRef(friction_r, 0);
                    Motor_DJI_SetRef(loader, 0);
                }
            }
            else
            {
                Motor_DJI_Stop(friction_l);
                Motor_DJI_Stop(friction_r);
                Motor_DJI_Stop(loader);
            }
        }
        else
        {
            Motor_DJI_Stop(friction_l);
            Motor_DJI_Stop(friction_r);
            Motor_DJI_Stop(loader);
        }
    }
