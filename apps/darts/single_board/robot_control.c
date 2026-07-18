#include "robot_control.h"

#include "bsp_def.h"
#include "darts_def.h"
#include "robot_func.h"
#include "shoot_func.h"
#include "tx_api.h"

#define LOG_TAG "app_robot_control"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

#define ROBOT_CONTROL_TASK_STACK_SIZE  1024U
#define ROBOT_CONTROL_TASK_PRIORITY    30U
#define ROBOT_CONTROL_TASK_SLEEP_TICKS 2U

static TX_THREAD                  robot_control_thread;
APPS_STACK_SECTION static uint8_t robot_control_thread_stack[ROBOT_CONTROL_TASK_STACK_SIZE];
static Shoot_Ctrl_Cmd_t           shoot_cmd;
extern volatile uint8_t           enable_flag;

static void robot_control_task(ULONG thread_input)
{
    (void)thread_input;

    while (1)
    {
        // RemoteControlSet(&shoot_cmd);
        if (enable_flag == 0)
            shoot_cmd.shoot_mode = shoot_start_1;
        else if (enable_flag == 1)
            shoot_cmd.shoot_mode = shoot_start_2;
        else
            shoot_cmd.shoot_mode = shoot_off;

        shoot_func(&shoot_cmd);
        tx_thread_sleep(ROBOT_CONTROL_TASK_SLEEP_TICKS);
    }
}

void robot_control_init(void)
{
    shoot_init();

    UINT status =
        tx_thread_create(&robot_control_thread, "robot_control_thread", robot_control_task, 0, robot_control_thread_stack,
                         ROBOT_CONTROL_TASK_STACK_SIZE, ROBOT_CONTROL_TASK_PRIORITY, ROBOT_CONTROL_TASK_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        LOG_E("robot_control_task failed");
        return;
    }

    LOG_I("robot_control init success");
}
