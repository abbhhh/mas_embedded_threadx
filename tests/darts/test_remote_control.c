#include "robot_func.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

static uint8_t offline_status = 1;
static int16_t channel_5;
static int16_t channel_6;
static uint8_t salvo_index;
static uint8_t channel_5_read;
static uint8_t channel_6_read;

uint8_t Module_Remote_get_offline_status(void) { return offline_status; }

int16_t Module_Remote_get_channel(uint8_t channel)
{
    if (channel == 5U)
    {
        channel_5_read = 1U;
        return channel_5;
    }
    if (channel == 6U)
    {
        channel_6_read = 1U;
        return channel_6;
    }
    assert(0);
    return 0;
}

uint8_t shoot_mode_get_salvo_index(void) { return salvo_index; }

static void reset_observation(void)
{
    channel_5_read = 0U;
    channel_6_read = 0U;
}

static void assert_command(uint8_t index, int16_t ch5, int16_t ch6, shoot_cmd_e expected)
{
    Shoot_Ctrl_Cmd_t command = {.shoot_cmd = shoot_stop};
    salvo_index             = index;
    channel_5               = ch5;
    channel_6               = ch6;
    reset_observation();

    RemoteControlSet(&command);
    assert(command.shoot_cmd == expected);
}

int main(void)
{
    assert_command(0U, 1000, 1500, shoot_stop);
    assert(channel_5_read == 1U);
    assert(channel_6_read == 0U);

    assert_command(0U, 1001, 0, shoot_start);
    assert(channel_5_read == 1U);
    assert(channel_6_read == 0U);

    assert_command(1U, 1500, 1000, shoot_stop);
    assert(channel_5_read == 0U);
    assert(channel_6_read == 1U);

    assert_command(1U, 0, 1001, shoot_start);
    assert(channel_5_read == 0U);
    assert(channel_6_read == 1U);

    assert_command(2U, 1500, 1500, shoot_stop);
    assert(channel_5_read == 0U);
    assert(channel_6_read == 0U);

    offline_status          = 0U;
    Shoot_Ctrl_Cmd_t command = {.shoot_cmd = shoot_start};
    reset_observation();
    RemoteControlSet(&command);
    assert(command.shoot_cmd == shoot_stop);
    assert(channel_5_read == 0U);
    assert(channel_6_read == 0U);

    puts("remote channel 5/6 tests passed");
    return 0;
}
