#ifndef TEST_MODULE_REMOTE_H
#define TEST_MODULE_REMOTE_H

#include <stdint.h>

#define SBUS_CHX_BIAS 1024
#define SBUS_CHX_UP   240
#define SBUS_CHX_DOWN 1807

uint8_t Module_Remote_get_offline_status(void);
int16_t Module_Remote_get_channel(uint8_t channel);

#endif
