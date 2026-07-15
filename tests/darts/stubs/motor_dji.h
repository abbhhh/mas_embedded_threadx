#ifndef TEST_MOTOR_DJI_H
#define TEST_MOTOR_DJI_H

typedef struct
{
    struct
    {
        struct
        {
            float ref;
        } controller;
        struct
        {
            float total_angle;
            float speed_rad;
        } measure;
    } base;
} DJI_Motor_t;

#endif /* TEST_MOTOR_DJI_H */
