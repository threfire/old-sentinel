/**
 * @file motor_feedback.h
 * @brief 电机反馈结构体与解析接口。
 * @note 全局反馈数组保留原名，供外部调用和 Keil Watch 直接查看。
 */
#ifndef MOTOR_FEEDBACK_H
#define MOTOR_FEEDBACK_H

#include <stdint.h>

typedef struct
{
    __packed struct
    {
        int id;
        int state;
        int p_int;
        int v_int;
        int t_int;
        int kp_int;
        int kd_int;
        float pos;
        float vel;
        float tor;
        float Kp;
        float Kd;
        float t_mos;
        float t_motor;
        uint32_t last_fdb_time;
    } fdb;
    __packed struct
    {
        float KP;
        float KD;
        float POS;
        float VEL;
        float TOR;
    } set;
    __packed struct
    {
        float P_min;
        float P_max;
        float V_min;
        float V_max;
        float KP_min;
        float KP_max;
        float KD_min;
        float KD_max;
        float T_min;
        float T_max;
    } param;
} MITMeasure_t;

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
    uint32_t last_fdb_time;
} motor_measure_t;

extern MITMeasure_t MIT_MOTOR_MEASURE[5];
extern motor_measure_t CHASSIS_MOTOR_MEASURE[8];
extern motor_measure_t DJI_MOTOR_MEASURE[8];

void motor_feedback_init(void);
void motor_feedback_update_mit(uint8_t index, const uint8_t data[8], uint32_t time_ms);
void motor_feedback_update_dji(motor_measure_t *measure, const uint8_t data[8], uint32_t time_ms);
motor_measure_t *get_chassis_motor_measure_point(uint8_t index);

#endif
