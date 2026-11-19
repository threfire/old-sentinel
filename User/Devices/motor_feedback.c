/**
 * @file motor_feedback.c
 * @brief MIT 与 DJI 电机反馈解析。
 */
#include "motor_feedback.h"
#include "robot_param.h"
#include <stddef.h>

MITMeasure_t MIT_MOTOR_MEASURE[5];
motor_measure_t CHASSIS_MOTOR_MEASURE[8];
motor_measure_t DJI_MOTOR_MEASURE[8];

static float mit_vel_lpf[5];

static float motor_feedback_uint_to_float(int value, float min, float max, int bits)
{
    return (float)value * (max - min) / (float)((1 << bits) - 1) + min;
}

void motor_feedback_init(void)
{
    MIT_MOTOR_MEASURE[SHOOT_STRUM_MIT_INDEX].param.P_min = DM2325_P_MIN;
    MIT_MOTOR_MEASURE[SHOOT_STRUM_MIT_INDEX].param.P_max = DM2325_P_MAX;
    MIT_MOTOR_MEASURE[SHOOT_STRUM_MIT_INDEX].param.V_min = DM2325_V_MIN;
    MIT_MOTOR_MEASURE[SHOOT_STRUM_MIT_INDEX].param.V_max = DM2325_V_MAX;
    MIT_MOTOR_MEASURE[SHOOT_STRUM_MIT_INDEX].param.KP_min = DM2325_KP_MIN;
    MIT_MOTOR_MEASURE[SHOOT_STRUM_MIT_INDEX].param.KP_max = DM2325_KP_MAX;
    MIT_MOTOR_MEASURE[SHOOT_STRUM_MIT_INDEX].param.KD_min = DM2325_KD_MIN;
    MIT_MOTOR_MEASURE[SHOOT_STRUM_MIT_INDEX].param.KD_max = DM2325_KD_MAX;
    MIT_MOTOR_MEASURE[SHOOT_STRUM_MIT_INDEX].param.T_min = DM2325_T_MIN;
    MIT_MOTOR_MEASURE[SHOOT_STRUM_MIT_INDEX].param.T_max = DM2325_T_MAX;

    MIT_MOTOR_MEASURE[GIMBAL_YAW_MIT_INDEX].param.P_min = P_MIN;
    MIT_MOTOR_MEASURE[GIMBAL_YAW_MIT_INDEX].param.P_max = P_MAX;
    MIT_MOTOR_MEASURE[GIMBAL_YAW_MIT_INDEX].param.V_min = V_MIN;
    MIT_MOTOR_MEASURE[GIMBAL_YAW_MIT_INDEX].param.V_max = V_MAX;
    MIT_MOTOR_MEASURE[GIMBAL_YAW_MIT_INDEX].param.KP_min = KP_MIN;
    MIT_MOTOR_MEASURE[GIMBAL_YAW_MIT_INDEX].param.KP_max = KP_MAX;
    MIT_MOTOR_MEASURE[GIMBAL_YAW_MIT_INDEX].param.KD_min = KD_MIN;
    MIT_MOTOR_MEASURE[GIMBAL_YAW_MIT_INDEX].param.KD_max = KD_MAX;
    MIT_MOTOR_MEASURE[GIMBAL_YAW_MIT_INDEX].param.T_min = T_MIN;
    MIT_MOTOR_MEASURE[GIMBAL_YAW_MIT_INDEX].param.T_max = T_MAX;
}

void motor_feedback_update_mit(uint8_t index, const uint8_t data[8], uint32_t time_ms)
{
    MITMeasure_t *measure;
    float vel_raw;
    const float alpha = 1.0f;

    if (index >= 5U || data == NULL)
    {
        return;
    }

    measure = &MIT_MOTOR_MEASURE[index];
    measure->fdb.id = data[0] & 0x0F;
    measure->fdb.state = data[0] >> 4;
    measure->fdb.p_int = (data[1] << 8) | data[2];
    measure->fdb.v_int = (data[3] << 4) | (data[4] >> 4);
    measure->fdb.t_int = ((data[4] & 0x0F) << 8) | data[5];
    measure->fdb.pos = motor_feedback_uint_to_float(measure->fdb.p_int,
                                                    measure->param.P_min,
                                                    measure->param.P_max, 16);
    vel_raw = motor_feedback_uint_to_float(measure->fdb.v_int,
                                           measure->param.V_min,
                                           measure->param.V_max, 12);
    mit_vel_lpf[index] += alpha * (vel_raw - mit_vel_lpf[index]);
    measure->fdb.vel = mit_vel_lpf[index];
    measure->fdb.tor = motor_feedback_uint_to_float(measure->fdb.t_int,
                                                    measure->param.T_min,
                                                    measure->param.T_max, 12);
    measure->fdb.t_mos = (float)data[6];
    measure->fdb.t_motor = (float)data[7];
    measure->fdb.last_fdb_time = time_ms;
}

void motor_feedback_update_dji(motor_measure_t *measure, const uint8_t data[8], uint32_t time_ms)
{
    if (measure == NULL || data == NULL)
    {
        return;
    }

    measure->last_ecd = measure->ecd;
    measure->ecd = (uint16_t)((data[0] << 8) | data[1]);
    measure->speed_rpm = (int16_t)((data[2] << 8) | data[3]);
    measure->given_current = (int16_t)((data[4] << 8) | data[5]);
    measure->temperate = data[6];
    measure->last_fdb_time = time_ms;
}

motor_measure_t *get_chassis_motor_measure_point(uint8_t index)
{
    return &CHASSIS_MOTOR_MEASURE[index & 0x07U];
}
