#include "vofa.h"
#include "bsp_fdcan.h"
#include "chassis_power_control.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#include "yaw_pitch_direct.h"
#include "shoot_task.h"
#include "detect_task.h"
#include "usart.h"
#include "stm32h7xx_hal.h"
#include <math.h>

#ifndef VOFA_SERVICE_CHASSIS_MOTOR_INDEX
#define VOFA_SERVICE_CHASSIS_MOTOR_INDEX 1U
#endif

typedef struct
{
    float fdata[6];
    uint8_t tail[4];
} VOFA_JustFloatFrame6_t;

static VOFA_JustFloatFrame6_t s_vofa_frame6 =
{
    .tail = {0x00, 0x00, 0x80, 0x7F}
};

static VOFA_JustFloatFrame_t s_vofa_frame12 =
{
    .tail = {0x00, 0x00, 0x80, 0x7F}
};

static void VOFA_Send6(float ch0, float ch1, float ch2, float ch3, float ch4, float ch5)
{
    if (huart7.gState != HAL_UART_STATE_READY)
    {
        return;
    }

    s_vofa_frame6.fdata[0] = ch0;
    s_vofa_frame6.fdata[1] = ch1;
    s_vofa_frame6.fdata[2] = ch2;
    s_vofa_frame6.fdata[3] = ch3;
    s_vofa_frame6.fdata[4] = ch4;
    s_vofa_frame6.fdata[5] = ch5;

    HAL_UART_Transmit_DMA(&huart7, (uint8_t *)&s_vofa_frame6, sizeof(s_vofa_frame6));
}

static void VOFA_Send12(float ch0, float ch1, float ch2, float ch3,
                        float ch4, float ch5, float ch6, float ch7,
                        float ch8, float ch9, float ch10, float ch11)
{
    if (huart7.gState != HAL_UART_STATE_READY)
    {
        return;
    }

    s_vofa_frame12.fdata[0] = ch0;
    s_vofa_frame12.fdata[1] = ch1;
    s_vofa_frame12.fdata[2] = ch2;
    s_vofa_frame12.fdata[3] = ch3;
    s_vofa_frame12.fdata[4] = ch4;
    s_vofa_frame12.fdata[5] = ch5;
    s_vofa_frame12.fdata[6] = ch6;
    s_vofa_frame12.fdata[7] = ch7;
    s_vofa_frame12.fdata[8] = ch8;
    s_vofa_frame12.fdata[9] = ch9;
    s_vofa_frame12.fdata[10] = ch10;
    s_vofa_frame12.fdata[11] = ch11;

    HAL_UART_Transmit_DMA(&huart7, (uint8_t *)&s_vofa_frame12, sizeof(s_vofa_frame12));
}

void VOFA_SendChassisMotorMeasure(uint8_t motor_idx)
{
    const motor_measure_t *motor;

    if (motor_idx >= VOFA_CHASSIS_MOTOR_COUNT)
    {
        return;
    }

    if (huart7.gState != HAL_UART_STATE_READY)
    {
        return;
    }

    motor = &CHASSIS_MOTOR_MEASURE[motor_idx];

    VOFA_Send6((float)motor->ecd,
               chassis_move.chassis_3508[motor_idx].speed_rad_s,
               chassis_move.chassis_3508[motor_idx].given_current_a,
               PowerLimit.P_bus,
               (float)motor->last_ecd,
               PowerLimit.P_origin);
}

void VOFA_SendChassisPowerDebug(uint8_t motor_idx)
{
    if (motor_idx >= VOFA_CHASSIS_MOTOR_COUNT)
    {
        return;
    }

    VOFA_Send6(PowerLimit.set_power,
               PowerLimit.P_origin,
               PowerLimit.K_Reduction,
               chassis_move.model_3508_out[motor_idx],
               (float)chassis_move.chassis_3508[motor_idx].give_current,
               PowerLimit.P_bus);
}

void VOFA_SendChassisPowerCurrentDebug(void)
{
    VOFA_Send6(PowerLimit.P_origin,
               chassis_move.chassis_3508[0].given_current_a,
               chassis_move.chassis_3508[1].given_current_a,
               chassis_move.chassis_3508[2].given_current_a,
               chassis_move.chassis_3508[3].given_current_a,
               PowerLimit.P_bus);
}

void VOFA_SendChassisSpeedAccel(void)
{
    VOFA_Send6(chassis_move.vx_plan,
               chassis_move.vy_plan,
               chassis_move.wz_plan,
               chassis_move.vx_plan_accel,
               chassis_move.vy_plan_accel,
               chassis_move.wz_plan_accel);
}

void VOFA_SendChassisRealSpeedCurrent(void)
{
    VOFA_Send6(chassis_move.vx,
               chassis_move.vy,
               chassis_move.chassis_3508[0].given_current_a,
               chassis_move.chassis_3508[1].given_current_a,
               chassis_move.chassis_3508[2].given_current_a,
               chassis_move.chassis_3508[3].given_current_a);
}

void VOFA_SendChassisAnglePidDebug(void)
{
    const float angle_error = rad_format(chassis_move.chassis_yaw_set -
                                         chassis_move.chassis_yaw);

    /* I0???I1?????I2????????I3????????
     * I4??P?I5??D?I6??wz???I7??wz?I8-I11?205/206/207/208??????A?? */
    VOFA_Send12((float)chassis_move.chassis_mode,
                angle_error,
                chassis_move.chassis_yaw_set_vel_filtered,
                chassis_move.chassis_yaw_rate_filtered,
                chassis_move.chassis_angle_pid.Pout,
                chassis_move.chassis_angle_pid.Dout,
                chassis_move.wz_set,
                chassis_move.wz,
                (float)chassis_move.chassis_3508[0].give_current * CHASSIS_CURRENT_CMD_TO_A,
                (float)chassis_move.chassis_3508[1].give_current * CHASSIS_CURRENT_CMD_TO_A,
                (float)chassis_move.chassis_3508[2].give_current * CHASSIS_CURRENT_CMD_TO_A,
                (float)chassis_move.chassis_3508[3].give_current * CHASSIS_CURRENT_CMD_TO_A);
}

void VOFA_SendChassisTranslatePairDebug(void)
{
    uint8_t motor_a = WHEEL_REAR_205;
    uint8_t motor_b = WHEEL_FRONT_207;

    if (fabsf(chassis_move.vx_plan) > fabsf(chassis_move.vy_plan))
    {
        motor_a = WHEEL_RIGHT_206;
        motor_b = WHEEL_LEFT_208;
    }

    VOFA_Send6(chassis_move.chassis_3508[motor_a].speed_set,
               chassis_move.chassis_3508[motor_a].speed,
               (float)chassis_move.chassis_3508[motor_a].give_current,
               chassis_move.chassis_3508[motor_b].speed_set,
               chassis_move.chassis_3508[motor_b].speed,
               (float)chassis_move.chassis_3508[motor_b].give_current);
}

void VOFA_SendChassisMotionDebug(void)
{
    float angle = chassis_move.chassis_yaw;
    float target_angle = chassis_move.chassis_yaw_set;

    if (chassis_move.chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        angle = chassis_move.chassis_relative_angle;
        target_angle = chassis_move.chassis_relative_angle_set;
    }

    VOFA_Send6(chassis_move.vx_plan,
               chassis_move.vy_plan,
               chassis_move.body_ff_ax,
               chassis_move.body_ff_ay,
               angle,
               target_angle);
}

void VOFA_SendGimbalFric(void)
{
    VOFA_Send6(shoot_task_control.fric1.speed_rpm,
               shoot_task_control.fric2.speed_rpm,
               shoot_task_control.fric1.give_current_a,
               shoot_task_control.fric2.give_current_a,
               shoot_task_control.bullet_speed_min_avg_rpm,
               shoot_task_control.estimated_bullet_speed_mps);
}

void VOFA_SendGimbalYaw(void)
{
    const gimbal_motor_t *yaw = &gimbal_control.gimbal_yaw_motor;

    VOFA_Send6(yaw->relative_angle_set,
               yaw->relative_angle,
               yaw->absolute_angle_set,
               yaw->absolute_angle,
               yaw->gyro,
               yaw->current_set);
}

void VOFA_SendGimbalPitch(void)
{
    const gimbal_motor_t *pitch = &gimbal_control.gimbal_pitch_motor;

    VOFA_Send6(pitch->relative_angle_set,
               pitch->relative_angle,
               pitch->absolute_angle_set,
               pitch->absolute_angle,
               pitch->gyro,
               pitch->current_set);
}

void VOFA_SendGimbalYawPitchHalf(void)
{
    const gimbal_motor_t *yaw = &gimbal_control.gimbal_yaw_motor;
    const gimbal_motor_t *pitch = &gimbal_control.gimbal_pitch_motor;

    VOFA_Send6(yaw->relative_angle_set,
               yaw->relative_angle,
               yaw->static_friction_comp,
               pitch->relative_angle_set,
               pitch->relative_angle,
               pitch->static_friction_comp);
}

void VOFA_SendGimbalStrum(void)
{
    const MITMeasure_t *strum = &MIT_MOTOR_MEASURE[SHOOT_STRUM_MIT_INDEX];

    /* ch0 位置反馈(rad)，ch1 目标速度(rad/s)，ch2 速度反馈(rad/s)
     * ch3 目标力矩(N*m)，ch4 力矩反馈(N*m)，ch5 电机温度(°C) */
    VOFA_Send6(strum->fdb.pos,
               strum->set.VEL,
               strum->fdb.vel,
               strum->set.TOR,
               strum->fdb.tor,
               strum->fdb.t_motor);
}

void VOFA_SendSmallYawDebug(void)
{
    const uint8_t small_yaw_feedback_ready =
        (uint8_t)(DJI_MOTOR_MEASURE[3].last_fdb_time != 0U &&
                  (HAL_GetTick() - DJI_MOTOR_MEASURE[3].last_fdb_time) <= 100U);
    const uint8_t yaw_mode_non_raw =
        (uint8_t)(gimbal_control.gimbal_yaw_motor.mode != GIMBAL_MOTOR_RAW);
    /* ch0-总模式，ch1-小yaw控制许可，ch2-结构体控制许可，ch3-yaw非RAW，
     * ch4-DBUS在线，ch5-小yaw反馈在线，ch6-小yaw电流(A)，
     * ch7-pitch反馈在线，ch8-pitch非RAW，ch9-pitch角度(rad)，
     * ch10-小yaw角度(rad)，ch11-小yaw目标(rad)。 */
    VOFA_Send12((float)gimbal_total_mode_get(),
                (float)gimbal_small_yaw_control_is_allowed(),
                (float)gimbal_control.small_yaw.control_allowed,
                (float)yaw_mode_non_raw,
                (float)(toe_is_error(DBUS_TOE) == 0U),
                (float)small_yaw_feedback_ready,
                gimbal_control.small_yaw.current_cmd_a,
                (float)(toe_is_error(PITCH_GIMBAL_MOTOR_TOE) == 0U),
                (float)(gimbal_control.gimbal_pitch_motor.mode != GIMBAL_MOTOR_RAW),
                gimbal_control.gimbal_pitch_motor.relative_angle,
                gimbal_control.small_yaw.angle_rad,
                gimbal_control.small_yaw.target_rad);
}

void VOFA_SendGimbalAutoSentinelDebug(void)
{
    const gimbal_motor_t *pitch = &gimbal_control.gimbal_pitch_motor;

    /* ch0-2 yaw目标角/实际角(rad)/电流指令(A)；ch3-5 pitch同序；ch6-7 yaw目标/实际角速度(rad/s)；ch8-9 pitch目标/实际角速度(rad/s)；ch10-11保留。 */
    VOFA_Send12(gimbal_control.small_yaw.target_rad,
                gimbal_control.small_yaw.angle_rad,
                gimbal_control.small_yaw.current_cmd_a,
                pitch->relative_angle_set,
                pitch->relative_angle,
                pitch->current_set,
                gimbal_control.small_yaw.speed_pid.set,
                gimbal_control.small_yaw.speed_feedback_rad_s,
                pitch->relative_angle_pid.out,
                pitch->relative_speed,
                0.0f,
                0.0f);
}

void VOFA_ServiceSend(void)
{
    VOFA_SendGimbalAutoSentinelDebug();
}
