/**
  * @file       gimbal_behaviour.c
  * @brief      云台行为模式控制
  * @note       将遥控器、键鼠、自瞄和初始化状态转换为云台行为模式与角度增量。
  */
#include "gimbal_behaviour.h"
#include "bsp_fdcan.h"
#include "detect_task.h"
#include "user_lib.h"
#include "stm32h7xx_hal.h"
#include <math.h>

#ifndef SHOOT_RC_MODE_CHANNEL
#define SHOOT_RC_MODE_CHANNEL 1
#endif

volatile gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE;
#define gimbal_total_mode            (gimbal_control.behaviour_state.total_mode)
#define gimbal_friction_enabled      (gimbal_control.behaviour_state.friction_enabled)
#define gimbal_strum_enabled         (gimbal_control.behaviour_state.strum_enabled)

static void gimbal_remote_state_update(const gimbal_control_t *control)
{
    static bool initialized = false;
    static uint16_t last_s0 = RC_SW_MID;
    static uint16_t last_s1 = RC_SW_MID;
    static uint8_t s0_up_count = 0U;
    static uint8_t s1_down_count = 0U;
    static uint32_t last_s0_up_tick = 0U;
    static uint32_t last_s1_down_tick = 0U;
    static uint32_t s1_down_tick = 0U;
    uint16_t s0;
    uint16_t s1;
    uint32_t now;

    if (control == 0 || control->gimbal_rc_ctrl == 0)
    {
        return;
    }

    s0 = control->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL];
    s1 = control->gimbal_rc_ctrl->rc.s[SHOOT_RC_MODE_CHANNEL];
    now = HAL_GetTick();

    if (!initialized)
    {
        initialized = true;
        last_s0 = s0;
        last_s1 = s1;
    }

    if (switch_is_up(s0) && switch_is_mid(last_s0))
    {
        if (s0_up_count == 1U && (now - last_s0_up_tick) <= GIMBAL_AUTO_SENTINEL_REPEAT_WINDOW_MS)
        {
            gimbal_total_mode = GIMBAL_TOTAL_AUTO_SENTINEL;
            s0_up_count = 0U;
        }
        else
        {
            s0_up_count = 1U;
            last_s0_up_tick = now;
        }
    }
    if (s0_up_count == 1U &&
        (now - last_s0_up_tick) > GIMBAL_AUTO_SENTINEL_REPEAT_WINDOW_MS)
    {
        s0_up_count = 0U;
    }

    if (switch_is_down(s0))
    {
        s0_up_count = 0U;
        gimbal_total_mode = GIMBAL_TOTAL_MOTIONLESS;
    }
    else if (switch_is_mid(s0))
    {
        gimbal_total_mode = GIMBAL_TOTAL_FOLLOW;
    }
    else if (switch_is_up(s0) && gimbal_total_mode != GIMBAL_TOTAL_AUTO_SENTINEL)
    {
        gimbal_total_mode = (s0_up_count == 1U) ?
                             GIMBAL_TOTAL_FOLLOW : GIMBAL_TOTAL_SPIN;
    }
    if (s1_down_count != 0U &&
        (now - last_s1_down_tick) > GIMBAL_FRICTION_REPEAT_WINDOW_MS)
    {
        s1_down_count = 0U;
    }
    if (switch_is_up(s1) || switch_is_down(s0))
    {
        s1_down_count = 0U;
    }
    if (switch_is_down(s1) && switch_is_mid(last_s1) && !switch_is_down(s0))
    {
        s1_down_tick = now;
        if (s1_down_count == 1U)
        {
            gimbal_friction_enabled = !gimbal_friction_enabled;
            s1_down_count = 0U;
        }
        else
        {
            s1_down_count = 1U;
            last_s1_down_tick = now;
        }
    }
    if (switch_is_down(s0))
    {
        gimbal_friction_enabled = false;
    }
    gimbal_strum_enabled = gimbal_friction_enabled && switch_is_down(s1) &&
                           (now - s1_down_tick) >= SHOOT_STRUM_LONG_PRESS_MS;

    last_s0 = s0;
    last_s1 = s1;
}

gimbal_total_mode_e gimbal_total_mode_get(void)
{
    return (gimbal_total_mode_e)gimbal_total_mode;
}

bool gimbal_small_yaw_control_is_allowed(void)
{
    return (uint8_t)(toe_is_error(DBUS_TOE) == 0U);
}

bool gimbal_friction_is_enabled(void)
{
    return gimbal_friction_enabled;
}

bool gimbal_strum_is_enabled(void)
{
    return gimbal_strum_enabled;
}

/**
  * @brief          对遥控器通道值执行死区处理
  * @retval         死区处理后的通道值
  */
static int16_t gimbal_apply_deadband(int16_t value)
{
    if (value > RC_DEADBAND || value < -RC_DEADBAND)
    {
        return value;
    }
    return 0;
}

/**
  * @brief          读取遥控器和鼠标输入并转换为云台角度增量
  * @retval         none
  */
static void gimbal_read_manual_input(float *yaw, float *pitch, gimbal_control_t *control)
{
    int16_t yaw_channel = 0;
    int16_t pitch_channel = 0;
    int mouse_x = 0;
    int mouse_y = 0;

    if (yaw == 0 || pitch == 0 || control == 0)
    {
        return;
    }

    *yaw = 0.0f;
    *pitch = 0.0f;

    if (control->gimbal_rc_ctrl == 0)
    {
        return;
    }

    yaw_channel = gimbal_apply_deadband(control->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL]);
    pitch_channel = gimbal_apply_deadband(control->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL]);
    mouse_x = control->gimbal_rc_ctrl->mouse.x;
    mouse_y = control->gimbal_rc_ctrl->mouse.y;

    *yaw = yaw_channel * YAW_RC_SEN - (float)mouse_x * YAW_MOUSE_SEN;
    *pitch = pitch_channel * PITCH_RC_SEN + (float)mouse_y * PITCH_MOUSE_SEN;
}

/**
  * @brief          设置云台行为模式对应的电机控制模式
  * @retval         none
  */
void gimbal_behaviour_mode_set(gimbal_control_t *control)
{
    if (control == 0)
    {
        return;
    }

    gimbal_behavour_set(control);

    switch (gimbal_behaviour)
    {
    case GIMBAL_ZERO_FORCE:
        control->gimbal_yaw_motor.mode = GIMBAL_MOTOR_RAW;
        control->gimbal_pitch_motor.mode = GIMBAL_MOTOR_RAW;
        break;

    case GIMBAL_INIT:
        control->gimbal_yaw_motor.mode = GIMBAL_MOTOR_GYRO;
        control->gimbal_pitch_motor.mode = GIMBAL_MOTOR_ENCODE;
        break;

    case GIMBAL_CALI:
        control->gimbal_yaw_motor.mode = GIMBAL_MOTOR_RAW;
        control->gimbal_pitch_motor.mode = GIMBAL_MOTOR_RAW;
        break;

    case GIMBAL_ABSOLUTE_ANGLE:
        control->gimbal_yaw_motor.mode = GIMBAL_MOTOR_GYRO;
        control->gimbal_pitch_motor.mode = GIMBAL_MOTOR_ENCODE;
        break;

    case GIMBAL_RELATIVE_ANGLE:
        control->gimbal_yaw_motor.mode = GIMBAL_MOTOR_GYRO;
        control->gimbal_pitch_motor.mode = GIMBAL_MOTOR_ENCODE;
        break;

    case GIMBAL_SPIN:
        control->gimbal_yaw_motor.mode = GIMBAL_MOTOR_GYRO;
        control->gimbal_pitch_motor.mode = GIMBAL_MOTOR_ENCODE;
        break;

    case GIMBAL_AUTO_SENTINEL:
        control->gimbal_yaw_motor.mode = GIMBAL_MOTOR_GYRO;
        control->gimbal_pitch_motor.mode = GIMBAL_MOTOR_ENCODE;
        break;

    case GIMBAL_DEBUG:
        control->gimbal_yaw_motor.mode = GIMBAL_MOTOR_GYRO;
        control->gimbal_pitch_motor.mode = GIMBAL_MOTOR_ENCODE;
        break;

    case GIMBAL_MOTIONLESS:
    default:
        control->gimbal_yaw_motor.mode = GIMBAL_MOTOR_RAW;
        control->gimbal_pitch_motor.mode = GIMBAL_MOTOR_RAW;
        break;
    }
}

/**
  * @brief          根据云台行为模式生成两轴角度增量
  * @retval         none
  */
void gimbal_behaviour_control_set(float *add_yaw, float *add_pitch, gimbal_control_t *control)
{
    if (add_yaw == 0 || add_pitch == 0 || control == 0)
    {
        return;
    }

    switch (gimbal_behaviour)
    {
    case GIMBAL_ZERO_FORCE:
        gimbal_zero_force_control(add_yaw, add_pitch, control);
        break;
    case GIMBAL_INIT:
        gimbal_init_control(add_yaw, add_pitch, control);
        break;
    case GIMBAL_CALI:
        gimbal_cali_control(add_yaw, add_pitch, control);
        break;
    case GIMBAL_ABSOLUTE_ANGLE:
        gimbal_absolute_angle_control(add_yaw, add_pitch, control);
        break;
    case GIMBAL_RELATIVE_ANGLE:
        gimbal_relative_angle_control(add_yaw, add_pitch, control);
        break;
    case GIMBAL_MOTIONLESS:
        gimbal_motionless_control(add_yaw, add_pitch, control);
        break;
    case GIMBAL_SPIN:
        gimbal_spin_control(add_yaw, add_pitch, control);
        break;
    case GIMBAL_AUTO_SENTINEL:
        gimbal_spin_control(add_yaw, add_pitch, control);
        break;
    case GIMBAL_DEBUG:
        gimbal_motionless_control(add_yaw, add_pitch, control);
        break;
    default:
        *add_yaw = 0.0f;
        *add_pitch = 0.0f;
        break;
    }
}

/**
  * @brief          输出云台对底盘停止的联动请求
  * @retval         none
  */
bool gimbal_cmd_to_chassis_stop(void)
{
    return (gimbal_behaviour == GIMBAL_INIT ||
            gimbal_behaviour == GIMBAL_CALI ||
            gimbal_behaviour == GIMBAL_MOTIONLESS ||
            gimbal_behaviour == GIMBAL_ZERO_FORCE);
}

/**
  * @brief          输出云台对底盘停止的联动请求
  * @retval         none
  */
bool gimbal_cmd_to_shoot_stop(void)
{
    return (gimbal_behaviour == GIMBAL_INIT ||
            gimbal_behaviour == GIMBAL_CALI ||
            gimbal_behaviour == GIMBAL_ZERO_FORCE ||
            gimbal_behaviour == GIMBAL_MOTIONLESS);
}

/**
  * @brief          更新云台行为状态机
  * @retval         none
  */
void gimbal_behavour_set(gimbal_control_t *control)
{
    static unsigned int init_time = 0U;
    static unsigned int init_stop_time = 0U;

    if (control == 0)
    {
        return;
    }

    /* 遥控器离线或拨杆数据无效时，禁止初始化回中及动作输出。 */
    if (control->gimbal_rc_ctrl == 0 || toe_is_error(DBUS_TOE) ||
        !(switch_is_up(control->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]) ||
          switch_is_mid(control->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]) ||
          switch_is_down(control->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL])) ||
        !(switch_is_up(control->gimbal_rc_ctrl->rc.s[SHOOT_RC_MODE_CHANNEL]) ||
          switch_is_mid(control->gimbal_rc_ctrl->rc.s[SHOOT_RC_MODE_CHANNEL]) ||
          switch_is_down(control->gimbal_rc_ctrl->rc.s[SHOOT_RC_MODE_CHANNEL])))
    {
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
        gimbal_total_mode = GIMBAL_TOTAL_MOTIONLESS;
        gimbal_friction_enabled = false;
        gimbal_strum_enabled = false;
        init_time = 0U;
        init_stop_time = 0U;
        return;
    }

    gimbal_remote_state_update(control);

    if (gimbal_behaviour == GIMBAL_CALI &&
        control->gimbal_cali.step != 0U &&
        control->gimbal_cali.step != GIMBAL_CALI_END_STEP)
    {
        return;
    }

    if (control->gimbal_cali.step == GIMBAL_CALI_START_STEP)
    {
        gimbal_behaviour = GIMBAL_CALI;
        return;
    }

    if (gimbal_behaviour == GIMBAL_INIT)
    {
        if (fabsf(control->gimbal_yaw_motor.relative_angle - INIT_YAW_SET) < GIMBAL_INIT_ANGLE_ERROR &&
            fabsf(control->gimbal_pitch_motor.relative_angle - INIT_PITCH_SET) < GIMBAL_INIT_ANGLE_ERROR)
        {
            if (init_stop_time < GIMBAL_INIT_STOP_TIME)
            {
                init_stop_time++;
            }
        }
        else
        {
            if (init_time < GIMBAL_INIT_TIME)
            {
                init_time++;
            }
        }

        if (init_time < GIMBAL_INIT_TIME && init_stop_time < GIMBAL_INIT_STOP_TIME)
        {
            return;
        }

        init_time = 0U;
        init_stop_time = 0U;
    }

    switch (gimbal_total_mode)
    {
    case GIMBAL_TOTAL_FOLLOW:
        gimbal_behaviour = GIMBAL_RELATIVE_ANGLE;
        break;
    case GIMBAL_TOTAL_SPIN:
        gimbal_behaviour = GIMBAL_SPIN;
        break;
    case GIMBAL_TOTAL_AUTO_SENTINEL:
        gimbal_behaviour = GIMBAL_AUTO_SENTINEL;
        break;
    case GIMBAL_TOTAL_MOTIONLESS:
    default:
        gimbal_behaviour = GIMBAL_MOTIONLESS;
        break;
    }

}

/**
  * @brief          无力模式云台角度增量清零
  * @retval         none
  */
void gimbal_zero_force_control(float *yaw, float *pitch, gimbal_control_t *control)
{
    (void)control;

    if (yaw == 0 || pitch == 0)
    {
        return;
    }

    *yaw = 0.0f;
    *pitch = 0.0f;
}

/**
  * @brief          初始化模式云台回中控制量生成
  * @retval         none
  */
void gimbal_init_control(float *yaw, float *pitch, gimbal_control_t *control)
{
    if (yaw == 0 || pitch == 0 || control == 0)
    {
        return;
    }

    *yaw = (INIT_YAW_SET - control->gimbal_yaw_motor.relative_angle) * GIMBAL_INIT_YAW_SPEED;
    *pitch = (INIT_PITCH_SET - control->gimbal_pitch_motor.relative_angle) * GIMBAL_INIT_PITCH_SPEED;
}

/**
  * @brief          云台校准模式控制量生成
  * @retval         none
  */
void gimbal_cali_control(float *yaw, float *pitch, gimbal_control_t *control)
{
    if (yaw == 0 || pitch == 0 || control == 0)
    {
        return;
    }

    switch (control->gimbal_cali.step)
    {
    case GIMBAL_CALI_PITCH_MAX_STEP:
        *yaw = 0.0f;
        *pitch = GIMBAL_CALI_MOTOR_SET;
        break;
    case GIMBAL_CALI_PITCH_MIN_STEP:
        *yaw = 0.0f;
        *pitch = -GIMBAL_CALI_MOTOR_SET;
        break;
    case GIMBAL_CALI_YAW_MAX_STEP:
        *yaw = GIMBAL_CALI_MOTOR_SET;
        *pitch = 0.0f;
        break;
    case GIMBAL_CALI_YAW_MIN_STEP:
        *yaw = -GIMBAL_CALI_MOTOR_SET;
        *pitch = 0.0f;
        break;
    default:
        *yaw = 0.0f;
        *pitch = 0.0f;
        break;
    }
}

/**
  * @brief          绝对角模式云台角度增量生成
  * @retval         none
  */
void gimbal_absolute_angle_control(float *yaw, float *pitch, gimbal_control_t *control)
{
    if (yaw == 0 || pitch == 0 || control == 0)
    {
        return;
    }

    gimbal_read_manual_input(yaw, pitch, control);
}

/**
  * @brief          绝对角模式云台角度增量生成
  * @retval         none
  */
void gimbal_relative_angle_control(float *yaw, float *pitch, gimbal_control_t *control)
{
    if (yaw == 0 || pitch == 0 || control == 0)
    {
        return;
    }

    gimbal_read_manual_input(yaw, pitch, control);
}

/**
  * @brief          无力模式云台角度增量清零
  * @retval         none
  */
void gimbal_motionless_control(float *yaw, float *pitch, gimbal_control_t *control)
{
    (void)control;

    if (yaw == 0 || pitch == 0)
    {
        return;
    }

    *yaw = 0.0f;
    *pitch = 0.0f;
}

/**
  * @brief          自旋模式云台角度增量生成
  * @retval         none
  */
void gimbal_spin_control(float *yaw, float *pitch, gimbal_control_t *control)
{
    float manual_yaw = 0.0f;
    float manual_pitch = 0.0f;

    if (yaw == 0 || pitch == 0 || control == 0)
    {
        return;
    }

    gimbal_read_manual_input(&manual_yaw, &manual_pitch, control);
    *yaw = manual_yaw;
    *pitch = manual_pitch;
}
