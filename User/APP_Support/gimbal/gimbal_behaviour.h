/**
  * @file       gimbal_behaviour.h
  * @brief      云台行为模式接口声明
  * @note       定义云台行为枚举和行为层控制接口。
  */
#ifndef GIMBAL_BEHAVIOUR_H
#define GIMBAL_BEHAVIOUR_H

#include <stdbool.h>

#include "gimbal_task.h"

typedef enum
{
    GIMBAL_ZERO_FORCE = 0,
    GIMBAL_INIT,
    GIMBAL_CALI,
    GIMBAL_ABSOLUTE_ANGLE,
    GIMBAL_RELATIVE_ANGLE,
    GIMBAL_MOTIONLESS,
    GIMBAL_SPIN,
    GIMBAL_AUTO_SENTINEL,
    GIMBAL_DEBUG,             // 自动哨兵调试模式
} gimbal_behaviour_e;

typedef enum
{
    GIMBAL_TOTAL_MOTIONLESS = 0,
    GIMBAL_TOTAL_FOLLOW,
    GIMBAL_TOTAL_SPIN,
    GIMBAL_TOTAL_AUTO_SENTINEL,
} gimbal_total_mode_e;

extern volatile gimbal_behaviour_e gimbal_behaviour;

#ifndef GIMBAL_SPIN_KEYBOARD
#define GIMBAL_SPIN_KEYBOARD KEY_PRESSED_OFFSET_SHIFT
#endif

#ifndef GIMBAL_ZERO_KEYBOARD
#define GIMBAL_ZERO_KEYBOARD KEY_PRESSED_OFFSET_X
#endif

#ifndef GIMBAL_RELATIVE_KEYBOARD
#define GIMBAL_RELATIVE_KEYBOARD KEY_PRESSED_OFFSET_C
#endif

#ifndef GIMBAL_SPIN_SPEED
#define GIMBAL_SPIN_SPEED 0.0f
#endif

#ifndef GIMBAL_AUTO_SENTINEL_REPEAT_WINDOW_MS
#define GIMBAL_AUTO_SENTINEL_REPEAT_WINDOW_MS 500U
#endif

#ifndef GIMBAL_FRICTION_REPEAT_WINDOW_MS
#define GIMBAL_FRICTION_REPEAT_WINDOW_MS 800U
#endif

#ifndef GIMBAL_YAW_ORIGIN_TOLERANCE
#define GIMBAL_YAW_ORIGIN_TOLERANCE 0.16f
#endif

/**
  * @brief          设置云台行为模式对应的电机控制模式
  * @retval         none
  */
void gimbal_behaviour_mode_set(gimbal_control_t *control);

/**
  * @brief          根据云台行为模式生成两轴角度增量
  * @retval         none
  */
void gimbal_behaviour_control_set(float *add_yaw, float *add_pitch, gimbal_control_t *control);

/**
  * @brief          输出云台对底盘停止的联动请求
  * @retval         true 表示请求底盘停止
  */
bool gimbal_cmd_to_chassis_stop(void);

/**
  * @brief          输出云台对射击停止的联动请求
  * @retval         true 表示请求射击停止
  */
bool gimbal_cmd_to_shoot_stop(void);

gimbal_total_mode_e gimbal_total_mode_get(void);
bool gimbal_small_yaw_control_is_allowed(void);
bool gimbal_friction_is_enabled(void);
bool gimbal_strum_is_enabled(void);

/**
  * @brief          更新云台行为状态机
  * @retval         none
  */
void gimbal_behavour_set(gimbal_control_t *control);

/**
  * @brief          无力模式云台角度增量清零
  * @retval         none
  */
void gimbal_zero_force_control(float *yaw, float *pitch, gimbal_control_t *control);

/**
  * @brief          初始化模式云台回中控制量生成
  * @retval         none
  */
void gimbal_init_control(float *yaw, float *pitch, gimbal_control_t *control);

/**
  * @brief          云台校准模式控制量生成
  * @retval         none
  */
void gimbal_cali_control(float *yaw, float *pitch, gimbal_control_t *control);

/**
  * @brief          绝对角模式云台角度增量生成
  * @retval         none
  */
void gimbal_absolute_angle_control(float *yaw, float *pitch, gimbal_control_t *control);

/**
  * @brief          相对角模式云台角度增量生成
  * @retval         none
  */
void gimbal_relative_angle_control(float *yaw, float *pitch, gimbal_control_t *control);

/**
  * @brief          静止模式云台角度增量清零
  * @retval         none
  */
void gimbal_motionless_control(float *yaw, float *pitch, gimbal_control_t *control);

/**
  * @brief          自旋模式云台角度增量生成
  * @retval         none
  */
void gimbal_spin_control(float *yaw, float *pitch, gimbal_control_t *control);

#endif
