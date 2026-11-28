/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      minimal gimbal control framework
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include <stdint.h>
#include <stdbool.h>

#include "robot_param.h"
#include "remote_control.h"
#include "pid.h"
#include "gravity_comp.h"

typedef enum
{
    GIMBAL_MOTOR_RAW = 0,
    GIMBAL_MOTOR_GYRO,
    GIMBAL_MOTOR_ENCODE,
} gimbal_motor_mode_e;

#ifndef GIMBAL_MOTOR_ENCONDE
#define GIMBAL_MOTOR_ENCONDE GIMBAL_MOTOR_ENCODE
#endif

typedef pid_type_def gimbal_pid_t;

typedef struct
{
    gimbal_motor_mode_e mode;        // 当前电机控制模式
    gimbal_motor_mode_e last_mode;   // 上一次电机控制模式，用于模式切换过渡

    float relative_angle;            // 相对角度反馈值
    float relative_angle_unwrapped;  // 大 yaw 连续累计相对角度，单位 rad
    float relative_angle_feedback_last; // 大 yaw 上一帧包角反馈，单位 rad
    uint8_t relative_angle_unwrap_init; // 大 yaw 连续角累计初始化标志
    float relative_angle_set;        // 相对角度目标值
    float relative_angle_last;       // 上一次相对角度反馈值
    float relative_speed;            // 相对角速度反馈，pitch控制中为低通后的反馈值
    uint8_t relative_speed_update_init; // 相对角速度差分初始化标志

    float absolute_angle;            // 绝对角度反馈值
    float absolute_angle_set;        // 绝对角度目标值

    float angle_offset;              // 上电/初始化时记录的角度零偏
    uint8_t angle_offset_init;       // 角度零偏是否已初始化

    float gyro;                      // 角速度反馈值
    float gyro_set;                  // 角速度目标值
    float gyro_last;                 // 上一次角速度反馈值
    float gyro_accel;                // 角加速度反馈值，由角速度差分得到
    uint8_t gyro_update_init;        // 角速度差分初始化标志

    float ref_vel;                   // 前馈轨迹参考速度
    float ref_vel_last;              // 上一次前馈轨迹参考速度
    float ref_accel;                 // 前馈轨迹参考加速度
    float rc_ref_vel;
    float rc_ref_vel_last;
    float rc_ref_accel;
    float rc_ref_target_last;
    uint8_t rc_ref_target_init;
    float auto_ref_vel;
    float auto_ref_vel_last;
    float auto_ref_accel;
    float auto_ref_target_last;
    uint8_t auto_ref_target_init;
    float pid_torque;                // PID 反馈输出力矩
    float ff_torque;                 // 惯量前馈输出力矩
    float static_friction_comp;      // 静摩擦补偿输出力矩
    float inertia_kgm2;              // 转动惯量参数 J，单位 kg*m^2

    float raw_cmd;                   // RAW 模式下直接输出命令
    float output;                    // 控制器最终输出值
    float current_set;               // 目标电流/目标力矩命令
    float given_current;             // 实际发送给电机的电流/力矩命令

    float max_relative_angle;        // 相对角度上限
    float min_relative_angle;        // 相对角度下限

    float rc_relative_angle_set;
    float rc_absolute_angle_set;
    float auto_relative_angle_set;
    float auto_absolute_angle_set;
    uint8_t rc_control_enable;
    uint8_t auto_control_enable;
    gimbal_pid_t absolute_angle_pid;
    gimbal_pid_t relative_angle_pid;
} gimbal_motor_t;

typedef struct
{
    gimbal_pid_t angle_pid;           // 小 yaw 角度环 PID
    gimbal_pid_t speed_pid;           // 小 yaw 速度环 PID
    float current_cmd_a;               // 小 yaw 电流命令，单位 A
    int16_t current_cmd_raw;           // 小 yaw 电流命令原始值
    uint8_t control_allowed;           // 小 yaw 控制许可
    uint8_t can_send_status;           // 小 yaw CAN 发送状态
    float angle_rad;                   // 小 yaw 编码器角度，单位 rad
    float target_rad;                  // 小 yaw 目标角度，单位 rad
    float speed_feedback_rad_s;        // 小 yaw 角速度反馈，单位 rad/s
    uint16_t last_ecd;                 // 小 yaw 上一帧编码器值
    uint8_t feedback_init;              // 小 yaw 编码器反馈初始化标志
    uint8_t scan_yaw_direction;         // 小 yaw 扫描方向
    uint8_t scan_pitch_direction;       // pitch 扫描方向
    float scan_pitch_target_rad;        // pitch 扫描目标角度，单位 rad
    float ref_vel_rad_s;               // 最终扫描轨迹参考速度，非速度环指令
    float ref_accel_rad_s2;            // 最终扫描轨迹参考角加速度
    float accel_ff_gain;               // 可在线整定的等效 J/Kt，A/(rad/s^2)
    float ff_max_current_a;            // 前馈电流独立限幅，单位 A
    float ff_current_a;                // 限幅及保护后的惯量前馈电流，单位 A
    float pid_current_a;               // 速度 PI 电流贡献（总限幅之前），单位 A
    float velocity_ff_gain;            // 速度前馈比例，0 关闭，1 为完整补偿
    float velocity_ff_rad_s;           // 速度前馈贡献，合成限幅前，单位 rad/s
} gimbal_small_yaw_t;

typedef struct
{
    float max_yaw;
    float min_yaw;
    float max_pitch;
    float min_pitch;
    uint16_t max_yaw_ecd;
    uint16_t min_yaw_ecd;
    uint16_t max_pitch_ecd;
    uint16_t min_pitch_ecd;
    uint8_t step;
} gimbal_step_cali_t;

typedef struct
{
    uint8_t total_mode;
    bool friction_enabled;
    bool strum_enabled;
} gimbal_behaviour_state_t;

typedef struct
{
    uint8_t active;                    // 正弦扫描是否已进入
    uint32_t start_tick;               // 本次扫描起始时刻，单位 ms
    float small_yaw_start_rad;         // 进入扫描时的小 yaw 实测角度
    float pitch_start_rad;             // 进入扫描时的 pitch 实测角度
    float blend;                       // smoothstep 混合系数，范围 0~1
    uint8_t exit_active;               // 正在正常退出扫描（无力保护不等待）
    uint32_t exit_start_tick;          // 退出过渡开始时刻，单位 ms
    float exit_blend;                  // 退出进度：0 扫描，1 停止
    float exit_pitch_target_rad;       // pitch 停在退出瞬间的目标位置
} gimbal_sine_scan_t;

typedef struct gimbal_control_t
{
    const RC_ctrl_t *gimbal_rc_ctrl;
    const float *gimbal_INT_angle_point;
    const float *gimbal_INT_gyro_point;
    gimbal_motor_t gimbal_yaw_motor;
    gimbal_motor_t gimbal_pitch_motor;
    gimbal_step_cali_t gimbal_cali;
    gravity_comp_t gimbal_pitch_gravity_comp;
    gimbal_small_yaw_t small_yaw;
    gimbal_behaviour_state_t behaviour_state;
    gimbal_sine_scan_t sine_scan;
} gimbal_control_t;

extern gimbal_control_t gimbal_control;
extern float yaw_can_set_current;
extern float pitch_can_set_current;
extern int16_t shoot_can_set_current;

/**
  * @brief          创建云台控制任务
  * @retval         none
  */
void GimbalTask_Init(void);

/**
  * @brief          获取 yaw 电机控制结构体指针
  * @retval         yaw 电机控制结构体只读指针
  */
const gimbal_motor_t *get_yaw_motor_point(void);

/**
  * @brief          获取 pitch 电机控制结构体指针
  * @retval         pitch 电机控制结构体只读指针
  */
const gimbal_motor_t *get_pitch_motor_point(void);

/**
  * @brief          更新绝对角目标并执行限幅
  * @retval         none
  */
void gimbal_absolute_angle_limit(gimbal_motor_t *motor, float add);

/**
  * @brief          执行云台电机绝对角控制
  * @retval         none
  */
void gimbal_motor_absolute_angle_control(gimbal_motor_t *motor);

/**
  * @brief          执行云台电机相对角控制
  * @retval         none
  */
void gimbal_motor_relative_angle_control(gimbal_motor_t *motor);

/**
  * @brief          执行云台电机 RAW 输出控制
  * @retval         none
  */
void gimbal_motor_raw_angle_control(gimbal_motor_t *motor);

/**
  * @brief          初始化云台 PID 控制器
  * @retval         none
  */
void gimbal_pid_init(gimbal_pid_t *pid, float kp, float ki, float kd, float max_out, float max_iout);

/**
  * @brief          清空云台 PID 控制器状态
  * @retval         none
  */
void gimbal_pid_clear(gimbal_pid_t *pid);

/**
  * @brief          计算云台 PID 输出
  * @retval         PID 输出
  */
float gimbal_pid_calc(gimbal_pid_t *pid, float get, float set, float error_delta);

/**
  * @brief          初始化云台控制结构体
  * @retval         none
  */
void gimbal_init(gimbal_control_t *control);
/**
  * @brief          设置云台控制模式
  * @retval         none
  */
void gimbal_set_mode(gimbal_control_t *control);
/**
  * @brief          更新云台反馈数据
  * @retval         none
  */
void gimbal_feedback_update(gimbal_control_t *control);
/**
  * @brief          初始化云台控制结构体
  * @retval         none
  */
void gimbal_mode_change_control_transit(gimbal_control_t *control);
/**
  * @brief          生成云台目标控制量
  * @retval         none
  */
void gimbal_set_control(gimbal_control_t *control);
/**
  * @brief          设置云台控制模式
  * @retval         none
  */
void gimbal_control_loop(gimbal_control_t *control);
/**
  * @brief          设置云台控制模式
  * @retval         none
  */
void gimbal_send_cmd(gimbal_control_t *control);
#endif
