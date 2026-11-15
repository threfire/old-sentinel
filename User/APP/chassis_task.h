/**
  * @file       chassis_task.h
  * @brief      底盘任务数据结构与接口声明
  * @note       定义底盘模式、电机状态、规划速度、前馈和制动补偿字段。
  */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "bsp_fdcan.h"
#include "gimbal_task.h"
#include "hwt_imu.h"
#include "pid.h"
#include "robot_param.h"
#include "remote_control.h"
#include "struct_typedef.h"
#include "user_lib.h"

#define rc_deadband_limit(input, output, dealine)       \
{                                                       \
    if ((input) > (dealine) || (input) < -(dealine))    \
    {                                                   \
        (output) = (input);                             \
    }                                                   \
    else                                                \
    {                                                   \
        (output) = 0;                                   \
    }                                                   \
}

typedef enum
{
    CHASSIS_VECTOR_NO_MOVE = 0,
    CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,
    CHASSIS_VECTOR_YAW_HOLD,
    CHASSIS_VECTOR_SPIN,
} chassis_mode_e;

typedef struct
{
    const motor_measure_t *chassis_motor_measure;
    fp32 accel;
    fp32 speed;
    fp32 speed_filtered;
    fp32 speed_rad_s;
    fp32 speed_set;
    fp32 angle;
    int16_t give_current;
    fp32 given_current_a;
} chassis_motor_t;

typedef struct
{
    fp32 now[CHASSIS_MODULE_NUM];
    fp32 last[CHASSIS_MODULE_NUM];
    fp32 initial[CHASSIS_MODULE_NUM];
} wheel_angle_offset_t;

typedef struct
{
    fp32 x_mm;
    fp32 y_mm;
    fp32 yaw_deg;
    fp32 gyro_dps;
    fp32 vx_mps;
    fp32 vy_mps;
    uint16_t can_id;
    uint8_t data_len;
    uint8_t valid;
    uint32_t last_update_tick;
    uint8_t raw[28];
} chassis_odometer_t;

typedef struct
{
    const RC_ctrl_t *chassis_RC;
    const gimbal_motor_t *chassis_yaw_motor;

    chassis_mode_e chassis_mode;
    chassis_mode_e last_chassis_mode;
    uint32_t spin_start_tick; /**< 小陀螺正弦变速起始时间（ms）。 */
    uint8_t spin_wave_active; /**< 小陀螺正弦变速启动标志，1 为已启动。 */

    uint8_t spin_exit_transition_active; /**< 小陀螺退出过渡状态，1 表示正在平滑减速。 */

    chassis_motor_t chassis_3508[CHASSIS_MODULE_NUM];
    wheel_angle_offset_t wheel_angle_offset;
    chassis_odometer_t odometer;

    fp32 model_3508_out[CHASSIS_MODULE_NUM];
    fp32 model_accel[CHASSIS_MODULE_NUM];
    fp32 model_last_speed_set[CHASSIS_MODULE_NUM];
    fp32 speed_pi_iout[CHASSIS_MODULE_NUM];
    fp32 speed_pid_last_error[CHASSIS_MODULE_NUM];
    uint8_t stop_brake_active[CHASSIS_MODULE_NUM];
    fp32 motor_current_limit_a[CHASSIS_MODULE_NUM];
    fp32 last_motor_current_limit_a[CHASSIS_MODULE_NUM];

    pid_type_def chassis_angle_pid;
    first_order_filter_type_t chassis_cmd_slow_set_vx;
    first_order_filter_type_t chassis_cmd_slow_set_vy;

    fp32 vx;
    fp32 vy;
    fp32 wz;
    fp32 vx_set;
    fp32 vy_set;
    fp32 wz_set;
    fp32 last_vx_set;
    fp32 last_vy_set;
    fp32 last_wz_set;
    fp32 vx_plan;
    fp32 vy_plan;
    fp32 wz_plan;
    fp32 vx_plan_accel;
    fp32 vy_plan_accel;
    fp32 wz_plan_accel;
    fp32 chassis_relative_angle;
    fp32 chassis_relative_angle_vel;
    fp32 chassis_relative_angle_target;
    fp32 chassis_relative_angle_set;
    fp32 chassis_relative_angle_set_vel;
    fp32 chassis_yaw_target;
    fp32 chassis_yaw_set;
    fp32 chassis_yaw_set_vel;
    fp32 chassis_yaw_set_vel_filtered;
    fp32 last_vx_plan_ff;
    fp32 last_vy_plan_ff;
    fp32 last_wz_plan_ff;
    fp32 body_ff_ax;
    fp32 body_ff_ay;
    fp32 body_ff_alpha;
    fp32 body_ff_current[CHASSIS_MODULE_NUM];
    fp32 body_vel_brake_error_vx;
    fp32 body_vel_brake_error_vy;
    fp32 body_vel_brake_error_wz;
    fp32 body_vel_brake_current[CHASSIS_MODULE_NUM];
    uint8_t body_ff_init;
    fp32 chassis_yaw_rate;
    fp32 wz_speed_pi_iout;

    fp32 vx_max_speed;
    fp32 vx_min_speed;
    fp32 vy_max_speed;
    fp32 vy_min_speed;
    fp32 chassis_yaw;
    fp32 chassis_pitch;
    fp32 chassis_roll;
    fp32 chassis_yaw_rate_filtered;
    first_order_filter_type_t chassis_yaw_rate_filter;
    first_order_filter_type_t chassis_yaw_set_vel_filter;
    first_order_filter_type_t wheel_speed_filter[CHASSIS_MODULE_NUM];
} chassis_move_t;

/**
  * @brief          底盘任务入口
  * @retval         none
  */
extern void chassis_task(void *pvParameters);

/**
  * @brief          初始化底盘控制结构体
  * @retval         none
  */
void chassis_init(chassis_move_t *chassis_move_init);

/**
  * @brief          设置底盘当前模式
  * @retval         none
  */
void chassis_set_mode(chassis_move_t *chassis_move_mode);

/**
  * @brief          处理底盘模式切换过渡
  * @retval         none
  */
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);

/**
  * @brief          更新底盘电机和传感器反馈
  * @retval         none
  */
void chassis_feedback_update(chassis_move_t *chassis_move_update);

/**
  * @brief          生成底盘控制目标量
  * @retval         none
  */
void chassis_set_contorl(chassis_move_t *chassis_move_control);

/**
  * @brief          执行底盘闭环控制
  * @retval         none
  */
void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

/**
  * @brief          发送底盘电机命令
  * @retval         none
  */
void chassis_send_cmd(chassis_move_t *chassis_move_send);

void chassis_odometer_can_rx(uint16_t can_id, const uint8_t *data, uint8_t len);

extern chassis_move_t chassis_move;
extern fp32 yaw_set;

#endif
