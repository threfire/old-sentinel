/**
  * @file       yaw_pitch_direct.c
  * @brief      yaw/pitch 直控云台控制实现
  * @note       实现云台反馈更新、目标限幅、前馈补偿、PID 闭环、在线保护和 MIT 力矩发送。
  */
#include "yaw_pitch_direct.h"
#include "gimbal_behaviour.h"
#include "auto_aim.h"
#include "hwt_imu.h"
#include "bsp_fdcan.h"
#include "detect_task.h"
#include "robot_param.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include <math.h>
#include <string.h>

#if (ROBOT_GIMBAL == ROBOT_GIMBAL_YAW_PITCH_DIRECT)

float yaw_can_set_current = 0.0f;
float pitch_can_set_current = 0.0f;
int16_t shoot_can_set_current = 0;

static float yaw_auto_aim_target_vel = 0.0f;
static float pitch_auto_aim_target_vel = 0.0f;
static gimbal_pid_t pitch_6020_speed_pid;
static uint16_t pitch_6020_last_ecd = 0U;
static uint16_t pitch_6020_reference_ecd = 0U;
static uint8_t pitch_6020_feedback_init = 0U;
static float pitch_6020_angle_rad = 0.0f;
static float pitch_6020_speed_feedback_rad_s = 0.0f;
#define small_yaw_angle_pid          (gimbal_control.small_yaw.angle_pid)
#define small_yaw_speed_pid          (gimbal_control.small_yaw.speed_pid)
#define small_yaw_current_cmd_a      (gimbal_control.small_yaw.current_cmd_a)
#define small_yaw_current_cmd_raw    (gimbal_control.small_yaw.current_cmd_raw)
#define small_yaw_control_allowed    (gimbal_control.small_yaw.control_allowed)
#define small_yaw_can_send_status    (gimbal_control.small_yaw.can_send_status)
#define small_yaw_angle_rad          (gimbal_control.small_yaw.angle_rad)
#define small_yaw_target_rad         (gimbal_control.small_yaw.target_rad)
#define small_yaw_speed_feedback_rad_s (gimbal_control.small_yaw.speed_feedback_rad_s)
#define small_yaw_last_ecd           (gimbal_control.small_yaw.last_ecd)
#define small_yaw_feedback_init      (gimbal_control.small_yaw.feedback_init)
#define small_yaw_scan_yaw_direction  (gimbal_control.small_yaw.scan_yaw_direction)
#define small_yaw_scan_pitch_direction (gimbal_control.small_yaw.scan_pitch_direction)
#define scan_pitch_target_rad         (gimbal_control.small_yaw.scan_pitch_target_rad)

static void small_yaw_feedback_update(void);
static float small_yaw_ecd_to_angle_rad(uint16_t ecd);
static void small_yaw_scan_update(gimbal_control_t *control, uint8_t auto_enable);
static void gimbal_debug_sine_target_update(gimbal_control_t *control);
static void small_yaw_control_update(void);
static int16_t small_yaw_current_to_cmd(float current_a);
static float pitch_6020_feedback_update(void);
static void pitch_6020_control_update(void);
static int16_t pitch_6020_current_to_cmd(float current_a);
static float pitch_6020_ecd_to_angle(uint16_t ecd);
static float small_yaw_clamp_target_rad(float target_rad);

/**
  * @brief          声明 yaw 电机在线检测函数
  * @retval         none
  */
static uint8_t gimbal_yaw_online(void);
/**
  * @brief          声明 pitch GM6020 反馈检测函数
  * @retval         none
  */
static uint8_t gimbal_pitch_6020_feedback_ready(void);
/**
  * @brief          声明单轴零输出清理函数
  * @retval         none
  */
static void gimbal_motor_zero_output(gimbal_motor_t *motor);
/**
  * @brief          声明角度归一化函数
  * @retval         归一化后的角度
  */
static float gimbal_wrap_angle(float angle);
/**
  * @brief          更新大 yaw 连续累计相对角度
  * @retval         连续累计相对角度，单位 rad
  */
static float gimbal_yaw_unwrap_update(gimbal_motor_t *motor, float wrapped_angle);
/**
  * @brief          声明自瞄补偿读取函数
  * @retval         自瞄补偿量
  */
static float gimbal_take_auto_aim_bias(gimbal_motor_t *motor);
/**
  * @brief          声明浮点限幅函数
  * @retval         限幅后的值
  */
static float gimbal_clamp(float value, float min_value, float max_value);
/**
  * @brief          声明静摩擦补偿原始量计算函数
  * @retval         静摩擦补偿原始量
  */
static float gimbal_calc_static_friction_comp_raw(const gimbal_motor_t *motor, float angle_error);
/**
  * @brief          声明静摩擦补偿更新函数
  * @retval         静摩擦补偿输出
  */
static float gimbal_update_static_friction_comp(gimbal_motor_t *motor, float angle_error);
/**
  * @brief          声明力矩命令限幅函数
  * @retval         力矩命令
  */
static float gimbal_float_to_torque_cmd(float output);
/**
  * @brief          声明惯量前馈计算函数
  * @retval         前馈力矩
  */
static float gimbal_calc_feedforward(gimbal_motor_t *motor);
/**
  * @brief          声明角度反馈力矩计算函数
  * @retval         反馈力矩
  */
static float gimbal_calc_feedback_torque(gimbal_motor_t *motor, gimbal_pid_t *angle_pid, float angle_get, float angle_set);
/**
  * @brief          声明角速度力矩计算函数
  * @retval         控制力矩
  */
static float gimbal_calc_angle_speed_torque(gimbal_motor_t *motor, gimbal_pid_t *pid,
                                           float angle_error, float speed_get);
/**
  * @brief          声明自瞄目标速度清理函数
  * @retval         none
  */
static void gimbal_auto_aim_clear_target_vel(gimbal_motor_t *motor);

/**
  * @brief          获取 INS 姿态角数组弱接口
  * @retval         姿态角数组指针
  */
__attribute__((weak)) const float *get_INS_angle_point(void)
{
    return 0;
}

/**
  * @brief          获取陀螺仪角速度数组弱接口
  * @retval         角速度数组指针
  */
__attribute__((weak)) const float *get_gyro_data_point(void)
{
    return 0;
}

/**
  * @brief          获取加速度数组弱接口
  * @retval         加速度数组指针
  */
__attribute__((weak)) const float *get_accel_data_point(void)
{
    return 0;
}

/**
  * @brief          对 MIT 力矩命令执行限幅
  * @retval         限幅后的值
  */
static float gimbal_mit_clamp(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }
    if (value < min_value)
    {
        return min_value;
    }
    return value;
}

/**
  * @brief          将控制器输出转换为 MIT 力矩命令
  * @retval         none
  */
static float gimbal_output_to_mit_torque(float output)
{
    return gimbal_mit_clamp(output, T_MIN, T_MAX);
}

/**
  * @brief          判断 yaw 电机是否在线
  * @retval         none
  */
static uint8_t gimbal_yaw_online(void)
{
    return (uint8_t)(toe_is_error(YAW_GIMBAL_MOTOR_TOE) == 0U);
}

static uint8_t gimbal_hwt906_yaw_ready(void)
{
    const hwt_imu_info_t *hwt906 = hwt906_get_info();

    return (uint8_t)((hwt906 != 0) && (hwt906->angle.yaw_init_flag != 0u));
}

/**
  * @brief          判断 pitch GM6020 反馈是否可用
  * @retval         none
  */
static uint8_t gimbal_pitch_6020_feedback_ready(void)
{
    const motor_measure_t *measure = &DJI_MOTOR_MEASURE[GIMBAL_PITCH_DJI_INDEX];
    uint32_t now = HAL_GetTick();

    return (uint8_t)((toe_is_error(PITCH_GIMBAL_MOTOR_TOE) == 0U) &&
                     (measure->last_fdb_time != 0U) &&
                     ((uint32_t)(now - measure->last_fdb_time) <=
                      (uint32_t)GIMBAL_PITCH_6020_FDB_TIMEOUT));
}

static float pitch_6020_feedback_update(void)
{
    const motor_measure_t *measure = &DJI_MOTOR_MEASURE[GIMBAL_PITCH_DJI_INDEX];

    if (measure->last_fdb_time == 0U)
    {
        return pitch_6020_angle_rad;
    }

    if (pitch_6020_feedback_init == 0U)
    {
        pitch_6020_last_ecd = measure->ecd;
        pitch_6020_reference_ecd = measure->ecd;
        /* 上电当前位置作为临时角度基准，实体限位需按装车位置标定。 */
        pitch_6020_angle_rad = INIT_PITCH_SET;
        pitch_6020_feedback_init = 1U;
    }
    else
    {
        int16_t delta_ecd = (int16_t)(measure->ecd - pitch_6020_last_ecd);
        if (delta_ecd > 4096) delta_ecd -= 8192;
        if (delta_ecd < -4096) delta_ecd += 8192;
        pitch_6020_angle_rad +=
            PITCH_6020_FEEDBACK_SIGN * (float)delta_ecd *
            (2.0f * YAW_PITCH_DIRECT_PI / 8192.0f);
        pitch_6020_last_ecd = measure->ecd;
    }

    return pitch_6020_angle_rad;
}

static float pitch_6020_ecd_to_angle(uint16_t ecd)
{
    int16_t delta_ecd = (int16_t)(ecd - pitch_6020_reference_ecd);

    if (delta_ecd > 4096)
    {
        delta_ecd -= 8192;
    }
    if (delta_ecd < -4096)
    {
        delta_ecd += 8192;
    }

    return INIT_PITCH_SET +
           PITCH_6020_FEEDBACK_SIGN * (float)delta_ecd *
           (2.0f * YAW_PITCH_DIRECT_PI / 8192.0f);
}

/**
  * @brief          清除单轴前馈轨迹状态
  * @retval         none
  */
static void gimbal_feedforward_clear(gimbal_motor_t *motor)
{
    if (motor == 0)
    {
        return;
    }

    motor->ref_vel = 0.0f;
    motor->ref_vel_last = 0.0f;
    motor->ref_accel = 0.0f;
    motor->rc_ref_vel = 0.0f;
    motor->rc_ref_vel_last = 0.0f;
    motor->rc_ref_accel = 0.0f;
    motor->rc_ref_target_last = 0.0f;
    motor->rc_ref_target_init = 0u;
    motor->auto_ref_vel = 0.0f;
    motor->auto_ref_vel_last = 0.0f;
    motor->auto_ref_accel = 0.0f;
    motor->auto_ref_target_last = 0.0f;
    motor->auto_ref_target_init = 0u;
    motor->ff_torque = 0.0f;
    gimbal_auto_aim_clear_target_vel(motor);
}

/**
  * @brief          清除单轴控制状态并置零输出
  * @retval         none
  */
static void gimbal_motor_zero_output(gimbal_motor_t *motor)
{
    if (motor == 0)
    {
        return;
    }

    gimbal_feedforward_clear(motor);
    gimbal_pid_clear(&motor->absolute_angle_pid);
    gimbal_pid_clear(&motor->relative_angle_pid);
    if (motor == &gimbal_control.gimbal_pitch_motor)
    {
        gimbal_pid_clear(&pitch_6020_speed_pid);
    }

    motor->raw_cmd = 0.0f;
    motor->current_set = 0.0f;
    motor->output = 0.0f;
    motor->given_current = 0.0f;
    motor->pid_torque = 0.0f;
    motor->static_friction_comp = 0.0f;
    motor->rc_control_enable = 0u;
    motor->auto_control_enable = 0u;
    motor->absolute_angle_set = motor->absolute_angle;
    motor->relative_angle_set = motor->relative_angle;
    motor->rc_absolute_angle_set = motor->absolute_angle;
    motor->rc_relative_angle_set = motor->relative_angle;
    motor->auto_absolute_angle_set = motor->absolute_angle;
    motor->auto_relative_angle_set = motor->relative_angle;
    motor->gyro_set = motor->gyro;
}

/**
  * @brief          清除指定控制来源的前馈状态
  * @retval         none
  */
static void gimbal_feedforward_clear_source(gimbal_motor_t *motor,
                                            gimbal_control_source_e source)
{
    if (motor == 0)
    {
        return;
    }

    if (source == GIMBAL_CONTROL_SOURCE_AUTO)
    {
        motor->auto_ref_vel = 0.0f;
        motor->auto_ref_vel_last = 0.0f;
        motor->auto_ref_accel = 0.0f;
        motor->auto_ref_target_last = 0.0f;
        motor->auto_ref_target_init = 0u;
        gimbal_auto_aim_clear_target_vel(motor);
        return;
    }

    motor->rc_ref_vel = 0.0f;
    motor->rc_ref_vel_last = 0.0f;
    motor->rc_ref_accel = 0.0f;
    motor->rc_ref_target_last = 0.0f;
    motor->rc_ref_target_init = 0u;
}

/**
  * @brief          根据目标角跟踪更新前馈速度和加速度
  * @retval         none
  */
static void gimbal_feedforward_track_target(gimbal_motor_t *motor,
                                            float target_angle,
                                            gimbal_control_source_e source)
{
    float ref_vel_cmd;
    float ref_accel_cmd;
    float alpha;
    float accel_limit;
    float *ref_vel;
    float *ref_vel_last;
    float *ref_accel;
    float *target_last;
    uint8_t *target_init;
    const float control_dt = (float)GIMBAL_CONTROL_TIME * 0.001f;

    if (motor == 0 || control_dt <= 0.0f)
    {
        return;
    }

    if (source == GIMBAL_CONTROL_SOURCE_AUTO)
    {
        ref_vel = &motor->auto_ref_vel;
        ref_vel_last = &motor->auto_ref_vel_last;
        ref_accel = &motor->auto_ref_accel;
        target_last = &motor->auto_ref_target_last;
        target_init = &motor->auto_ref_target_init;
    }
    else
    {
        ref_vel = &motor->rc_ref_vel;
        ref_vel_last = &motor->rc_ref_vel_last;
        ref_accel = &motor->rc_ref_accel;
        target_last = &motor->rc_ref_target_last;
        target_init = &motor->rc_ref_target_init;
    }

    if (*target_init == 0u)
    {
        *target_last = target_angle;
        *target_init = 1u;
        *ref_vel_last = 0.0f;
        *ref_vel = 0.0f;
        *ref_accel = 0.0f;
        return;
    }

    alpha = gimbal_mit_clamp(YAW_REF_VEL_FILTER_ALPHA, 0.0f, 1.0f);
    ref_vel_cmd = (target_angle - *target_last) / control_dt;

    *ref_vel_last = *ref_vel;
    *ref_vel += alpha * (ref_vel_cmd - *ref_vel);

    ref_accel_cmd = (*ref_vel - *ref_vel_last) / control_dt;
    accel_limit = YAW_REF_ACCEL_LIMIT;
    if (accel_limit > 0.0f)
    {
        ref_accel_cmd = gimbal_mit_clamp(ref_accel_cmd,
                                         -accel_limit,
                                         accel_limit);
    }

    *ref_accel = ref_accel_cmd;
    *target_last = target_angle;
}

/**
  * @brief          将角度归一化到 [-pi, pi]
  * @retval         none
  */
static float yaw_pitch_direct_wrap_angle(float angle)
{
    while (angle > YAW_PITCH_DIRECT_PI)
    {
        angle -= 2.0f * YAW_PITCH_DIRECT_PI;
    }
    while (angle < -YAW_PITCH_DIRECT_PI)
    {
        angle += 2.0f * YAW_PITCH_DIRECT_PI;
    }
    return angle;
}

static float gimbal_yaw_unwrap_update(gimbal_motor_t *motor, float wrapped_angle)
{
    float delta;

    if (motor == 0)
    {
        return wrapped_angle;
    }

    if (motor->relative_angle_unwrap_init == 0u)
    {
        motor->relative_angle_feedback_last = wrapped_angle;
        motor->relative_angle_unwrapped = wrapped_angle;
        motor->relative_angle_unwrap_init = 1u;
        return motor->relative_angle_unwrapped;
    }

    delta = wrapped_angle - motor->relative_angle_feedback_last;
    if (delta > YAW_PITCH_DIRECT_PI)
    {
        delta -= 2.0f * YAW_PITCH_DIRECT_PI;
    }
    else if (delta < -YAW_PITCH_DIRECT_PI)
    {
        delta += 2.0f * YAW_PITCH_DIRECT_PI;
    }

    motor->relative_angle_unwrapped += delta;
    motor->relative_angle_feedback_last = wrapped_angle;
    return motor->relative_angle_unwrapped;
}

/**
  * @brief          规划自瞄目标角速度受限的目标角
  * @retval         none
  */
static float gimbal_auto_aim_plan_target(float target_set,
                                         float desired_target,
                                         float *target_vel,
                                         float kp,
                                         float max_speed,
                                         float max_accel,
                                         float dt)
{
    float err;
    float vel_cmd;
    float vel_delta;
    float vel_delta_max;
    float step;

    if (target_vel == 0 || dt <= 0.0f)
    {
        return target_set;
    }

    err = desired_target - target_set;
    vel_cmd = gimbal_mit_clamp(kp * err, -max_speed, max_speed);
    vel_delta_max = max_accel * dt;
    vel_delta = gimbal_mit_clamp(vel_cmd - *target_vel,
                                 -vel_delta_max,
                                  vel_delta_max);
    *target_vel += vel_delta;

    step = *target_vel * dt;
    if ((err > 0.0f && step > err) || (err < 0.0f && step < err))
    {
        step = err;
        *target_vel = 0.0f;
    }

    return target_set + step;
}

/**
  * @brief          清除单轴自瞄目标速度状态
  * @retval         none
  */
static void gimbal_auto_aim_clear_target_vel(gimbal_motor_t *motor)
{
    if (motor == &gimbal_control.gimbal_yaw_motor)
    {
        yaw_auto_aim_target_vel = 0.0f;
    }
    else if (motor == &gimbal_control.gimbal_pitch_motor)
    {
        pitch_auto_aim_target_vel = 0.0f;
    }
}

/**
  * @brief          更新 yaw 绝对角目标并执行限幅
  * @retval         none
  */
static void gimbal_yaw_absolute_angle_limit(gimbal_control_t *control,
                                            float add,
                                            uint8_t rc_enable,
                                            uint8_t auto_enable)
{
    gimbal_motor_t *yaw_motor;
    float chassis_yaw;
    float rc_relative_angle_set;
    float auto_relative_angle_set;
    float desired_relative_angle;
    const float control_dt = (float)GIMBAL_CONTROL_TIME * 0.001f;
    if (control == 0)
    {
        return;
    }

    yaw_motor = &control->gimbal_yaw_motor;
    chassis_yaw = yaw_motor->absolute_angle - yaw_motor->relative_angle_unwrapped;
    rc_relative_angle_set =
        yaw_motor->rc_absolute_angle_set -
        chassis_yaw;

    if (rc_enable != 0u)
    {
        rc_relative_angle_set += add;
    }
    else
    {
        rc_relative_angle_set = yaw_motor->relative_angle;
        gimbal_feedforward_clear_source(yaw_motor, GIMBAL_CONTROL_SOURCE_RC);
    }

    yaw_motor->rc_relative_angle_set = rc_relative_angle_set;
    yaw_motor->rc_absolute_angle_set =
        chassis_yaw + rc_relative_angle_set;

    if (auto_enable != 0u)
    {
        desired_relative_angle =
            yaw_motor->relative_angle_unwrapped +
            auto_aim_get_yaw_err_rad();
        auto_relative_angle_set =
            gimbal_auto_aim_plan_target(yaw_motor->auto_relative_angle_set,
                                        desired_relative_angle,
                                        &yaw_auto_aim_target_vel,
                                        GIMBAL_AUTO_AIM_YAW_KP,
                                        GIMBAL_AUTO_AIM_YAW_MAX_SPEED,
                                        GIMBAL_AUTO_AIM_YAW_MAX_ACCEL,
                                        control_dt);
    }
    else
    {
        auto_relative_angle_set = rc_relative_angle_set;
    }

    yaw_motor->auto_relative_angle_set = auto_relative_angle_set;
    yaw_motor->auto_absolute_angle_set =
        chassis_yaw + auto_relative_angle_set;

    yaw_motor->relative_angle_set =
        rc_relative_angle_set +
        ((auto_enable != 0u) ? (auto_relative_angle_set - yaw_motor->relative_angle) : 0.0f);
    yaw_motor->absolute_angle_set =
        chassis_yaw + yaw_motor->relative_angle_set;
}

/**
  * @brief          更新 pitch 相对角目标并执行限幅
  * @retval         none
  */
static void gimbal_pitch_relative_angle_limit(gimbal_control_t *control,
                                              float add,
                                              uint8_t rc_enable,
                                              uint8_t auto_enable)
{
    gimbal_motor_t *pitch_motor;
    float rc_relative_angle_set;
    float desired_relative_angle;
    float auto_relative_angle_set;
    const float control_dt = (float)GIMBAL_CONTROL_TIME * 0.001f;

    if (control == 0)
    {
        return;
    }

    pitch_motor = &control->gimbal_pitch_motor;

    if (rc_enable != 0u)
    {
        rc_relative_angle_set = pitch_motor->rc_relative_angle_set + add;
    }
    else
    {
        rc_relative_angle_set = pitch_motor->relative_angle;
        gimbal_feedforward_clear_source(pitch_motor, GIMBAL_CONTROL_SOURCE_RC);
    }

    pitch_motor->rc_relative_angle_set =
        gimbal_mit_clamp(rc_relative_angle_set,
                         pitch_motor->min_relative_angle,
                         pitch_motor->max_relative_angle);
    pitch_motor->rc_absolute_angle_set = pitch_motor->rc_relative_angle_set;

    if (auto_enable != 0u)
    {
        desired_relative_angle =
            pitch_motor->relative_angle +
            auto_aim_get_pitch_err_rad();
        desired_relative_angle =
            gimbal_mit_clamp(desired_relative_angle,
                             pitch_motor->min_relative_angle,
                             pitch_motor->max_relative_angle);
        auto_relative_angle_set =
            gimbal_auto_aim_plan_target(pitch_motor->auto_relative_angle_set,
                                        desired_relative_angle,
                                        &pitch_auto_aim_target_vel,
                                        GIMBAL_AUTO_AIM_PITCH_KP,
                                        GIMBAL_AUTO_AIM_PITCH_MAX_SPEED,
                                        GIMBAL_AUTO_AIM_PITCH_MAX_ACCEL,
                                        control_dt);
        pitch_motor->auto_relative_angle_set =
            gimbal_mit_clamp(auto_relative_angle_set,
                             pitch_motor->min_relative_angle,
                             pitch_motor->max_relative_angle);
    }
    else
    {
        pitch_motor->auto_relative_angle_set = pitch_motor->rc_relative_angle_set;
    }

    pitch_motor->auto_absolute_angle_set = pitch_motor->auto_relative_angle_set;
    pitch_motor->relative_angle_set =
        pitch_motor->rc_relative_angle_set +
        ((auto_enable != 0u) ?
             (pitch_motor->auto_relative_angle_set - pitch_motor->relative_angle) :
             0.0f);
    pitch_motor->absolute_angle_set = pitch_motor->relative_angle_set;
}

/**
  * @brief          更新绝对角目标并执行限幅
  * @retval         none
  */
__attribute__((used)) void gimbal_absolute_angle_limit(gimbal_motor_t *motor, float add)
{
    const float bias = gimbal_take_auto_aim_bias(motor);

    if (motor == 0)
    {
        return;
    }

    if (motor == &gimbal_control.gimbal_yaw_motor)
    {
        motor->absolute_angle_set += add + bias;
    }
    else
    {
        motor->absolute_angle_set =
            gimbal_clamp(motor->absolute_angle_set + add + bias,
                         motor->min_relative_angle,
                         motor->max_relative_angle);
    }
}

/**
  * @brief          执行云台电机绝对角控制
  * @retval         none
  */
__attribute__((used)) void gimbal_motor_absolute_angle_control(gimbal_motor_t *motor)
{
    float angle_get;
    float angle_set;

    if (motor == 0)
    {
        return;
    }

    motor->ref_vel =
        ((motor->rc_control_enable != 0u) ? motor->rc_ref_vel : 0.0f) +
        ((motor->auto_control_enable != 0u) ? motor->auto_ref_vel : 0.0f);
    motor->ref_accel =
        ((motor->rc_control_enable != 0u) ? motor->rc_ref_accel : 0.0f) +
        ((motor->auto_control_enable != 0u) ? motor->auto_ref_accel : 0.0f);

    if (motor == &gimbal_control.gimbal_yaw_motor)
    {
        angle_get = motor->absolute_angle;
        gimbal_calc_angle_speed_torque(motor,
                                       &motor->absolute_angle_pid,
                                       gimbal_wrap_angle(motor->absolute_angle_set - angle_get),
                                       motor->gyro);
    }
    else
    {
        angle_get = gimbal_wrap_angle(motor->absolute_angle);
        angle_set = gimbal_wrap_angle(motor->absolute_angle_set);
        gimbal_calc_feedback_torque(motor, &motor->absolute_angle_pid, angle_get, angle_set);
    }

    motor->current_set = motor->pid_torque + gimbal_calc_feedforward(motor);
    motor->output = motor->current_set;
    motor->given_current = gimbal_float_to_torque_cmd(motor->output);
}

/**
  * @brief          执行云台电机绝对角控制
  * @retval         none
  */
__attribute__((used)) void gimbal_motor_relative_angle_control(gimbal_motor_t *motor)
{
    if (motor == 0)
    {
        return;
    }

    motor->ref_vel =
        ((motor->rc_control_enable != 0u) ? motor->rc_ref_vel : 0.0f) +
        ((motor->auto_control_enable != 0u) ? motor->auto_ref_vel : 0.0f);
    motor->ref_accel =
        ((motor->rc_control_enable != 0u) ? motor->rc_ref_accel : 0.0f) +
        ((motor->auto_control_enable != 0u) ? motor->auto_ref_accel : 0.0f);

    if (motor == &gimbal_control.gimbal_yaw_motor)
    {
        gimbal_calc_angle_speed_torque(motor,
                                       &motor->relative_angle_pid,
                                       gimbal_wrap_angle(motor->relative_angle_set - motor->relative_angle),
                                       GIMBAL_YAW_FEEDBACK_SIGN *
                                           MIT_MOTOR_MEASURE[GIMBAL_YAW_MIT_INDEX].fdb.vel);
    }
    else
    {
        gimbal_calc_angle_speed_torque(motor,
                                       &motor->relative_angle_pid,
                                       motor->relative_angle_set - motor->relative_angle,
                                       motor->gyro);
    }

    motor->current_set = motor->pid_torque + gimbal_calc_feedforward(motor);
    motor->output = motor->current_set;
    motor->given_current = gimbal_float_to_torque_cmd(motor->output);
}

/**
  * @brief          执行云台电机 RAW 输出控制
  * @retval         none
  */
__attribute__((used)) void gimbal_motor_raw_angle_control(gimbal_motor_t *motor)
{
    if (motor == 0)
    {
        return;
    }

    motor->current_set = motor->raw_cmd;
    motor->output = motor->raw_cmd;
    motor->pid_torque = 0.0f;
    motor->ff_torque = 0.0f;
    motor->static_friction_comp = 0.0f;
    motor->rc_control_enable = 0u;
    motor->auto_control_enable = 0u;
    motor->given_current = gimbal_float_to_torque_cmd(motor->output);
}

/**
  * @brief          初始化云台 PID 控制器
  * @retval         none
  */
__attribute__((used)) void gimbal_pid_init(gimbal_pid_t *pid, float kp, float ki, float kd, float max_out, float max_iout)
{
    float pid_param[3];

    if (pid == NULL)
    {
        return;
    }

    pid_param[0] = kp;
    pid_param[1] = ki;
    pid_param[2] = kd;
    PID_init(pid, PID_POSITION, pid_param, max_out, max_iout);
}

/**
  * @brief          清空云台 PID 控制器状态
  * @retval         none
  */
__attribute__((used)) void gimbal_pid_clear(gimbal_pid_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    PID_clear(pid);
}

/**
  * @brief          计算云台 PID 输出
  * @retval         none
  */
__attribute__((used)) float gimbal_pid_calc(gimbal_pid_t *pid, float get, float set, float error_delta)
{
    (void)error_delta;

    if (pid == NULL)
    {
        return 0.0f;
    }

    return PID_Calc(pid, get, set);
}

/**
  * @brief          将角度归一化到 [-pi, pi]
  * @retval         none
  */
static float gimbal_wrap_angle(float angle)
{
    while (angle > GIMBAL_PI)
    {
        angle -= 2.0f * GIMBAL_PI;
    }
    while (angle < -GIMBAL_PI)
    {
        angle += 2.0f * GIMBAL_PI;
    }
    return angle;
}

/**
  * @brief          读取并清除单轴自瞄角度补偿量
  * @retval         none
  */
static float gimbal_take_auto_aim_bias(gimbal_motor_t *motor)
{
    float bias = 0.0f;

    (void)motor;

    return bias;
}

/**
  * @brief          对浮点数执行区间限幅
  * @retval         none
  */
static float gimbal_clamp(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }
    if (value < min_value)
    {
        return min_value;
    }
    return value;
}

/**
  * @brief          计算单轴静摩擦补偿原始量
  * @retval         静摩擦补偿原始量
  */
static float gimbal_calc_static_friction_comp_raw(const gimbal_motor_t *motor, float angle_error)
{
    float abs_error;
    float comp_scale;
    float deadband;
    float fullband;

    if (motor == &gimbal_control.gimbal_pitch_motor)
    {
        deadband = GIMBAL_PITCH_STATIC_FRICTION_DEADBAND;
        fullband = GIMBAL_PITCH_STATIC_FRICTION_FULLBAND;
    }
    else
    {
        deadband = GIMBAL_YAW_STATIC_FRICTION_DEADBAND;
        fullband = GIMBAL_YAW_STATIC_FRICTION_FULLBAND;
    }

    abs_error = fabsf(angle_error);
    if (abs_error <= deadband)
    {
        return 0.0f;
    }

    if (fullband <= deadband ||
        abs_error >= fullband)
    {
        comp_scale = 1.0f;
    }
    else
    {
        comp_scale =
            (abs_error - deadband) /
            (fullband - deadband);
    }

    if (angle_error > 0.0f)
    {
        if (motor == &gimbal_control.gimbal_pitch_motor)
        {
            return GIMBAL_PITCH_STATIC_FRICTION_COMP_UP * comp_scale;
        }
        return GIMBAL_YAW_STATIC_FRICTION_COMP * comp_scale;
    }
    if (angle_error < 0.0f)
    {
        if (motor == &gimbal_control.gimbal_pitch_motor)
        {
            return -GIMBAL_PITCH_STATIC_FRICTION_COMP_DOWN * comp_scale;
        }
        return -GIMBAL_YAW_STATIC_FRICTION_COMP * comp_scale;
    }
    return 0.0f;
}

/**
  * @brief          更新单轴静摩擦补偿输出
  * @retval         静摩擦补偿输出
  */
static float gimbal_update_static_friction_comp(gimbal_motor_t *motor, float angle_error)
{
    float alpha;
    float comp_cmd;

    comp_cmd = gimbal_calc_static_friction_comp_raw(motor, angle_error);
    if (motor == 0)
    {
        return comp_cmd;
    }

    if (motor == &gimbal_control.gimbal_pitch_motor)
    {
        alpha = gimbal_clamp(GIMBAL_PITCH_STATIC_FRICTION_FILTER_ALPHA, 0.0f, 1.0f);
    }
    else
    {
        alpha = gimbal_clamp(GIMBAL_YAW_STATIC_FRICTION_FILTER_ALPHA, 0.0f, 1.0f);
    }
    motor->static_friction_comp +=
        alpha * (comp_cmd - motor->static_friction_comp);

    return motor->static_friction_comp;
}

/**
  * @brief          将浮点输出限制为力矩命令范围
  * @retval         力矩命令
  */
static float gimbal_float_to_torque_cmd(float output)
{
    return gimbal_clamp(output, T_MIN, T_MAX);
}

/**
  * @brief          计算单轴惯量前馈力矩
  * @retval         前馈力矩
  */
static float gimbal_calc_feedforward(gimbal_motor_t *motor)
{
    float velocity_torque = 0.0f;

    if (motor == 0)
    {
        return 0.0f;
    }

    if (motor == &gimbal_control.gimbal_pitch_motor)
    {
        velocity_torque = PITCH_VELOCITY_FF_GAIN * motor->ref_vel;
    }

    motor->ff_torque = velocity_torque + motor->inertia_kgm2 * motor->ref_accel;

    return motor->ff_torque;
}

/**
  * @brief          计算单轴角度反馈力矩
  * @retval         反馈力矩
  */
static float gimbal_calc_feedback_torque(gimbal_motor_t *motor, gimbal_pid_t *angle_pid, float angle_get, float angle_set)
{
    float angle_torque;
    float angle_error;

    if (motor == 0 || angle_pid == 0)
    {
        return 0.0f;
    }

    motor->gyro_set = motor->ref_vel;
    angle_error = angle_set - angle_get;
    angle_torque = gimbal_pid_calc(angle_pid, angle_get, angle_set, 0.0f);
    motor->pid_torque =
        gimbal_clamp(angle_torque + gimbal_update_static_friction_comp(motor, angle_error),
                     -angle_pid->max_out,
                     angle_pid->max_out);

    return motor->pid_torque;
}

/**
  * @brief          根据角度误差和角速度误差计算控制力矩
  * @retval         控制力矩
  */
static float gimbal_calc_angle_speed_torque(gimbal_motor_t *motor, gimbal_pid_t *pid,
                                           float angle_error, float speed_get)
{
    float speed_error;
    float output;

    if (motor == 0 || pid == 0)
    {
        return 0.0f;
    }

    motor->gyro_set = motor->ref_vel;
    speed_error = motor->gyro_set - speed_get;

    pid->set = angle_error;
    pid->fdb = 0.0f;
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = angle_error;
    pid->Pout = pid->Kp * angle_error;
    pid->Iout = 0.0f;
    pid->Dout = pid->Kd * speed_error;

    output = pid->Pout + pid->Dout + gimbal_update_static_friction_comp(motor, angle_error);
    output = gimbal_clamp(output, -pid->max_out, pid->max_out);
    pid->out = output;
    motor->pid_torque = output;

    return motor->pid_torque;
}

/**
  * @brief          清空云台两轴全部 PID 状态
  * @retval         none
  */
static void gimbal_total_pid_clear(gimbal_control_t *control)
{
    if (control == 0)
    {
        return;
    }

    gimbal_pid_clear(&control->gimbal_yaw_motor.absolute_angle_pid);
    gimbal_pid_clear(&control->gimbal_yaw_motor.relative_angle_pid);
    gimbal_pid_clear(&control->gimbal_pitch_motor.absolute_angle_pid);
    gimbal_pid_clear(&control->gimbal_pitch_motor.relative_angle_pid);
    gimbal_pid_clear(&pitch_6020_speed_pid);
}

/**
  * @brief          初始化云台控制结构体
  * @retval         none
  */
__attribute__((used)) void gimbal_init(gimbal_control_t *control)
{
    if (control == 0)
    {
        return;
    }

    memset(control, 0, sizeof(*control));
    pitch_6020_last_ecd = 0U;
    pitch_6020_feedback_init = 0U;
    pitch_6020_angle_rad = 0.0f;
    pitch_6020_reference_ecd = 0U;
    pitch_6020_speed_feedback_rad_s = 0.0f;

    control->gimbal_rc_ctrl = get_remote_control_point();
    control->gimbal_INT_angle_point = get_INS_angle_point();
    control->gimbal_INT_gyro_point = get_gyro_data_point();

    control->gimbal_yaw_motor.mode = GIMBAL_MOTOR_RAW;
    control->gimbal_yaw_motor.last_mode = GIMBAL_MOTOR_RAW;
    control->gimbal_pitch_motor.mode = GIMBAL_MOTOR_RAW;
    control->gimbal_pitch_motor.last_mode = GIMBAL_MOTOR_RAW;
    control->small_yaw.can_send_status = 1U;
    small_yaw_scan_yaw_direction = 1U;
    small_yaw_scan_pitch_direction = 1U;

    gimbal_pid_init(&control->gimbal_yaw_motor.absolute_angle_pid,
                    YAW_GYRO_ABSOLUTE_PID_KP,
                    YAW_GYRO_ABSOLUTE_PID_KI,
                    YAW_GYRO_ABSOLUTE_PID_KD,
                    YAW_GYRO_ABSOLUTE_PID_MAX_OUT,
                    YAW_GYRO_ABSOLUTE_PID_MAX_IOUT);
    gimbal_pid_init(&control->gimbal_yaw_motor.relative_angle_pid,
                    YAW_ENCODE_RELATIVE_PID_KP,
                    YAW_ENCODE_RELATIVE_PID_KI,
                    YAW_ENCODE_RELATIVE_PID_KD,
                    YAW_ENCODE_RELATIVE_PID_MAX_OUT,
                    YAW_ENCODE_RELATIVE_PID_MAX_IOUT);

    gimbal_pid_init(&control->gimbal_pitch_motor.relative_angle_pid,
                    PITCH_6020_ANGLE_PID_KP,
                    PITCH_6020_ANGLE_PID_KI,
                    PITCH_6020_ANGLE_PID_KD,
                    PITCH_6020_ANGLE_PID_MAX_OUT,
                    0.0f);
    gimbal_pid_init(&pitch_6020_speed_pid,
                    PITCH_6020_SPEED_PID_KP,
                    PITCH_6020_SPEED_PID_KI,
                    PITCH_6020_SPEED_PID_KD,
                    PITCH_6020_SPEED_PID_MAX_OUT,
                    PITCH_6020_SPEED_PID_MAX_IOUT);

    control->gimbal_pitch_motor.max_relative_angle = 0.0f;
    control->gimbal_pitch_motor.min_relative_angle = 0.0f;
    control->gimbal_yaw_motor.inertia_kgm2 = YAW_INERTIA_KGM2;
    control->gimbal_pitch_motor.inertia_kgm2 = PITCH_INERTIA_KGM2;
		
//		Motor_MIT_MODE(&hfdcan1, DM_YAW_CAN_ID);
    gimbal_pid_init(&small_yaw_angle_pid,
                    SMALL_YAW_ANGLE_PID_KP,
                    SMALL_YAW_ANGLE_PID_KI,
                    SMALL_YAW_ANGLE_PID_KD,
                    SMALL_YAW_ANGLE_PID_MAX_OUT,
                    0.0f);
    gimbal_pid_init(&small_yaw_speed_pid,
                    SMALL_YAW_SPEED_PID_KP,
                    SMALL_YAW_SPEED_PID_KI,
                    SMALL_YAW_SPEED_PID_KD,
                    SMALL_YAW_SPEED_PID_MAX_OUT,
                    SMALL_YAW_SPEED_PID_MAX_IOUT);
    control->small_yaw.accel_ff_gain = SMALL_YAW_ACCEL_FF_GAIN;
    control->small_yaw.ff_max_current_a = SMALL_YAW_FF_MAX_CURRENT_A;
    control->small_yaw.velocity_ff_gain = SMALL_YAW_VELOCITY_FF_GAIN;
    Motor_ENABLE(&hfdcan1, DM_BIG_YAW_CAN_ID);

    gimbal_total_pid_clear(control);
    gimbal_feedback_update(control);

    control->gimbal_yaw_motor.absolute_angle_set = control->gimbal_yaw_motor.absolute_angle;
    control->gimbal_yaw_motor.relative_angle_set = control->gimbal_yaw_motor.relative_angle;
    control->gimbal_yaw_motor.rc_absolute_angle_set = control->gimbal_yaw_motor.absolute_angle;
    control->gimbal_yaw_motor.rc_relative_angle_set = control->gimbal_yaw_motor.relative_angle;
    control->gimbal_yaw_motor.auto_absolute_angle_set = control->gimbal_yaw_motor.absolute_angle;
    control->gimbal_yaw_motor.auto_relative_angle_set = control->gimbal_yaw_motor.relative_angle;
    control->gimbal_yaw_motor.gyro_set = control->gimbal_yaw_motor.gyro;

    control->gimbal_pitch_motor.absolute_angle_set = control->gimbal_pitch_motor.absolute_angle;
    control->gimbal_pitch_motor.relative_angle_set = control->gimbal_pitch_motor.relative_angle;
    control->gimbal_pitch_motor.rc_absolute_angle_set = control->gimbal_pitch_motor.absolute_angle;
    control->gimbal_pitch_motor.rc_relative_angle_set = control->gimbal_pitch_motor.relative_angle;
    control->gimbal_pitch_motor.auto_absolute_angle_set = control->gimbal_pitch_motor.absolute_angle;
    control->gimbal_pitch_motor.auto_relative_angle_set = control->gimbal_pitch_motor.relative_angle;
    control->gimbal_pitch_motor.gyro_set = control->gimbal_pitch_motor.gyro;

}

/**
  * @brief          设置云台控制模式
  * @retval         none
  */
__attribute__((used)) void gimbal_set_mode(gimbal_control_t *control)
{
    if (control == 0)
    {
        return;
    }

    gimbal_behaviour_mode_set(control);
    if (gimbal_pitch_6020_feedback_ready() == 0u)
    {
        gimbal_motor_zero_output(&control->gimbal_pitch_motor);
    }
}

/**
  * @brief          更新云台反馈数据
  * @retval         none
  */

static void gimbal_pitch_feedback_update(gimbal_control_t *control,
                                         float pitch_motor_pos,
                                         uint8_t pitch_feedback_ready)
{
    if (control == 0)
    {
        return;
    }

    if (pitch_feedback_ready == 0u)
    {
        control->gimbal_pitch_motor.relative_speed = 0.0f;
        control->gimbal_pitch_motor.relative_speed_update_init = 0u;
    }
    else if (control->gimbal_pitch_motor.angle_offset_init == 0u)
    {
        control->gimbal_pitch_motor.angle_offset = 0.0f;
        control->gimbal_pitch_motor.absolute_angle = pitch_motor_pos;
        control->gimbal_pitch_motor.relative_angle = pitch_motor_pos;
        control->gimbal_pitch_motor.relative_angle_set = pitch_motor_pos;
        control->gimbal_pitch_motor.rc_relative_angle_set = pitch_motor_pos;
        control->gimbal_pitch_motor.auto_relative_angle_set = pitch_motor_pos;
        control->gimbal_pitch_motor.relative_angle_last = pitch_motor_pos;
        control->gimbal_pitch_motor.relative_speed = 0.0f;
        control->gimbal_pitch_motor.relative_speed_update_init = 1u;
        control->gimbal_pitch_motor.absolute_angle_set = pitch_motor_pos;
        control->gimbal_pitch_motor.rc_absolute_angle_set = pitch_motor_pos;
        control->gimbal_pitch_motor.auto_absolute_angle_set = pitch_motor_pos;
        control->gimbal_pitch_motor.angle_offset_init = 1u;
    }
    else
    {
        control->gimbal_pitch_motor.absolute_angle = pitch_motor_pos;
        control->gimbal_pitch_motor.relative_angle = pitch_motor_pos;
        control->gimbal_pitch_motor.relative_speed =
            PITCH_6020_FEEDBACK_SIGN *
            (float)DJI_MOTOR_MEASURE[GIMBAL_PITCH_DJI_INDEX].speed_rpm *
            (2.0f * YAW_PITCH_DIRECT_PI / 60.0f);
        control->gimbal_pitch_motor.relative_angle_last =
            control->gimbal_pitch_motor.relative_angle;
        control->gimbal_pitch_motor.relative_speed_update_init = 1u;
    }

    if (control->gimbal_pitch_motor.angle_offset_init != 0u)
    {
        const float pitch_head_up_angle =
            pitch_6020_ecd_to_angle(PITCH_HEAD_UP_MOTOR_ECD);
        const float pitch_head_down_angle =
            pitch_6020_ecd_to_angle(PITCH_HEAD_DOWN_MOTOR_ECD);

        control->gimbal_pitch_motor.min_relative_angle =
            (pitch_head_up_angle < pitch_head_down_angle) ?
            pitch_head_up_angle : pitch_head_down_angle;
        control->gimbal_pitch_motor.max_relative_angle =
            (pitch_head_up_angle > pitch_head_down_angle) ?
            pitch_head_up_angle : pitch_head_down_angle;
    }
}

__attribute__((used)) void gimbal_feedback_update(gimbal_control_t *control)
{
    float pitch_motor_pos = 0.0f;
    float yaw_gyro_last = 0.0f;
    float pitch_gyro_last = 0.0f;
            float yaw_relative_wrapped = 0.0f;
    uint8_t pitch_feedback_ready = gimbal_pitch_6020_feedback_ready();
    const hwt_imu_info_t *hwt906 = hwt906_get_info();
    const float *imu_gyro = hwt906_get_gimbal_gyro_point();
    const float control_dt = (float)GIMBAL_CONTROL_TIME * 0.001f;

    if (control == 0)
    {
        return;
    }

    small_yaw_feedback_update();
    pitch_motor_pos = pitch_6020_feedback_update();
    gimbal_pitch_feedback_update(control, pitch_motor_pos, pitch_feedback_ready);

    if ((control->gimbal_INT_angle_point != 0) &&
        (hwt906 != 0) && (hwt906->angle.yaw_init_flag != 0u))
    {
        control->gimbal_yaw_motor.absolute_angle = hwt906->angle.yaw_total_rad;
        if (gimbal_yaw_online() != 0u &&
            MIT_MOTOR_MEASURE[GIMBAL_YAW_MIT_INDEX].fdb.last_fdb_time != 0U &&
            control->gimbal_yaw_motor.angle_offset_init == 0u)
        {
            control->gimbal_yaw_motor.angle_offset = GIMBAL_YAW_SAFE_ANGLE_RAD;

            yaw_relative_wrapped =
                GIMBAL_YAW_FEEDBACK_SIGN * yaw_pitch_direct_wrap_angle(
                    MIT_MOTOR_MEASURE[GIMBAL_YAW_MIT_INDEX].fdb.pos -
                    control->gimbal_yaw_motor.angle_offset);
            control->gimbal_yaw_motor.relative_angle_unwrap_init = 0u;
            control->gimbal_yaw_motor.relative_angle = yaw_relative_wrapped;
            control->gimbal_yaw_motor.relative_angle_unwrapped =
                gimbal_yaw_unwrap_update(&control->gimbal_yaw_motor,
                                         yaw_relative_wrapped);
            control->gimbal_yaw_motor.relative_angle_set = control->gimbal_yaw_motor.relative_angle;
            control->gimbal_yaw_motor.rc_relative_angle_set = control->gimbal_yaw_motor.relative_angle;
            control->gimbal_yaw_motor.auto_relative_angle_set = control->gimbal_yaw_motor.relative_angle;
            control->gimbal_yaw_motor.absolute_angle_set =
                control->gimbal_yaw_motor.absolute_angle;
            control->gimbal_yaw_motor.rc_absolute_angle_set =
                control->gimbal_yaw_motor.absolute_angle;
            control->gimbal_yaw_motor.auto_absolute_angle_set =
                control->gimbal_yaw_motor.absolute_angle;
            control->gimbal_yaw_motor.angle_offset_init = 1u;
        }
        else if (gimbal_yaw_online() != 0u &&
                 control->gimbal_yaw_motor.angle_offset_init != 0u)
        {
            yaw_relative_wrapped =
                GIMBAL_YAW_FEEDBACK_SIGN * yaw_pitch_direct_wrap_angle(
                    MIT_MOTOR_MEASURE[GIMBAL_YAW_MIT_INDEX].fdb.pos -
                    control->gimbal_yaw_motor.angle_offset);
            control->gimbal_yaw_motor.relative_angle = yaw_relative_wrapped;
            control->gimbal_yaw_motor.relative_angle_unwrapped =
                gimbal_yaw_unwrap_update(&control->gimbal_yaw_motor,
                                         yaw_relative_wrapped);
        }


    }

    if (control->gimbal_INT_gyro_point != 0)
    {
        yaw_gyro_last = control->gimbal_yaw_motor.gyro;
        pitch_gyro_last = control->gimbal_pitch_motor.gyro;

        control->gimbal_yaw_motor.gyro =
            control->gimbal_INT_gyro_point[INS_GYRO_Z_ADDRESS_OFFSET];
        control->gimbal_pitch_motor.gyro =
            imu_gyro[HWT_AXIS_PITCH];

        if (control->gimbal_yaw_motor.gyro_update_init == 0u)
        {
            control->gimbal_yaw_motor.gyro_last = control->gimbal_yaw_motor.gyro;
            control->gimbal_yaw_motor.gyro_accel = 0.0f;
            control->gimbal_yaw_motor.gyro_update_init = 1u;
        }
        else
        {
            control->gimbal_yaw_motor.gyro_last = yaw_gyro_last;
            control->gimbal_yaw_motor.gyro_accel =
                (control->gimbal_yaw_motor.gyro - yaw_gyro_last) / control_dt;
        }

        if (control->gimbal_pitch_motor.gyro_update_init == 0u)
        {
            control->gimbal_pitch_motor.gyro_last = control->gimbal_pitch_motor.gyro;
            control->gimbal_pitch_motor.gyro_accel = 0.0f;
            control->gimbal_pitch_motor.gyro_update_init = 1u;
        }
        else
        {
            control->gimbal_pitch_motor.gyro_last = pitch_gyro_last;
            control->gimbal_pitch_motor.gyro_accel =
                (control->gimbal_pitch_motor.gyro - pitch_gyro_last) / control_dt;
        }
    }
}

/**
  * @brief          处理云台模式切换过渡
  * @retval         none
  */
__attribute__((used)) void gimbal_mode_change_control_transit(gimbal_control_t *control)
{
    if (control == 0)
    {
        return;
    }

    if (control->gimbal_yaw_motor.last_mode != GIMBAL_MOTOR_RAW && control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_RAW)
    {
        control->gimbal_yaw_motor.raw_cmd = control->gimbal_yaw_motor.current_set = control->gimbal_yaw_motor.given_current;
    }
    else if (control->gimbal_yaw_motor.last_mode != GIMBAL_MOTOR_GYRO && control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_GYRO)
    {
        control->gimbal_yaw_motor.absolute_angle_set = control->gimbal_yaw_motor.absolute_angle;
        control->gimbal_yaw_motor.rc_absolute_angle_set = control->gimbal_yaw_motor.absolute_angle;
        control->gimbal_yaw_motor.auto_absolute_angle_set = control->gimbal_yaw_motor.absolute_angle;
        control->gimbal_yaw_motor.gyro_set = control->gimbal_yaw_motor.gyro;
    }
    else if (control->gimbal_yaw_motor.last_mode != GIMBAL_MOTOR_ENCODE && control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_ENCODE)
    {
        control->gimbal_yaw_motor.relative_angle_set = control->gimbal_yaw_motor.relative_angle;
        control->gimbal_yaw_motor.rc_relative_angle_set = control->gimbal_yaw_motor.relative_angle;
        control->gimbal_yaw_motor.auto_relative_angle_set = control->gimbal_yaw_motor.relative_angle;
        control->gimbal_yaw_motor.gyro_set = control->gimbal_yaw_motor.gyro;
    }
    control->gimbal_yaw_motor.last_mode = control->gimbal_yaw_motor.mode;

    if (gimbal_pitch_6020_feedback_ready() == 0u)
    {
        gimbal_motor_zero_output(&control->gimbal_pitch_motor);
        return;
    }

    if (control->gimbal_pitch_motor.last_mode != GIMBAL_MOTOR_RAW && control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_RAW)
    {
        control->gimbal_pitch_motor.raw_cmd = control->gimbal_pitch_motor.current_set = control->gimbal_pitch_motor.given_current;
    }
    else if (control->gimbal_pitch_motor.last_mode != GIMBAL_MOTOR_ENCODE && control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_ENCODE)
    {
        control->gimbal_pitch_motor.relative_angle_set = control->gimbal_pitch_motor.relative_angle;
        control->gimbal_pitch_motor.rc_relative_angle_set = control->gimbal_pitch_motor.relative_angle;
        control->gimbal_pitch_motor.auto_relative_angle_set = control->gimbal_pitch_motor.relative_angle;
        control->gimbal_pitch_motor.gyro_set = control->gimbal_pitch_motor.gyro;
    }

    control->gimbal_pitch_motor.last_mode = control->gimbal_pitch_motor.mode;
}

/**
  * @brief          生成云台目标控制量
  * @retval         none
  */
__attribute__((used)) void gimbal_set_control(gimbal_control_t *control)
{
    static float add_yaw = 0.0f;
    static float add_pitch = 0.0f;
    static uint8_t follow_origin_init = 0u;
    uint8_t auto_enable;
    uint8_t yaw_rc_enable;
    uint8_t pitch_rc_enable;
    uint8_t scan_requested;
    uint8_t scan_allowed;
    gimbal_motor_t *yaw_motor;
    gimbal_motor_t *pitch_motor;

    if (control == 0)
    {
        return;
    }

    gimbal_behaviour_control_set(&add_yaw, &add_pitch, control);
    add_yaw *= -GIMBAL_YAW_FEEDBACK_SIGN;
    small_yaw_target_rad = 0.0f;
    /* 扫描及其退出过渡提供解析前馈，禁止对自瞄跳变直接差分。 */
    control->small_yaw.ref_vel_rad_s = 0.0f;
    control->small_yaw.ref_accel_rad_s2 = 0.0f;
    auto_enable = (uint8_t)((gimbal_behaviour == GIMBAL_AUTO_SENTINEL) &&
                            (auto_aim_is_active() != 0U));
    scan_requested = (uint8_t)(gimbal_behaviour == GIMBAL_DEBUG ||
        (gimbal_behaviour == GIMBAL_AUTO_SENTINEL && auto_enable == 0U));
    scan_allowed = (uint8_t)(control->gimbal_yaw_motor.mode != GIMBAL_MOTOR_RAW &&
        !toe_is_error(DBUS_TOE) && gimbal_pitch_6020_feedback_ready() != 0U &&
        DJI_MOTOR_MEASURE[3].last_fdb_time != 0U &&
        (HAL_GetTick() - DJI_MOTOR_MEASURE[3].last_fdb_time) <= 100U);
    if (scan_allowed == 0U || auto_enable != 0U)
    {
        /* 无力、离线及自瞄接管立即取消扫描和退出过渡。 */
        control->sine_scan.active = 0U;
        control->sine_scan.blend = 0.0f;
        control->sine_scan.exit_active = 0U;
        control->sine_scan.exit_blend = 0.0f;
    }
    else if (scan_requested != 0U && control->sine_scan.exit_active != 0U)
    {
        control->sine_scan.active = 0U;
        control->sine_scan.exit_active = 0U;
        control->sine_scan.exit_blend = 0.0f;
    }
    else if (scan_requested == 0U && control->sine_scan.active != 0U &&
             control->sine_scan.exit_active == 0U)
    {
#if (GIMBAL_DEBUG_SINE_EXIT_MS > 0U)
        control->sine_scan.exit_start_tick = HAL_GetTick();
        control->sine_scan.exit_pitch_target_rad = control->gimbal_pitch_motor.relative_angle_set;
        control->sine_scan.exit_blend = 0.0f;
        control->sine_scan.exit_active = 1U;
#else
        control->sine_scan.active = 0U;
        control->sine_scan.exit_blend = 1.0f;
#endif
    }
    /* 偏航自瞄由小 yaw 执行，大 yaw 保持遥控目标控制。 */
    yaw_rc_enable = 1u;
    pitch_rc_enable =
        (uint8_t)((auto_enable == 0u) || (add_pitch != 0.0f));
    yaw_motor = &control->gimbal_yaw_motor;
    pitch_motor = &control->gimbal_pitch_motor;

    if (gimbal_behaviour != GIMBAL_RELATIVE_ANGLE)
    {
        follow_origin_init = 0u;
    }
    else if (follow_origin_init == 0u)
    {
        yaw_motor->rc_absolute_angle_set =
            yaw_motor->absolute_angle;
        yaw_motor->auto_absolute_angle_set = yaw_motor->rc_absolute_angle_set;
        follow_origin_init = 1u;
    }

    if (yaw_motor->mode == GIMBAL_MOTOR_RAW)
    {
        auto_aim_reset_delta_accum();
        gimbal_feedforward_clear(yaw_motor);
        yaw_motor->rc_control_enable = 0u;
        yaw_motor->auto_control_enable = 0u;
        yaw_motor->raw_cmd = add_yaw;
    }
    else if (yaw_motor->mode == GIMBAL_MOTOR_GYRO)
    {
        yaw_motor->rc_control_enable = yaw_rc_enable;
        yaw_motor->auto_control_enable = 0u;
        gimbal_yaw_absolute_angle_limit(control,
                                        add_yaw,
                                        yaw_rc_enable,
                                        0u);
        if (yaw_rc_enable != 0u)
        {
            gimbal_feedforward_track_target(yaw_motor,
                                            yaw_motor->rc_absolute_angle_set,
                                            GIMBAL_CONTROL_SOURCE_RC);
        }
        gimbal_feedforward_clear_source(yaw_motor, GIMBAL_CONTROL_SOURCE_AUTO);
    }
    else if (yaw_motor->mode == GIMBAL_MOTOR_ENCODE)
    {
        yaw_motor->rc_control_enable = 0u;
        yaw_motor->auto_control_enable = 0u;
        yaw_motor->rc_relative_angle_set = 0.0f;
        yaw_motor->auto_relative_angle_set = 0.0f;
        yaw_motor->relative_angle_set = 0.0f;
        gimbal_feedforward_clear(yaw_motor);
    }

    if (gimbal_pitch_6020_feedback_ready() == 0u)
    {
        gimbal_motor_zero_output(pitch_motor);
        control->sine_scan.active = 0U;
        control->sine_scan.blend = 0.0f;
        control->sine_scan.exit_active = 0U;
        control->sine_scan.exit_blend = 0.0f;
        return;
    }

    if (pitch_motor->mode == GIMBAL_MOTOR_RAW)
    {
        auto_aim_reset_delta_accum();
        gimbal_feedforward_clear(pitch_motor);
        pitch_motor->rc_control_enable = 0u;
        pitch_motor->auto_control_enable = 0u;
        pitch_motor->raw_cmd = add_pitch;
    }
    else if (pitch_motor->mode == GIMBAL_MOTOR_ENCODE)
    {
        if (auto_enable != 0U && pitch_motor->auto_control_enable == 0U)
        {
            /* 扫描转跟随时从实测角度重新规划，清除扫描阶段的速度状态。 */
            pitch_motor->auto_relative_angle_set = pitch_motor->relative_angle;
            gimbal_feedforward_clear_source(pitch_motor, GIMBAL_CONTROL_SOURCE_AUTO);
        }
        pitch_motor->rc_control_enable = pitch_rc_enable;
        pitch_motor->auto_control_enable = auto_enable;
        gimbal_pitch_relative_angle_limit(control,
                                          add_pitch,
                                          pitch_rc_enable,
                                          auto_enable);
        if (pitch_rc_enable != 0u)
        {
            gimbal_feedforward_track_target(pitch_motor,
                                            pitch_motor->rc_relative_angle_set,
                                            GIMBAL_CONTROL_SOURCE_RC);
        }
        if (auto_enable != 0u)
        {
            gimbal_feedforward_track_target(pitch_motor,
                                            pitch_motor->auto_relative_angle_set,
                                            GIMBAL_CONTROL_SOURCE_AUTO);
        }
        else
        {
            gimbal_feedforward_track_target(pitch_motor,
                                            pitch_motor->auto_relative_angle_set,
                                            GIMBAL_CONTROL_SOURCE_AUTO);
        }

    }

    if (scan_allowed != 0U && (scan_requested != 0U || control->sine_scan.exit_active != 0U))
    {
        gimbal_debug_sine_target_update(control);
    }
    else if (gimbal_behaviour == GIMBAL_AUTO_SENTINEL && auto_enable != 0U)
    {
        small_yaw_scan_update(control, auto_enable);
    }
}

/**
  * @brief          生成云台目标控制量
  * @retval         none
  */
__attribute__((used)) void gimbal_control_loop(gimbal_control_t *control)
{
    if (control == 0)
    {
        return;
    }

    small_yaw_control_allowed = 0U;
    if ((control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_RAW) ||
        (gimbal_hwt906_yaw_ready() == 0u) ||
        (gimbal_yaw_online() == 0u) || toe_is_error(DBUS_TOE))
    {
        /* RAW 无力模式和 yaw 离线保护直接零输出，不进入 yaw 闭环。 */
        gimbal_motor_zero_output(&control->gimbal_yaw_motor);
    }
    else if (control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_motor_absolute_angle_control(&control->gimbal_yaw_motor);
    }
    else if (control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_ENCODE)
    {
        gimbal_motor_relative_angle_control(&control->gimbal_yaw_motor);
    }

    if (gimbal_pitch_6020_feedback_ready() == 0u)
    {
        gimbal_motor_zero_output(&control->gimbal_pitch_motor);
    }
    else if (control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_RAW)
    {
        /* RAW 无力模式直接零输出，不进入 pitch 闭环。 */
        gimbal_motor_zero_output(&control->gimbal_pitch_motor);
    }
    else if (control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_ENCODE)
    {
        pitch_6020_control_update();
    }

    small_yaw_control_update();
}

/**
  * @brief          发送云台电机命令
  * @retval         none
  */
uint8_t dji_gimbal_data[8] = {0};
__attribute__((used)) void gimbal_send_cmd(gimbal_control_t *control)
{
    float yaw_cmd_torque;
    
    int16_t small_yaw_cmd = 0;
    int16_t pitch_6020_cmd = 0;

    if (control == 0)
    {
        return;
    }

    /* Control loop owns the command value; this layer only encodes and sends it. */
    small_yaw_cmd = small_yaw_current_to_cmd(small_yaw_current_cmd_a);
    small_yaw_current_cmd_raw = small_yaw_cmd;
    if ((control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_RAW) ||
        (gimbal_yaw_online() == 0u) || toe_is_error(DBUS_TOE))
    {
        yaw_can_set_current = 0.0f;
    }
    else
    {
        yaw_can_set_current = control->gimbal_yaw_motor.given_current;
    }

    if ((control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_RAW) ||
        (gimbal_pitch_6020_feedback_ready() == 0u) || toe_is_error(DBUS_TOE))
    {
        pitch_can_set_current = 0.0f;
    }
    else
    {
        pitch_can_set_current = control->gimbal_pitch_motor.given_current;
    }

    if ((control->gimbal_pitch_motor.mode != GIMBAL_MOTOR_RAW) &&
        (gimbal_pitch_6020_feedback_ready() != 0u) && !toe_is_error(DBUS_TOE))
    {
        pitch_6020_cmd = pitch_6020_current_to_cmd(pitch_can_set_current);
    }
    dji_gimbal_data[0] = (uint8_t)((uint16_t)small_yaw_cmd >> 8);
    dji_gimbal_data[1] = (uint8_t)small_yaw_cmd;
    dji_gimbal_data[2] = (uint8_t)((uint16_t)pitch_6020_cmd >> 8);
    dji_gimbal_data[3] = (uint8_t)pitch_6020_cmd;
		
    canx_send_data(&hfdcan2, CAN_SMALL_YAW_CURRENT_CMD_ID, dji_gimbal_data, 8U);
    yaw_cmd_torque = gimbal_output_to_mit_torque(GIMBAL_YAW_FEEDBACK_SIGN * yaw_can_set_current);
    CAN_cmd_MIT(&hfdcan1, DM_BIG_YAW_CAN_ID, 0.0f, 0.0f, 0.0f, 0.0f, yaw_cmd_torque);
}

static void small_yaw_feedback_update(void)
{
    const motor_measure_t *measure = &DJI_MOTOR_MEASURE[3];
    uint16_t ecd = measure->ecd;
    int16_t delta;
    if (measure->last_fdb_time == 0U)
    {
        return;
    }
    if (small_yaw_feedback_init == 0U)
    {
        small_yaw_angle_rad = small_yaw_ecd_to_angle_rad(ecd);
        small_yaw_target_rad = small_yaw_angle_rad;
        small_yaw_last_ecd = ecd;
        small_yaw_feedback_init = 1U;
        return;
    }
    delta = (int16_t)(ecd - small_yaw_last_ecd);
    if (delta > 4096) delta -= 8192;
    if (delta < -4096) delta += 8192;
    small_yaw_angle_rad += (float)delta * (2.0f * YAW_PITCH_DIRECT_PI / 8192.0f);
    small_yaw_last_ecd = ecd;
}

static float small_yaw_ecd_to_angle_rad(uint16_t ecd)
{
    int16_t delta = (int16_t)ecd - (int16_t)SMALL_YAW_FORWARD_ECD;
    if (delta > 4096)
    {
        delta -= 8192;
    }
    if (delta < -4096)
    {
        delta += 8192;
    }
    return (float)delta * (2.0f * YAW_PITCH_DIRECT_PI / 8192.0f);
}

static float small_yaw_clamp_target_rad(float target_rad)
{
    const float ccw_limit_rad =
        small_yaw_ecd_to_angle_rad(SMALL_YAW_CCW_LIMIT_ECD);
    const float cw_limit_rad =
        small_yaw_ecd_to_angle_rad(SMALL_YAW_CW_LIMIT_ECD);

    return gimbal_mit_clamp(target_rad,
                            (cw_limit_rad < ccw_limit_rad) ?
                            cw_limit_rad : ccw_limit_rad,
                            (cw_limit_rad > ccw_limit_rad) ?
                            cw_limit_rad : ccw_limit_rad);
}

static void gimbal_debug_sine_target_update(gimbal_control_t *control)
{
    const uint32_t tick_ms = HAL_GetTick();
    uint32_t elapsed_ms;
    float blend;
    float blend_vel = 0.0f;
    float blend_accel = 0.0f;
    gimbal_sine_scan_t *scan;
    gimbal_motor_t *pitch_motor;

    if (control == 0)
    {
        return;
    }

    scan = &control->sine_scan;
    pitch_motor = &control->gimbal_pitch_motor;
    if (scan->active == 0U)
    {
        scan->start_tick = tick_ms;
        scan->small_yaw_start_rad = small_yaw_angle_rad;
        scan->pitch_start_rad = pitch_motor->relative_angle;
        scan->active = 1U;
        scan->exit_blend = 0.0f;
    }

    /* 用本次扫描的相对时间生成正弦，避免上电时刻影响进入相位。 */
    elapsed_ms = tick_ms - scan->start_tick;
#if (GIMBAL_DEBUG_SINE_ENTRY_MS > 0U)
    blend = gimbal_mit_clamp((float)elapsed_ms / (float)GIMBAL_DEBUG_SINE_ENTRY_MS,
                             0.0f, 1.0f);
    if (elapsed_ms < GIMBAL_DEBUG_SINE_ENTRY_MS)
    {
        const float entry_s = (float)GIMBAL_DEBUG_SINE_ENTRY_MS * 0.001f;
        blend_vel = 6.0f * blend * (1.0f - blend) / entry_s;
        blend_accel = (6.0f - 12.0f * blend) / (entry_s * entry_s);
    }
    blend = blend * blend * (3.0f - 2.0f * blend);
#else
    blend = 1.0f;
#endif
    scan->blend = blend;

#if (GIMBAL_DEBUG_SMALL_YAW_SINE_ENABLE != 0U)
    {
        const float omega = 2.0f * YAW_PITCH_DIRECT_PI /
                            ((float)SMALL_YAW_DEBUG_SINE_PERIOD_MS * 0.001f);
        const float phase = 2.0f * YAW_PITCH_DIRECT_PI *
                            (float)(elapsed_ms % SMALL_YAW_DEBUG_SINE_PERIOD_MS) /
                            (float)SMALL_YAW_DEBUG_SINE_PERIOD_MS;
        const float small_yaw_sine_value =
            sinf(phase);
        const float sine_target = gimbal_mit_clamp(
            SMALL_YAW_SCAN_LIMIT_RAD * small_yaw_sine_value,
            -SMALL_YAW_SCAN_LIMIT_RAD,
            SMALL_YAW_SCAN_LIMIT_RAD);
        const float sine_vel = SMALL_YAW_SCAN_LIMIT_RAD * omega * cosf(phase);
        const float sine_accel = -omega * omega * sine_target;
        const float target = scan->small_yaw_start_rad +
                             blend * (sine_target - scan->small_yaw_start_rad);
        small_yaw_target_rad = small_yaw_clamp_target_rad(target);
        if (small_yaw_target_rad == target)
        {
            /* 对 theta0 + s(t)*(q(t)-theta0) 求导，含入口混合项。 */
            control->small_yaw.ref_vel_rad_s =
                blend_vel * (sine_target - scan->small_yaw_start_rad) + blend * sine_vel;
            control->small_yaw.ref_accel_rad_s2 =
                blend_accel * (sine_target - scan->small_yaw_start_rad) +
                2.0f * blend_vel * sine_vel + blend * sine_accel;
        }
        else
        {
            /* 目标被机械限幅时，不沿用未限幅轨迹的速度/加速度。 */
            control->small_yaw.ref_vel_rad_s = 0.0f;
            control->small_yaw.ref_accel_rad_s2 = 0.0f;
        }
    }
#endif

#if (GIMBAL_DEBUG_PITCH_SINE_ENABLE != 0U)
    {
        const float pitch_sine_value =
            sinf(2.0f * YAW_PITCH_DIRECT_PI *
                 (float)(elapsed_ms % PITCH_DEBUG_SINE_PERIOD_MS) /
                 (float)PITCH_DEBUG_SINE_PERIOD_MS);
        const float pitch_center =
            0.5f * (pitch_motor->min_relative_angle + pitch_motor->max_relative_angle);
        float pitch_half_range =
            0.5f * (pitch_motor->max_relative_angle - pitch_motor->min_relative_angle);
        if (pitch_half_range > PITCH_SCAN_LIMIT_RAD)
        {
            pitch_half_range = PITCH_SCAN_LIMIT_RAD;
        }
        const float pitch_sine_target = gimbal_mit_clamp(
            pitch_center + pitch_half_range * pitch_sine_value,
            pitch_motor->min_relative_angle,
            pitch_motor->max_relative_angle);
        const float pitch_target = gimbal_mit_clamp(
            scan->pitch_start_rad + blend * (pitch_sine_target - scan->pitch_start_rad),
            pitch_motor->min_relative_angle,
            pitch_motor->max_relative_angle);

        pitch_motor->rc_relative_angle_set = pitch_target;
        pitch_motor->auto_relative_angle_set = pitch_target;
        pitch_motor->relative_angle_set = pitch_target;
        pitch_motor->absolute_angle_set = pitch_target;
    }
#else
    (void)pitch_motor;
#endif

#if (GIMBAL_DEBUG_SINE_EXIT_MS > 0U)
    if (scan->exit_active != 0U)
    {
        const uint32_t exit_elapsed_ms = tick_ms - scan->exit_start_tick;
        const float duration_s = (float)GIMBAL_DEBUG_SINE_EXIT_MS * 0.001f;
        const float u = gimbal_mit_clamp((float)exit_elapsed_ms /
                                        (float)GIMBAL_DEBUG_SINE_EXIT_MS, 0.0f, 1.0f);
        const float s = u * u * (3.0f - 2.0f * u);
        const float s_vel = (exit_elapsed_ms < GIMBAL_DEBUG_SINE_EXIT_MS) ?
                            6.0f * u * (1.0f - u) / duration_s : 0.0f;
        const float s_accel = (exit_elapsed_ms < GIMBAL_DEBUG_SINE_EXIT_MS) ?
                              (6.0f - 12.0f * u) / (duration_s * duration_s) : 0.0f;
        scan->exit_blend = s;
#if (GIMBAL_DEBUG_SMALL_YAW_SINE_ENABLE != 0U)
        {
            const float q = small_yaw_target_rad;
            const float v = control->small_yaw.ref_vel_rad_s;
            const float a = control->small_yaw.ref_accel_rad_s2;
            /* theta=(1-s)*q：延续扫描相位，起点速度连续，终点回零且速度为零。 */
            const float target = (1.0f - s) * q;
            small_yaw_target_rad = small_yaw_clamp_target_rad(target);
            control->small_yaw.ref_vel_rad_s = (1.0f - s) * v - s_vel * q;
            control->small_yaw.ref_accel_rad_s2 =
                (1.0f - s) * a - 2.0f * s_vel * v - s_accel * q;
            if (small_yaw_target_rad != target)
            {
                control->small_yaw.ref_vel_rad_s = 0.0f;
                control->small_yaw.ref_accel_rad_s2 = 0.0f;
            }
        }
#endif
#if (GIMBAL_DEBUG_PITCH_SINE_ENABLE != 0U)
        {
            const float target = gimbal_mit_clamp(
                (1.0f - s) * pitch_motor->relative_angle_set + s * scan->exit_pitch_target_rad,
                pitch_motor->min_relative_angle, pitch_motor->max_relative_angle);
            pitch_motor->rc_relative_angle_set = target;
            pitch_motor->auto_relative_angle_set = target;
            pitch_motor->relative_angle_set = target;
            pitch_motor->absolute_angle_set = target;
        }
#endif
        if (exit_elapsed_ms >= GIMBAL_DEBUG_SINE_EXIT_MS)
        {
            scan->active = 0U;
            scan->exit_active = 0U;
        }
    }
#endif
}

static void small_yaw_scan_update(gimbal_control_t *control, uint8_t auto_enable)
{
    if (auto_enable != 0U)
    {
        small_yaw_target_rad = small_yaw_angle_rad + auto_aim_get_yaw_err_rad();
        small_yaw_target_rad = gimbal_mit_clamp(small_yaw_target_rad,
                                                -SMALL_YAW_SCAN_LIMIT_RAD,
                                                SMALL_YAW_SCAN_LIMIT_RAD);
        small_yaw_target_rad = small_yaw_clamp_target_rad(small_yaw_target_rad);
        return;
    }
    gimbal_debug_sine_target_update(control);
}

static void small_yaw_control_update(void)
{
    gimbal_small_yaw_t *small = &gimbal_control.small_yaw;
    const float control_dt = (float)GIMBAL_CONTROL_TIME * 0.001f;
    const float speed_feedback_alpha = gimbal_mit_clamp(SMALL_YAW_SPEED_FEEDBACK_ALPHA, 0.0f, 1.0f);
    float angle_error;
    float speed_set;
    float speed_feedback;
    float speed_error;
    float output;
    float integral_candidate;
    uint8_t outward_limit = 0U;

    small_yaw_control_allowed =
        (uint8_t)(gimbal_small_yaw_control_is_allowed() != false);
    if (small_yaw_control_allowed == 0U ||
        gimbal_control.gimbal_yaw_motor.mode == GIMBAL_MOTOR_RAW ||
        DJI_MOTOR_MEASURE[3].last_fdb_time == 0U ||
        (HAL_GetTick() - DJI_MOTOR_MEASURE[3].last_fdb_time) > 100U)
    {
        gimbal_pid_clear(&small_yaw_angle_pid);
        gimbal_pid_clear(&small_yaw_speed_pid);
        small_yaw_speed_feedback_rad_s = 0.0f;
        small_yaw_speed_pid.out = 0.0f;
        small_yaw_current_cmd_a = 0.0f;
        small->ref_vel_rad_s = 0.0f;
        small->ref_accel_rad_s2 = 0.0f;
        small->ff_current_a = 0.0f;
        small->pid_current_a = 0.0f;
        small->velocity_ff_rad_s = 0.0f;
        return;
    }

    small_yaw_target_rad = small_yaw_clamp_target_rad(small_yaw_target_rad);

    /* DJI 原始速度反馈与编码器角度增量同向，直接用于阻尼和速度环。 */
    speed_feedback = (float)DJI_MOTOR_MEASURE[3].speed_rpm *
                     (2.0f * YAW_PITCH_DIRECT_PI / 60.0f);
    small_yaw_speed_feedback_rad_s +=
        speed_feedback_alpha * (speed_feedback - small_yaw_speed_feedback_rad_s);

    angle_error = small_yaw_target_rad - small_yaw_angle_rad;
    small_yaw_angle_pid.set = small_yaw_target_rad;
    small_yaw_angle_pid.fdb = small_yaw_angle_rad;
    small_yaw_angle_pid.error[2] = small_yaw_angle_pid.error[1];
    small_yaw_angle_pid.error[1] = small_yaw_angle_pid.error[0];
    small_yaw_angle_pid.error[0] = angle_error;
    small_yaw_angle_pid.Pout = small_yaw_angle_pid.Kp * angle_error;
    small_yaw_angle_pid.Iout = 0.0f;
    small_yaw_angle_pid.Dout = -small_yaw_angle_pid.Kd * small_yaw_speed_feedback_rad_s;
    /* gain=1 时：v_set = v_ref + Kp*e + Kd*(v_ref-v_fdb)。
     * ref_vel 已包含 smoothstep 导数，退出扫描/自瞄时在目标生成层清零。 */
    small->velocity_ff_rad_s = 0.0f;
    if (isfinite(small->velocity_ff_gain) && small->velocity_ff_gain > 0.0f &&
        isfinite(small->ref_vel_rad_s))
    {
        const float velocity_ff = small->velocity_ff_gain *
                                  (1.0f + small_yaw_angle_pid.Kd) * small->ref_vel_rad_s;
        if (isfinite(velocity_ff))
        {
            small->velocity_ff_rad_s = velocity_ff;
        }
    }
    speed_set = gimbal_mit_clamp(SMALL_YAW_SPEED_OUTPUT_SCALE *
                                 (small_yaw_angle_pid.Pout +
                                  small_yaw_angle_pid.Dout + small->velocity_ff_rad_s),
                                 -small_yaw_angle_pid.max_out,
                                 small_yaw_angle_pid.max_out);

    /* 力矩前馈换算到电流端，不能直接加到单位为 rad/s 的角度环输出。 */
    small->ff_current_a = 0.0f;
    if (isfinite(small->accel_ff_gain) && small->accel_ff_gain > 0.0f &&
        isfinite(small->ff_max_current_a) && small->ff_max_current_a > 0.0f &&
        isfinite(small->ref_accel_rad_s2))
    {
        const float ff_limit = gimbal_mit_clamp(small->ff_max_current_a,
                                               0.0f, SMALL_YAW_6020_CURRENT_LIMIT_A);
        const float ff_current = small->accel_ff_gain * small->ref_accel_rad_s2;
        if (isfinite(ff_current))
        {
            small->ff_current_a = gimbal_mit_clamp(ff_current, -ff_limit, ff_limit);
        }
    }

    {
        const float ccw_limit_rad =
            small_yaw_ecd_to_angle_rad(SMALL_YAW_CCW_LIMIT_ECD);
        const float cw_limit_rad =
            small_yaw_ecd_to_angle_rad(SMALL_YAW_CW_LIMIT_ECD);

        if ((small_yaw_angle_rad >= ccw_limit_rad && small->ff_current_a > 0.0f) ||
            (small_yaw_angle_rad <= cw_limit_rad && small->ff_current_a < 0.0f))
        {
            small->ff_current_a = 0.0f;
        }

        if ((small_yaw_angle_rad >= ccw_limit_rad && speed_set > 0.0f) ||
            (small_yaw_angle_rad <= cw_limit_rad && speed_set < 0.0f))
        {
            speed_set = 0.0f;
            outward_limit = 1U;
        }
    }

    if (outward_limit != 0U)
    {
        small_yaw_speed_pid.Iout = 0.0f;
        small->ff_current_a = 0.0f;
    }

    /* 记录限速及机械限位处理后真正交给速度环的指令。 */
    small_yaw_angle_pid.out = speed_set;
    speed_error = speed_set - small_yaw_speed_feedback_rad_s;
    speed_error = gimbal_mit_clamp(speed_error,
                                   -SMALL_YAW_SPEED_ERROR_LIMIT_RAD_S,
                                   SMALL_YAW_SPEED_ERROR_LIMIT_RAD_S);
    if ((small_yaw_speed_pid.error[0] != 0.0f) &&
        (speed_error * small_yaw_speed_pid.error[0] < 0.0f))
    {
        small_yaw_speed_pid.Iout = 0.0f;
    }
    small_yaw_speed_pid.set = speed_set;
    small_yaw_speed_pid.fdb = small_yaw_speed_feedback_rad_s;
    small_yaw_speed_pid.error[2] = small_yaw_speed_pid.error[1];
    small_yaw_speed_pid.error[1] = small_yaw_speed_pid.error[0];
    small_yaw_speed_pid.error[0] = speed_error;
    small_yaw_speed_pid.Pout = small_yaw_speed_pid.Kp * speed_error;
    integral_candidate = small_yaw_speed_pid.Iout +
                         small_yaw_speed_pid.Ki * speed_error * control_dt;
    integral_candidate = gimbal_mit_clamp(integral_candidate,
                                          -small_yaw_speed_pid.max_iout,
                                          small_yaw_speed_pid.max_iout);
    /* 抗饱和判断包含前馈，避免总电流饱和后积分仍继续累积。 */
    output = small_yaw_speed_pid.Pout + integral_candidate + small->ff_current_a;
    if ((output < small_yaw_speed_pid.max_out && output > -small_yaw_speed_pid.max_out) ||
        (output >= small_yaw_speed_pid.max_out && speed_error < 0.0f) ||
        (output <= -small_yaw_speed_pid.max_out && speed_error > 0.0f))
    {
        small_yaw_speed_pid.Iout = integral_candidate;
    }
    small->pid_current_a = small_yaw_speed_pid.Pout + small_yaw_speed_pid.Iout;
    output = gimbal_mit_clamp(small->pid_current_a + small->ff_current_a,
                              -small_yaw_speed_pid.max_out,
                              small_yaw_speed_pid.max_out);
    small_yaw_speed_pid.Dout = 0.0f;
    small_yaw_speed_pid.out = output;
    small_yaw_current_cmd_a = output;
}

static void pitch_6020_control_update(void)
{
    gimbal_motor_t *motor = &gimbal_control.gimbal_pitch_motor;
    const float control_dt = (float)GIMBAL_CONTROL_TIME * 0.001f;
    const float speed_feedback_raw =
        PITCH_6020_FEEDBACK_SIGN *
        (float)DJI_MOTOR_MEASURE[GIMBAL_PITCH_DJI_INDEX].speed_rpm *
        (2.0f * YAW_PITCH_DIRECT_PI / 60.0f);
    float speed_feedback;
    float angle_error;
    float speed_set;
    float speed_error;
    float integral_candidate;
    float output;

    if (gimbal_pitch_6020_feedback_ready() == 0U)
    {
        gimbal_pid_clear(&motor->relative_angle_pid);
        gimbal_pid_clear(&pitch_6020_speed_pid);
        pitch_6020_speed_feedback_rad_s = 0.0f;
        motor->current_set = 0.0f;
        motor->output = 0.0f;
        motor->given_current = 0.0f;
        return;
    }

    pitch_6020_speed_feedback_rad_s +=
        SMALL_YAW_SPEED_FEEDBACK_ALPHA *
        (speed_feedback_raw - pitch_6020_speed_feedback_rad_s);
    speed_feedback = pitch_6020_speed_feedback_rad_s;
    motor->relative_speed = speed_feedback;

    motor->ref_vel =
        ((motor->rc_control_enable != 0U) ? motor->rc_ref_vel : 0.0f) +
        ((motor->auto_control_enable != 0U) ? motor->auto_ref_vel : 0.0f);
    motor->ref_accel =
        ((motor->rc_control_enable != 0U) ? motor->rc_ref_accel : 0.0f) +
        ((motor->auto_control_enable != 0U) ? motor->auto_ref_accel : 0.0f);

    angle_error = motor->relative_angle_set - pitch_6020_angle_rad;
    motor->relative_angle_pid.set = motor->relative_angle_set;
    motor->relative_angle_pid.fdb = pitch_6020_angle_rad;
    motor->relative_angle_pid.error[2] = motor->relative_angle_pid.error[1];
    motor->relative_angle_pid.error[1] = motor->relative_angle_pid.error[0];
    motor->relative_angle_pid.error[0] = angle_error;
    motor->relative_angle_pid.Pout = motor->relative_angle_pid.Kp * angle_error;
    motor->relative_angle_pid.Iout = 0.0f;
    motor->relative_angle_pid.Dout =
        -motor->relative_angle_pid.Kd * speed_feedback;
    speed_set = gimbal_mit_clamp(
        motor->relative_angle_pid.Pout + motor->relative_angle_pid.Dout,
        -motor->relative_angle_pid.max_out,
        motor->relative_angle_pid.max_out);
    motor->relative_angle_pid.out = speed_set;

    speed_error = gimbal_mit_clamp(
        speed_set - speed_feedback,
        -SMALL_YAW_SPEED_ERROR_LIMIT_RAD_S,
        SMALL_YAW_SPEED_ERROR_LIMIT_RAD_S);
    if ((pitch_6020_speed_pid.error[0] != 0.0f) &&
        (speed_error * pitch_6020_speed_pid.error[0] < 0.0f))
    {
        pitch_6020_speed_pid.Iout = 0.0f;
    }
    pitch_6020_speed_pid.set = speed_set;
    pitch_6020_speed_pid.fdb = speed_feedback;
    pitch_6020_speed_pid.error[2] = pitch_6020_speed_pid.error[1];
    pitch_6020_speed_pid.error[1] = pitch_6020_speed_pid.error[0];
    pitch_6020_speed_pid.error[0] = speed_error;
    pitch_6020_speed_pid.Pout = pitch_6020_speed_pid.Kp * speed_error;
    integral_candidate = pitch_6020_speed_pid.Iout +
                         pitch_6020_speed_pid.Ki * speed_error * control_dt;
    integral_candidate = gimbal_mit_clamp(integral_candidate,
                                          -pitch_6020_speed_pid.max_iout,
                                          pitch_6020_speed_pid.max_iout);
    output = pitch_6020_speed_pid.Pout + integral_candidate;
    if ((output < pitch_6020_speed_pid.max_out && output > -pitch_6020_speed_pid.max_out) ||
        (output >= pitch_6020_speed_pid.max_out && speed_error < 0.0f) ||
        (output <= -pitch_6020_speed_pid.max_out && speed_error > 0.0f))
    {
        pitch_6020_speed_pid.Iout = integral_candidate;
    }
    output = gimbal_mit_clamp(pitch_6020_speed_pid.Pout + pitch_6020_speed_pid.Iout,
                              -pitch_6020_speed_pid.max_out,
                              pitch_6020_speed_pid.max_out);
    pitch_6020_speed_pid.Dout = 0.0f;
    pitch_6020_speed_pid.out = output;

    motor->pid_torque = output;
    motor->ff_torque = 0.0f;
    motor->current_set = output;
    motor->output = motor->current_set;
    motor->given_current = gimbal_clamp(motor->output,
                                        -PITCH_6020_CURRENT_LIMIT_A,
                                        PITCH_6020_CURRENT_LIMIT_A);
}

static int16_t small_yaw_current_to_cmd(float current_a)
{
    float cmd = current_a * (16384.0f / SMALL_YAW_6020_CURRENT_LIMIT_A);
    if (cmd > 16384.0f) cmd = 16384.0f;
    if (cmd < -16384.0f) cmd = -16384.0f;
    return (int16_t)cmd;
}

static int16_t pitch_6020_current_to_cmd(float current_a)
{
    float cmd = PITCH_6020_FEEDBACK_SIGN * current_a *
                (16384.0f / PITCH_6020_CURRENT_LIMIT_A);
    if (cmd > 16384.0f) cmd = 16384.0f;
    if (cmd < -16384.0f) cmd = -16384.0f;
    return (int16_t)cmd;
}
#endif
