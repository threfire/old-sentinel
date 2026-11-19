/**
  * @file       Omni_chassis.c
  * @brief      四舵轮底盘控制实现
  * @note       实现反馈更新、速度规划、舵向双环、行走轮控制和两帧电流发送。
  */
#include "bsp_usart.h"
#include "bsp_fdcan.h"
#include "chassis_behaviour.h"
#include "chassis_calculate.h"
#include "chassis_power_control.h"
#include "Omni_chassis.h"
#include "cmsis_os.h"
#include "detect_task.h"
#include "hwt_imu.h"
#include "gimbal_behaviour.h"

#include "pid.h"
#include "robot_param.h"
#include "remote_control.h"

#include "user_lib.h"

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

#if (ROBOT_CHASSIS == ROBOT_CHASSIS_STEER)

static const float chassis_yaw_pid_param[3] = {
	CHASSIS_ANGLE_PD_KP,
	0.0f,
	CHASSIS_ANGLE_PD_KD
};
static const float chassis_x_order_filter = CHASSIS_ACCEL_X_NUM;
static const float chassis_y_order_filter = CHASSIS_ACCEL_Y_NUM;
static const float chassis_yaw_rate_filter_tau = CHASSIS_YAW_RATE_FILTER_TAU;
static const float chassis_yaw_set_vel_filter_tau = CHASSIS_YAW_RATE_FILTER_TAU;
static const float chassis_wheel_speed_filter_tau = CHASSIS_WHEEL_SPEED_FILTER_TAU;
static const float chassis_steer_angle_pid_param[3] = {
	CHASSIS_STEER_ANGLE_KP, CHASSIS_STEER_ANGLE_KI, CHASSIS_STEER_ANGLE_KD
};
static const float chassis_steer_speed_pid_param[3] = {
	CHASSIS_STEER_SPEED_KP, CHASSIS_STEER_SPEED_KI, CHASSIS_STEER_SPEED_KD
};

/**
  * @brief          更新底盘电机反馈、正解车速和姿态反馈
  * @retval         none
  */
__attribute__((used)) void chassis_feedback_update(chassis_move_t *chassis_move_update);
/**
  * @brief          初始化底盘结构体、控制器和运动学零偏
  * @retval         none
  */
__attribute__((used)) void chassis_init(chassis_move_t *chassis_move_init);
/**
  * @brief          刷新底盘控制模式
  * @retval         none
  */
__attribute__((used)) void chassis_set_mode(chassis_move_t *chassis_move_mode);
/**
  * @brief          处理底盘模式切换状态过渡
  * @retval         none
  */
__attribute__((used)) void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
/**
  * @brief          计算底盘目标速度并写入控制结构体
  * @retval         none
  */
__attribute__((used)) void chassis_set_contorl(chassis_move_t *chassis_move_control);
/**
  * @brief          判断底盘 HWT906 航向角是否可用
  * @retval         none
  */
static uint8_t chassis_hwt906_yaw_ready(void);
/**
  * @brief          计算底盘航向角 PD 输出
  * @retval         none
  */
static float chassis_angle_pd_calc(pid_type_def *pd, float actual, float target, uint8_t wrap_enable, float target_vel, float actual_vel);
/**
  * @brief          计算云台跟随模式底盘自转速度
  * @retval         none
  */
static float chassis_follow_yaw_control(chassis_move_t *chassis_move_follow, float manual_wz);
static void chassis_wz_speed_pi_clear(chassis_move_t *chassis_move_clear);
static float chassis_wz_speed_pi_calc(chassis_move_t *chassis_move_pi, float target_wz, float actual_wz);
static void chassis_yaw_set_vel_filter_clear(chassis_move_t *chassis_move_clear);
/**
  * @brief          底盘航向保持模式速度给定
  * @retval         none
  */
static float chassis_yaw_hold_control(chassis_move_t *chassis_move_yaw_hold, float manual_wz);
/**
  * @brief          判断底盘控制链路是否在线
  * @retval         none
  */
static uint8_t chassis_control_online(void);
/**
  * @brief          对数值执行对称限幅
  * @retval         none
  */
static float chassis_limit_abs(float value, float max_abs);
/**
  * @brief          按速度、加速度和 jerk 约束更新 S 曲线规划
  * @retval         none
  */
static float chassis_s_curve_update(float cmd, float *plan, float *accel, float max_speed, float max_accel, float max_jerk, float stop_accel, float stop_jerk);
/**
  * @brief          计算单个底盘电机速度 PID 输出
  * @retval         none
  */
static float chassis_speed_pi_calc(chassis_move_t *chassis_move_pi, uint8_t motor_idx, float error_v);
static uint8_t chassis_zero_speed_set(const chassis_move_t *chassis_move_zero_check);
/**
  * @brief          清除整车前馈和速度制动补偿状态
  * @retval         none
  */
static void chassis_body_feedforward_clear(chassis_move_t *chassis_move_ff);
/**
  * @brief          清除底盘控制状态并置零电机输出
  * @retval         none
  */
static void chassis_zero_force_clear(chassis_move_t *chassis_move_zero);
/**
  * @brief          根据整车加速度规划更新轮端前馈电流
  * @retval         none
  */
static void chassis_body_feedforward_update(chassis_move_t *chassis_move_ff);
/**
  * @brief          清除整车速度制动补偿状态
  * @retval         none
  */
static void chassis_body_velocity_brake_clear(chassis_move_t *chassis_move_brake);
/**
  * @brief          根据目标速度和正解车速更新制动补偿
  * @retval         none
  */
static void chassis_body_velocity_brake_update(chassis_move_t *chassis_move_brake);
/**
  * @brief          计算单轮模型前馈和速度环合成输出
  * @retval         电流命令值
  */
static float Model_Based_Control(uint8_t motor_idx, float set_speed, float ref_speed);
static void chassis_steer_force_distribution(const chassis_move_t *move,
                                             float ax, float ay, float alpha,
                                             float wheel_force[CHASSIS_MODULE_NUM]);
/**
  * @brief          计算底盘四轮速度环输出
  * @retval         none
  */
static void PID_Calc_Jump(chassis_move_t *chassis_pid_calc);
/**
  * @brief          根据四轮电流需求更新动态电流限幅
  * @retval         none
  */
static void chassis_dynamic_current_limit_update(chassis_move_t *chassis_move_limit);
/**
  * @brief          执行底盘速度规划、前馈、速度环和电流限幅
  * @retval         none
  */
__attribute__((used)) void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
/**
  * @brief          发送底盘四个 3508 电机电流命令
  * @retval         none
  */
__attribute__((used)) void chassis_send_cmd(chassis_move_t *chassis_move_send);
/**
  * @brief          更新底盘电机反馈、正解车速和姿态反馈
  * @retval         none
  */
__attribute__((used)) void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
	if (chassis_move_update == NULL) return;
	static float last_speed[CHASSIS_MODULE_NUM] = {0.0f};
	float wheel_speed[CHASSIS_MODULE_NUM] = {0.0f};
	float wheel_angle[CHASSIS_MODULE_NUM] = {0.0f};
	float yaw_relative_angle = 0.0f;
	const hwt_imu_info_t *hwt906 = hwt906_get_info();

	for (uint8_t i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		chassis_move_update->chassis_3508[i].speed = chassis_move_update->chassis_3508[i].chassis_motor_measure->speed_rpm / MPS_to_RPM;
		first_order_filter_cali(&chassis_move_update->wheel_speed_filter[i],
		                        chassis_move_update->chassis_3508[i].speed);
		chassis_move_update->chassis_3508[i].speed_filtered =
			chassis_move_update->wheel_speed_filter[i].out;
		chassis_move_update->chassis_3508[i].speed_rad_s = (float)chassis_move_update->chassis_3508[i].chassis_motor_measure->speed_rpm * CHASSIS_RPM_TO_RAD_PER_SEC;
		chassis_move_update->chassis_3508[i].given_current_a = (float)chassis_move_update->chassis_3508[i].chassis_motor_measure->given_current * CHASSIS_CURRENT_CMD_TO_A;
		chassis_move_update->chassis_3508[i].accel = (chassis_move_update->chassis_3508[i].speed_filtered - last_speed[i]) * CHASSIS_CONTROL_FREQUENCE;
		last_speed[i] = chassis_move_update->chassis_3508[i].speed_filtered;
		wheel_speed[i] = chassis_move_update->chassis_3508[i].speed_filtered;
		chassis_move_update->chassis_6020[i].angle =
			rad_format((float)chassis_move_update->chassis_6020[i].chassis_motor_measure->ecd *
			           CHASSIS_STEER_ECD_TO_RAD + chassis_move_update->wheel_angle_offset.now[i]);
		chassis_move_update->chassis_6020[i].speed =
			(float)chassis_move_update->chassis_6020[i].chassis_motor_measure->speed_rpm;
		wheel_angle[i] = chassis_move_update->chassis_6020[i].angle;
	}

	chas_for_cal(wheel_angle,
	             wheel_speed,
	             &chassis_move_update->vx,
	             &chassis_move_update->vy,
	             &chassis_move_update->wz);

	if ((hwt906 != NULL) && (hwt906->angle.yaw_init_flag != 0u))
	{
		if ((chassis_move_update->chassis_yaw_motor != NULL) &&
		    (chassis_move_update->chassis_yaw_motor->angle_offset_init != 0u) &&
		    (toe_is_error(YAW_GIMBAL_MOTOR_TOE) == 0U))
		{
			/* 云台绝对航向减大 yaw 关节角，得到底盘航向。 */
			yaw_relative_angle = chassis_move_update->chassis_yaw_motor->relative_angle;
			if ((chassis_move_update->chassis_mode == CHASSIS_VECTOR_SPIN) &&
			    (chassis_move_update->chassis_yaw_motor->relative_angle_unwrap_init != 0u))
			{
				yaw_relative_angle = chassis_move_update->chassis_yaw_motor->relative_angle_unwrapped;
			}
			chassis_move_update->chassis_yaw = hwt906->angle.yaw_total_rad +
				CHASSIS_GIMBAL_YAW_FEEDBACK_SIGN *
				yaw_relative_angle;
			/* 速度反馈必须沿用相对角的位置符号链。 */
			chassis_move_update->chassis_yaw_rate =
				CHASSIS_YAW_RATE_FEEDBACK_SIGN * hwt906->gyro.radps[HWT_AXIS_YAW] +
				CHASSIS_GIMBAL_YAW_FEEDBACK_SIGN *
				GIMBAL_YAW_FEEDBACK_SIGN *
				MIT_MOTOR_MEASURE[GIMBAL_YAW_MIT_INDEX].fdb.vel;
		}
		chassis_move_update->chassis_pitch = rad_format(hwt906->angle.rad[HWT_AXIS_PITCH]);
		chassis_move_update->chassis_roll = rad_format(hwt906->angle.rad[HWT_AXIS_ROLL]);
	}
	first_order_filter_cali(&chassis_move_update->chassis_yaw_rate_filter,
	                        chassis_move_update->chassis_yaw_rate);
	chassis_move_update->chassis_yaw_rate_filtered =
		chassis_move_update->chassis_yaw_rate_filter.out;

	if ((chassis_move_update->chassis_yaw_motor != NULL) &&
	    (chassis_move_update->chassis_yaw_motor->angle_offset_init != 0u) &&
	    (toe_is_error(YAW_GIMBAL_MOTOR_TOE) == 0U))
	{
		chassis_move_update->chassis_relative_angle =
			CHASSIS_GIMBAL_YAW_FEEDBACK_SIGN *
			rad_format(chassis_move_update->chassis_yaw_motor->relative_angle);
		chassis_move_update->chassis_relative_angle_vel =
			CHASSIS_GIMBAL_YAW_FEEDBACK_SIGN *
			GIMBAL_YAW_FEEDBACK_SIGN *
			MIT_MOTOR_MEASURE[GIMBAL_YAW_MIT_INDEX].fdb.vel;
	}
	else
	{
		chassis_move_update->chassis_relative_angle_vel = 0.0f;
	}
}

/**
  * @brief          初始化底盘控制结构体
  * @retval         none
  */
__attribute__((used)) void chassis_init(chassis_move_t *chassis_move_init)
{
	if (chassis_move_init == NULL) return;

	chassis_move_init->chassis_mode = CHASSIS_VECTOR_NO_MOVE;
	chassis_move_init->spin_start_tick = 0U;
	chassis_move_init->spin_wave_active = 0U;
	chassis_move_init->spin_exit_transition_active = 0U;
	chassis_move_init->chassis_RC = get_remote_control_point();
	chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();

	for(uint8_t i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		chassis_move_init->chassis_3508[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
		chassis_move_init->chassis_6020[i].chassis_motor_measure = get_chassis_motor_measure_point(i + CHASSIS_MODULE_NUM);
		PID_init(&chassis_move_init->steer_angle_pid[i], PID_POSITION,
		         chassis_steer_angle_pid_param, CHASSIS_STEER_ANGLE_MAX_RPM,
		         CHASSIS_STEER_ANGLE_MAX_IOUT);
		PID_init(&chassis_move_init->steer_speed_pid[i], PID_POSITION,
		         chassis_steer_speed_pid_param, CHASSIS_STEER_CURRENT_MAX,
		         CHASSIS_STEER_CURRENT_MAX_IOUT);
		chassis_move_init->chassis_3508[i].speed_filtered = 0.0f;
		chassis_move_init->model_3508_out[i] = 0.0f;
		chassis_move_init->model_accel[i] = 0.0f;
		chassis_move_init->model_last_speed_set[i] = 0.0f;
		chassis_move_init->speed_pi_iout[i] = 0.0f;
		chassis_move_init->speed_pid_last_error[i] = 0.0f;
		chassis_move_init->stop_brake_active[i] = 0u;
		chassis_move_init->motor_current_limit_a[i] = CHASSIS_CURRENT_BASE_LIMIT_A;
		chassis_move_init->last_motor_current_limit_a[i] = CHASSIS_CURRENT_BASE_LIMIT_A;
		first_order_filter_init(&chassis_move_init->wheel_speed_filter[i],
		                        CHASSIS_CONTROL_TIME,
		                        &chassis_wheel_speed_filter_tau);
	}
	PID_init(&chassis_move_init->chassis_angle_pid, PID_USUAL, chassis_yaw_pid_param, CHASSIS_ANGLE_PD_MAX_OUT, 0.0f);

	first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, &chassis_x_order_filter);
	first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, &chassis_y_order_filter);

	chassis_move_init->vx_max_speed =  NORMAL_MAX_CHASSIS_SPEED_X;
	chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
	chassis_move_init->vy_max_speed =  NORMAL_MAX_CHASSIS_SPEED_Y;
	chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
	chassis_move_init->vx = 0.0f;
	chassis_move_init->vy = 0.0f;
	chassis_move_init->wz = 0.0f;
	chassis_move_init->vx_plan = 0.0f;
	chassis_move_init->vy_plan = 0.0f;
	chassis_move_init->wz_plan = 0.0f;
	chassis_move_init->vx_plan_accel = 0.0f;
	chassis_move_init->vy_plan_accel = 0.0f;
	chassis_move_init->wz_plan_accel = 0.0f;
	chassis_move_init->chassis_relative_angle = 0.0f;
	chassis_move_init->chassis_relative_angle_vel = 0.0f;
	chassis_move_init->chassis_relative_angle_target = 0.0f;
	chassis_move_init->chassis_relative_angle_set = 0.0f;
	chassis_move_init->chassis_relative_angle_set_vel = 0.0f;
	chassis_move_init->chassis_yaw_target = 0.0f;
	chassis_move_init->chassis_yaw_set = 0.0f;
	chassis_move_init->chassis_yaw_set_vel = 0.0f;
	chassis_move_init->chassis_yaw_set_vel_filtered = 0.0f;
	chassis_body_feedforward_clear(chassis_move_init);
	chassis_move_init->chassis_yaw_rate = 0.0f;
	chassis_move_init->chassis_yaw_rate_filtered = 0.0f;
	first_order_filter_init(&chassis_move_init->chassis_yaw_rate_filter,
	                        CHASSIS_CONTROL_TIME,
	                        &chassis_yaw_rate_filter_tau);
	first_order_filter_init(&chassis_move_init->chassis_yaw_set_vel_filter,
	                        CHASSIS_CONTROL_TIME,
	                        &chassis_yaw_set_vel_filter_tau);
	chassis_move_init->wz_speed_pi_iout = 0.0f;

	chassis_wheel_angle_offset_init();

	chassis_feedback_update(chassis_move_init);
	for(uint8_t i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		chassis_move_init->chassis_6020[i].angle_set = chassis_move_init->chassis_6020[i].angle;
	}
}

/**
  * @brief          刷新底盘控制模式
  * @retval         none
  */
__attribute__((used)) void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
	if(chassis_move_mode == NULL) return;
	chassis_behaviour_mode_set(chassis_move_mode);
}

/**
  * @brief          处理底盘模式切换过渡
  * @retval         none
  */
__attribute__((used)) void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
	uint8_t i;

	if(chassis_move_transit == NULL) return;

	if((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_MOVE) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_MOVE)
	{
		chassis_move_transit->spin_exit_transition_active = 0U;
		chassis_move_transit->chassis_relative_angle_target = 0.0f;
		chassis_move_transit->chassis_relative_angle_set = 0.0f;
		chassis_move_transit->chassis_relative_angle_set_vel = 0.0f;
		chassis_move_transit->chassis_yaw_target = chassis_move_transit->chassis_yaw;
		chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
		chassis_move_transit->chassis_yaw_set_vel = 0.0f;
		chassis_move_transit->wz_set = 0.0f;
		chassis_move_transit->wz_plan = 0.0f;
		chassis_move_transit->wz_plan_accel = 0.0f;
		PID_clear(&chassis_move_transit->chassis_angle_pid);
		chassis_wz_speed_pi_clear(chassis_move_transit);
		chassis_yaw_set_vel_filter_clear(chassis_move_transit);
		chassis_body_feedforward_clear(chassis_move_transit);
		chassis_zero_force_clear(chassis_move_transit);
	}
	else if((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
	{
		if (chassis_move_transit->last_chassis_mode == CHASSIS_VECTOR_SPIN)
		{
			/* 退出小陀螺时从当前实测角速度开始，避免轮速目标瞬间反向。 */
			chassis_move_transit->spin_exit_transition_active = 1U;
			chassis_move_transit->wz_plan = chassis_move_transit->wz;
			chassis_move_transit->wz_plan_accel = 0.0f;
			for (i = 0U; i < CHASSIS_MODULE_NUM; i++)
			{
				chassis_move_transit->speed_pi_iout[i] = 0.0f;
				chassis_move_transit->speed_pid_last_error[i] =
					chassis_move_transit->chassis_3508[i].speed_filtered;
				chassis_move_transit->model_accel[i] = 0.0f;
				chassis_move_transit->stop_brake_active[i] = 0U;
			}
		}
		else
		{
			chassis_move_transit->spin_exit_transition_active = 0U;
		}
		chassis_move_transit->chassis_relative_angle_target = 0.0f;
		chassis_move_transit->chassis_relative_angle_set = 0.0f;
		chassis_move_transit->chassis_relative_angle_set_vel = 0.0f;
		PID_clear(&chassis_move_transit->chassis_angle_pid);
		chassis_wz_speed_pi_clear(chassis_move_transit);
		chassis_yaw_set_vel_filter_clear(chassis_move_transit);
		chassis_body_feedforward_clear(chassis_move_transit);
	}
	else if((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_YAW_HOLD) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_YAW_HOLD)
	{
		chassis_move_transit->spin_exit_transition_active = 0U;
		chassis_move_transit->chassis_yaw_target = chassis_move_transit->chassis_yaw;
		chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
		chassis_move_transit->chassis_yaw_set_vel = 0.0f;
		PID_clear(&chassis_move_transit->chassis_angle_pid);
		chassis_yaw_set_vel_filter_clear(chassis_move_transit);
		chassis_body_feedforward_clear(chassis_move_transit);
	}
	else if((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_SPIN) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_SPIN)
	{
		chassis_move_transit->spin_exit_transition_active = 0U;
		chassis_move_transit->chassis_relative_angle_target = chassis_move_transit->chassis_yaw;
		chassis_move_transit->chassis_relative_angle_set = chassis_move_transit->chassis_yaw;
		chassis_move_transit->chassis_relative_angle_set_vel = 0.0f;
		PID_clear(&chassis_move_transit->chassis_angle_pid);
		chassis_yaw_set_vel_filter_clear(chassis_move_transit);
		chassis_body_feedforward_clear(chassis_move_transit);
	}

	chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

/**
  * @brief          计算底盘目标速度并写入控制结构体
  * @retval         none
  */
__attribute__((used)) void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
	float vector[2];

	if (chassis_move_control == NULL) return;

	float vx_set = 0.0f, vy_set = 0.0f, wz_set = 0.0f;

	// 由行为层生成基础速度指令
	chassis_behaviour_control_set(&vx_set, &vy_set, &wz_set, chassis_move_control);

	if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_MOVE)
	{
		chassis_move_control->vx_set = vx_set = 0.0f;
		chassis_move_control->vy_set = vy_set = 0.0f;
		chassis_move_control->wz_set = wz_set = 0.0f;
	}
	else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
	{
		if (chassis_move_control->spin_exit_transition_active != 0U)
		{
			/* 退出过渡期间固定减速到零，暂停跟随外环。 */
			chassis_move_control->wz_set = 0.0f;
		}
		else
		{
			chassis_move_control->wz_set = chassis_follow_yaw_control(chassis_move_control, wz_set);
		}

		chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
		chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
	}
	else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_YAW_HOLD)
	{
		chassis_move_control->wz_set = chassis_yaw_hold_control(chassis_move_control, wz_set);

		chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
		chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
	}
	else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_SPIN)
	{
		const hwt_imu_info_t *hwt906 = hwt906_get_info();
		if ((hwt906 == NULL) || (hwt906->angle.yaw_init_flag == 0u) ||
		    (chassis_move_control->chassis_yaw_motor == NULL) ||
		    (chassis_move_control->chassis_yaw_motor->angle_offset_init == 0u) ||
		    (toe_is_error(YAW_GIMBAL_MOTOR_TOE) != 0U) ||
		    (MIT_MOTOR_MEASURE[GIMBAL_YAW_MIT_INDEX].fdb.last_fdb_time == 0U))
		{
			chassis_move_control->vx_set = 0.0f;
			chassis_move_control->vy_set = 0.0f;
			chassis_move_control->wz_set = 0.0f;
			return;
		}

		/* 遥控平移以 HWT906 航向为参考，换算到底盘自身坐标。 */
		vector[0] = vx_set;
		vector[1] = vy_set;
		/* 使用 HWT906 解算的底盘航向，将航向坐标系速度换算到底盘坐标系。 */
		vector_rotate(hwt906->angle.yaw_total_rad - chassis_move_control->chassis_yaw, vector);
		chassis_move_control->vx_set = fp32_constrain(vector[0], chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
		chassis_move_control->vy_set = fp32_constrain(vector[1], chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
		chassis_move_control->wz_set = wz_set;
	}
}

/**
  * @brief          判断底盘 HWT906 航向角是否可用
  * @retval         1 表示可用，0 表示不可用
  */
static uint8_t chassis_hwt906_yaw_ready(void)
{
	const hwt_imu_info_t *hwt906 = hwt906_get_info();

	if ((hwt906 != NULL) && (hwt906->angle.yaw_init_flag != 0u))
	{
		return 1u;
	}

	return 0u;
}

/**
  * @brief          判断底盘控制链路是否在线
  * @retval         1 表示在线，0 表示离线
  */
static uint8_t chassis_control_online(void)
{
	if (toe_is_error(DBUS_TOE) != 0U) return 0u;
	for (uint8_t i = CHASSIS_MOTOR1_TOE; i <= CHASSIS_MOTOR8_TOE; i++)
	{
		if (toe_is_error(i) != 0U) return 0u;
	}

	return 1u;
}

/**
  * @brief          清除整车前馈和速度制动补偿状态
  * @retval         none
  */
static void chassis_body_feedforward_clear(chassis_move_t *chassis_move_ff)
{
	uint8_t i;

	if (chassis_move_ff == NULL) return;

	chassis_move_ff->last_vx_plan_ff = 0.0f;
	chassis_move_ff->last_vy_plan_ff = 0.0f;
	chassis_move_ff->last_wz_plan_ff = 0.0f;
	chassis_move_ff->body_ff_ax = 0.0f;
	chassis_move_ff->body_ff_ay = 0.0f;
	chassis_move_ff->body_ff_alpha = 0.0f;
	for (i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		chassis_move_ff->body_ff_current[i] = 0.0f;
	}
	chassis_body_velocity_brake_clear(chassis_move_ff);
	chassis_move_ff->body_ff_init = 0u;
}

/**
  * @brief          清除底盘控制状态并置零电机输出
  * @retval         none
  */
static void chassis_zero_force_clear(chassis_move_t *chassis_move_zero)
{
	uint8_t i;

	if (chassis_move_zero == NULL) return;

	chassis_move_zero->vx_set = 0.0f;
	chassis_move_zero->vy_set = 0.0f;
	chassis_move_zero->wz_set = 0.0f;
	chassis_move_zero->last_vx_set = 0.0f;
	chassis_move_zero->last_vy_set = 0.0f;
	chassis_move_zero->last_wz_set = 0.0f;
	chassis_move_zero->vx_plan = 0.0f;
	chassis_move_zero->vy_plan = 0.0f;
	chassis_move_zero->wz_plan = 0.0f;
	chassis_move_zero->vx_plan_accel = 0.0f;
	chassis_move_zero->vy_plan_accel = 0.0f;
	chassis_move_zero->wz_plan_accel = 0.0f;
	chassis_move_zero->chassis_cmd_slow_set_vx.out = 0.0f;
	chassis_move_zero->chassis_cmd_slow_set_vy.out = 0.0f;
	PID_clear(&chassis_move_zero->chassis_angle_pid);
	chassis_wz_speed_pi_clear(chassis_move_zero);
	chassis_yaw_set_vel_filter_clear(chassis_move_zero);
	chassis_body_feedforward_clear(chassis_move_zero);

	for (i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		chassis_move_zero->chassis_3508[i].speed_set = 0.0f;
		chassis_move_zero->chassis_3508[i].give_current = 0;
		chassis_move_zero->chassis_6020[i].angle_set = chassis_move_zero->chassis_6020[i].angle;
		chassis_move_zero->chassis_6020[i].give_current = 0;
		PID_clear(&chassis_move_zero->steer_angle_pid[i]);
		PID_clear(&chassis_move_zero->steer_speed_pid[i]);
		chassis_move_zero->model_3508_out[i] = 0.0f;
		chassis_move_zero->model_accel[i] = 0.0f;
		chassis_move_zero->model_last_speed_set[i] = 0.0f;
		chassis_move_zero->speed_pi_iout[i] = 0.0f;
		chassis_move_zero->speed_pid_last_error[i] = 0.0f;
		chassis_move_zero->stop_brake_active[i] = 0u;
	}
}

/**
  * @brief          将轮端力换算为 3508 电流命令
  * @retval         电流命令值
  */
static float chassis_body_force_to_current_cmd(float force_n)
{
	float torque_output;
	float current_a;
	float current_cmd;

	torque_output = (force_n * Wheel_Radius) / CHASSIS_EFFICIENCY;
	torque_output = chassis_limit_abs(torque_output, M3508_MAX_CONT_TORQUE * M3508_REDUCTION_RATIO);
	current_a = torque_output / M3508_TORQUE_CONSTANT;
	current_cmd = current_a * (CHASSIS_CURRENT_CMD_FULL_SCALE / CHASSIS_CURRENT_FULL_SCALE_A);

	return chassis_limit_abs(current_cmd, CHASSIS_BODY_FF_MAX_CURRENT_CMD);
}

/* 将车体合力和 yaw 力矩分配到当前四个舵轮的行走方向。 */
static void chassis_steer_force_distribution(const chassis_move_t *move,
                                             float ax, float ay, float alpha,
                                             float wheel_force[CHASSIS_MODULE_NUM])
{
	static const float x_m[CHASSIS_MODULE_NUM] = {
		CHASSIS_STEER_HALF_LENGTH_M, -CHASSIS_STEER_HALF_LENGTH_M,
		-CHASSIS_STEER_HALF_LENGTH_M, CHASSIS_STEER_HALF_LENGTH_M
	};
	static const float y_m[CHASSIS_MODULE_NUM] = {
		CHASSIS_STEER_HALF_WIDTH_M, CHASSIS_STEER_HALF_WIDTH_M,
		-CHASSIS_STEER_HALF_WIDTH_M, -CHASSIS_STEER_HALF_WIDTH_M
	};
	const float radius_sq = CHASSIS_STEER_HALF_LENGTH_M * CHASSIS_STEER_HALF_LENGTH_M +
	                        CHASSIS_STEER_HALF_WIDTH_M * CHASSIS_STEER_HALF_WIDTH_M;
	const float yaw_gain = CHASSIS_BODY_FF_YAW_INERTIA_KGM2 * alpha /
	                       ((float)CHASSIS_MODULE_NUM * radius_sq);

	for (uint8_t i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		float force_x = ROBOT_MASS * ax / (float)CHASSIS_MODULE_NUM - yaw_gain * y_m[i];
		float force_y = ROBOT_MASS * ay / (float)CHASSIS_MODULE_NUM + yaw_gain * x_m[i];
		float angle = move->chassis_6020[i].angle;
		wheel_force[i] = force_x * cosf(angle) + force_y * sinf(angle);
	}
}

/**
  * @brief          根据整车加速度规划更新轮端前馈电流
  * @retval         none
  */
static void chassis_body_feedforward_update(chassis_move_t *chassis_move_ff)
{
	float ax;
	float ay;
	float alpha;
	float wheel_force[CHASSIS_MODULE_NUM] = {0.0f};
	const float accel_limit = (CHASSIS_STOP_DECEL > CHASSIS_MAX_ACCEL) ? CHASSIS_STOP_DECEL : CHASSIS_MAX_ACCEL;

	if (chassis_move_ff == NULL) return;

	if (chassis_move_ff->body_ff_init == 0u)
	{
		chassis_move_ff->last_vx_plan_ff = chassis_move_ff->vx_plan;
		chassis_move_ff->last_vy_plan_ff = chassis_move_ff->vy_plan;
		chassis_move_ff->last_wz_plan_ff = chassis_move_ff->wz_plan;
		chassis_move_ff->body_ff_init = 1u;
		return;
	}

	ax = (chassis_move_ff->vx_plan - chassis_move_ff->last_vx_plan_ff) / CHASSIS_CONTROL_TIME;
	ay = (chassis_move_ff->vy_plan - chassis_move_ff->last_vy_plan_ff) / CHASSIS_CONTROL_TIME;
	alpha = (chassis_move_ff->wz_plan - chassis_move_ff->last_wz_plan_ff) / CHASSIS_CONTROL_TIME;
	ax = chassis_limit_abs(ax, accel_limit);
	ay = chassis_limit_abs(ay, accel_limit);
	alpha = chassis_limit_abs(alpha, CHASSIS_BODY_FF_YAW_ACCEL_LIMIT);

	chassis_move_ff->last_vx_plan_ff = chassis_move_ff->vx_plan;
	chassis_move_ff->last_vy_plan_ff = chassis_move_ff->vy_plan;
	chassis_move_ff->last_wz_plan_ff = chassis_move_ff->wz_plan;
	chassis_move_ff->vx_plan_accel = ax;
	chassis_move_ff->vy_plan_accel = ay;
	chassis_move_ff->wz_plan_accel = alpha;
	chassis_move_ff->body_ff_ax = ax;
	chassis_move_ff->body_ff_ay = ay;
	chassis_move_ff->body_ff_alpha = alpha;

	chassis_steer_force_distribution(chassis_move_ff, ax, ay, alpha, wheel_force);

	for (uint8_t i = 0U; i < CHASSIS_MODULE_NUM; i++)
	{
		chassis_move_ff->body_ff_current[i] = chassis_body_force_to_current_cmd(wheel_force[i]);
	}
}

/**
  * @brief          判断单轴速度制动补偿是否激活
  * @retval         1 表示激活，0 表示退出
  */
static uint8_t chassis_body_velocity_brake_axis_active(float target, float actual, float deadband)
{
	if (fabsf(actual) < deadband)
	{
		return 0u;
	}

	if (fabsf(target) < deadband)
	{
		return 1u;
	}

	if ((target * actual) < 0.0f)
	{
		return 1u;
	}

	return 0u;
}

/**
  * @brief          清除整车速度制动补偿状态
  * @retval         none
  */
static void chassis_body_velocity_brake_clear(chassis_move_t *chassis_move_brake)
{
	uint8_t i;

	if (chassis_move_brake == NULL) return;

	chassis_move_brake->body_vel_brake_error_vx = 0.0f;
	chassis_move_brake->body_vel_brake_error_vy = 0.0f;
	chassis_move_brake->body_vel_brake_error_wz = 0.0f;
	for (i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		chassis_move_brake->body_vel_brake_current[i] = 0.0f;
	}
}

/**
  * @brief          根据目标速度和正解车速更新制动补偿
  * @retval         none
  */
static void chassis_body_velocity_brake_update(chassis_move_t *chassis_move_brake)
{
#if (CHASSIS_BODY_VEL_BRAKE_ENABLE != 0U)
	float ax = 0.0f;
	float ay = 0.0f;
	float alpha = 0.0f;
	float wheel_force[CHASSIS_MODULE_NUM] = {0.0f};
	float xy_speed;
	float xy_kp;
	float xy_kp_ratio;
	uint8_t i;

	if (chassis_move_brake == NULL) return;

	chassis_body_velocity_brake_clear(chassis_move_brake);
	xy_speed = sqrtf(chassis_move_brake->vx * chassis_move_brake->vx +
	                 chassis_move_brake->vy * chassis_move_brake->vy);
	if (xy_speed <= CHASSIS_BODY_VEL_BRAKE_LOW_SPEED)
	{
		xy_kp = CHASSIS_BODY_VEL_BRAKE_XY_KP_LOW;
	}
	else if (xy_speed >= CHASSIS_BODY_VEL_BRAKE_HIGH_SPEED)
	{
		xy_kp = CHASSIS_BODY_VEL_BRAKE_XY_KP_HIGH;
	}
	else
	{
		xy_kp_ratio = (xy_speed - CHASSIS_BODY_VEL_BRAKE_LOW_SPEED) /
		              (CHASSIS_BODY_VEL_BRAKE_HIGH_SPEED - CHASSIS_BODY_VEL_BRAKE_LOW_SPEED);
		xy_kp = CHASSIS_BODY_VEL_BRAKE_XY_KP_LOW +
		        (CHASSIS_BODY_VEL_BRAKE_XY_KP_HIGH - CHASSIS_BODY_VEL_BRAKE_XY_KP_LOW) * xy_kp_ratio;
	}

	if (chassis_body_velocity_brake_axis_active(chassis_move_brake->vx_set,
	                                            chassis_move_brake->vx,
	                                            CHASSIS_BODY_VEL_BRAKE_SPEED_EPS) != 0u)
	{
		chassis_move_brake->body_vel_brake_error_vx =
			chassis_move_brake->vx_set - chassis_move_brake->vx;
		ax = chassis_limit_abs(chassis_move_brake->body_vel_brake_error_vx *
		                       xy_kp,
		                       CHASSIS_BODY_VEL_BRAKE_ACCEL_LIMIT);
	}

	if (chassis_body_velocity_brake_axis_active(chassis_move_brake->vy_set,
	                                            chassis_move_brake->vy,
	                                            CHASSIS_BODY_VEL_BRAKE_SPEED_EPS) != 0u)
	{
		chassis_move_brake->body_vel_brake_error_vy =
			chassis_move_brake->vy_set - chassis_move_brake->vy;
		ay = chassis_limit_abs(chassis_move_brake->body_vel_brake_error_vy *
		                       xy_kp,
		                       CHASSIS_BODY_VEL_BRAKE_ACCEL_LIMIT);
	}

	if ((chassis_move_brake->spin_exit_transition_active == 0U) &&
	    chassis_body_velocity_brake_axis_active(chassis_move_brake->wz_set,
                                             chassis_move_brake->wz,
                                             CHASSIS_BODY_VEL_BRAKE_WZ_EPS) != 0u)
	{
		chassis_move_brake->body_vel_brake_error_wz =
			chassis_move_brake->wz_set - chassis_move_brake->wz;
		alpha = chassis_limit_abs(chassis_move_brake->body_vel_brake_error_wz *
		                          CHASSIS_BODY_VEL_BRAKE_WZ_KP,
		                          CHASSIS_BODY_VEL_BRAKE_WZ_ACCEL_LIMIT);
	}

	chassis_steer_force_distribution(chassis_move_brake, ax, ay, alpha, wheel_force);

	for (i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		chassis_move_brake->body_vel_brake_current[i] =
			chassis_limit_abs(chassis_body_force_to_current_cmd(wheel_force[i]),
			                  CHASSIS_BODY_VEL_BRAKE_CURRENT_CMD_LIMIT);
		chassis_move_brake->body_ff_current[i] =
			chassis_limit_abs(chassis_move_brake->body_ff_current[i] +
			                  chassis_move_brake->body_vel_brake_current[i],
			                  CHASSIS_BODY_FF_MAX_CURRENT_CMD);
	}
#else
	chassis_body_velocity_brake_clear(chassis_move_brake);
#endif
}

/**
  * @brief          计算底盘航向角 PD 输出
  * @retval         PD 输出
  */
static float chassis_angle_pd_calc(pid_type_def *pd, float actual, float target, uint8_t wrap_enable, float target_vel, float actual_vel)
{
	float error;

	if (pd == NULL) return 0.0f;

	error = target - actual;
	if (wrap_enable != 0u)
	{
		error = rad_format(error);
	}
	if (fabsf(error) < CHASSIS_ANGLE_PD_DEADBAND)
	{
		error = 0.0f;
	}

	pd->set = target;
	pd->fdb = actual;
	pd->error[2] = pd->error[1];
	pd->error[1] = pd->error[0];
	pd->error[0] = error;
	pd->Pout = pd->Kp * error;
	pd->Iout = 0.0f;
	pd->Dout = pd->Kd * (target_vel - actual_vel);
	pd->out = pd->Pout + pd->Dout;
	pd->out = chassis_limit_abs(pd->out, pd->max_out);

	return pd->out;
}

/**
  * @brief          计算云台跟随模式底盘自转速度
  * @retval         自转速度给定
  */
static float chassis_follow_yaw_control(chassis_move_t *chassis_move_follow, float manual_wz)
{
	float pd_out;
	float wz_cmd;

	if (chassis_move_follow == NULL) return 0.0f;

	if ((chassis_hwt906_yaw_ready() == 0u) ||
	    (chassis_move_follow->chassis_yaw_motor == NULL) ||
	    (chassis_move_follow->chassis_yaw_motor->angle_offset_init == 0u))
	{
		PID_clear(&chassis_move_follow->chassis_angle_pid);
		chassis_wz_speed_pi_clear(chassis_move_follow);
		chassis_yaw_set_vel_filter_clear(chassis_move_follow);
		return 0.0f;
	}

	(void)manual_wz;
	/* 跟随使用云台实测航向，避免把云台目标轨迹延迟引入底盘环。 */
	chassis_move_follow->chassis_yaw_set =
		chassis_move_follow->chassis_yaw_motor->absolute_angle;
	chassis_move_follow->chassis_yaw_target =
		chassis_move_follow->chassis_yaw_set;
	chassis_move_follow->chassis_yaw_set_vel =
		CHASSIS_YAW_RATE_FEEDBACK_SIGN *
		chassis_move_follow->chassis_yaw_motor->gyro;
	first_order_filter_cali(&chassis_move_follow->chassis_yaw_set_vel_filter,
	                        chassis_move_follow->chassis_yaw_set_vel);
	chassis_move_follow->chassis_yaw_set_vel_filtered =
		chassis_move_follow->chassis_yaw_set_vel_filter.out;

	pd_out = chassis_angle_pd_calc(&chassis_move_follow->chassis_angle_pid,
	                               chassis_move_follow->chassis_yaw,
	                               chassis_move_follow->chassis_yaw_set,
	                               1u,
	                               chassis_move_follow->chassis_yaw_set_vel_filtered,
	                               chassis_move_follow->chassis_yaw_rate_filtered);
	wz_cmd = pd_out + chassis_wz_speed_pi_calc(chassis_move_follow,
	                                           pd_out,
	                                           chassis_move_follow->wz);
	wz_cmd = chassis_limit_abs(wz_cmd, CHASSIS_WZ_MAX_SPEED);
	if (fabsf(wz_cmd) < CHASSIS_WZ_OUTPUT_DEADBAND)
	{
		wz_cmd = 0.0f;
		chassis_wz_speed_pi_clear(chassis_move_follow);
	}
	return wz_cmd;
}

/**
  * @brief          计算底盘航向保持模式自转速度
  * @retval         自转速度给定
  */
static float chassis_yaw_hold_control(chassis_move_t *chassis_move_yaw_hold, float manual_wz)
{
	float yaw_rate_set;
	float pd_out;
	float wz_cmd;

	if (chassis_move_yaw_hold == NULL) return 0.0f;

	if (chassis_hwt906_yaw_ready() == 0u)
	{
		PID_clear(&chassis_move_yaw_hold->chassis_angle_pid);
		chassis_move_yaw_hold->chassis_yaw_target = chassis_move_yaw_hold->chassis_yaw;
		chassis_move_yaw_hold->chassis_yaw_set = chassis_move_yaw_hold->chassis_yaw;
		chassis_move_yaw_hold->chassis_yaw_set_vel = 0.0f;
		chassis_yaw_set_vel_filter_clear(chassis_move_yaw_hold);
		return 0.0f;
	}

	yaw_rate_set = -manual_wz * CHASSIS_YAW_HOLD_RC_SEN;
	yaw_rate_set = chassis_limit_wz_by_lateral_accel(chassis_move_yaw_hold->vx_plan,
	                                                 chassis_move_yaw_hold->vy_plan,
	                                                 yaw_rate_set);
	yaw_rate_set = fp32_constrain(yaw_rate_set, -CHASSIS_WZ_MAX_SPEED, CHASSIS_WZ_MAX_SPEED);

	chassis_move_yaw_hold->chassis_yaw_target += yaw_rate_set * CHASSIS_CONTROL_TIME;
	chassis_move_yaw_hold->chassis_yaw_set =
		chassis_move_yaw_hold->chassis_yaw_target;
	chassis_move_yaw_hold->chassis_yaw_set_vel = yaw_rate_set;
	first_order_filter_cali(&chassis_move_yaw_hold->chassis_yaw_set_vel_filter,
	                        chassis_move_yaw_hold->chassis_yaw_set_vel);
	chassis_move_yaw_hold->chassis_yaw_set_vel_filtered =
		chassis_move_yaw_hold->chassis_yaw_set_vel_filter.out;

	pd_out = chassis_angle_pd_calc(&chassis_move_yaw_hold->chassis_angle_pid,
	                               chassis_move_yaw_hold->chassis_yaw,
	                               chassis_move_yaw_hold->chassis_yaw_set,
	                               0u,
	                               chassis_move_yaw_hold->chassis_yaw_set_vel_filtered,
	                               chassis_move_yaw_hold->chassis_yaw_rate_filtered);
	wz_cmd = chassis_limit_abs(pd_out, CHASSIS_WZ_MAX_SPEED);
	if (fabsf(wz_cmd) < CHASSIS_WZ_OUTPUT_DEADBAND)
	{
		wz_cmd = 0.0f;
	}
	return wz_cmd;
}

/**
  * @brief          对数值执行对称限幅
  * @retval         限幅后的数值
  */
static float chassis_limit_abs(float value, float max_abs)
{
	if (value > max_abs)
	{
		return max_abs;
	}
	else if (value < -max_abs)
	{
		return -max_abs;
	}

	return value;
}

/**
  * @brief          清除整车 wz 速度 PI 积分
  * @retval         none
  */
static void chassis_wz_speed_pi_clear(chassis_move_t *chassis_move_clear)
{
	if (chassis_move_clear == NULL) return;

	chassis_move_clear->wz_speed_pi_iout = 0.0f;
}

static void chassis_yaw_set_vel_filter_clear(chassis_move_t *chassis_move_clear)
{
	if (chassis_move_clear == NULL) return;

	first_order_filter_init(&chassis_move_clear->chassis_yaw_set_vel_filter,
	                        CHASSIS_CONTROL_TIME,
	                        &chassis_yaw_set_vel_filter_tau);
	chassis_move_clear->chassis_yaw_set_vel_filtered = 0.0f;
}

/**
  * @brief          计算整车 wz 速度 PI 修正量
  * @retval         wz 速度修正量，单位 rad/s
  */
static float chassis_wz_speed_pi_calc(chassis_move_t *chassis_move_pi, float target_wz, float actual_wz)
{
	float error_wz;
	float iout_candidate;
	float out_raw;
	float out;

	if (chassis_move_pi == NULL) return 0.0f;

	error_wz = target_wz - actual_wz;
	iout_candidate = chassis_move_pi->wz_speed_pi_iout +
		CHASSIS_WZ_SPEED_PI_KI * error_wz * CHASSIS_CONTROL_TIME;
	iout_candidate = chassis_limit_abs(iout_candidate, CHASSIS_WZ_SPEED_PI_MAX_IOUT);
	out_raw = CHASSIS_WZ_SPEED_PI_KP * error_wz + iout_candidate;
	out = chassis_limit_abs(out_raw, CHASSIS_WZ_SPEED_PI_MAX_OUT);

	/* 修正量饱和且误差仍推动同向时冻结积分，误差反向时允许积分释放。 */
	if (!((out_raw > CHASSIS_WZ_SPEED_PI_MAX_OUT && error_wz > 0.0f) ||
	      (out_raw < -CHASSIS_WZ_SPEED_PI_MAX_OUT && error_wz < 0.0f)))
	{
		chassis_move_pi->wz_speed_pi_iout = iout_candidate;
	}

	return out;
}

/**
  * @brief          按速度、加速度和 jerk 约束更新 S 曲线规划
  * @retval         更新后的规划速度
  */
static float chassis_s_curve_update(float cmd, float *plan, float *accel, float max_speed, float max_accel, float max_jerk, float stop_accel, float stop_jerk)
{
	float accel_target;
	float accel_step;
	float next_plan;
	float target;
	float decel_limit;
	float jerk_limit;
	const float stop_epsilon = 0.0001f;
	uint8_t brake_to_zero;

	if (plan == NULL || accel == NULL) return 0.0f;

	target = chassis_limit_abs(cmd, max_speed);
	brake_to_zero = ((fabsf(target) < stop_epsilon) || ((*plan) * target < 0.0f)) ? 1u : 0u;
	if (brake_to_zero != 0u)
	{
		decel_limit = (stop_accel > 0.0f) ? stop_accel : max_accel;
		jerk_limit = (stop_jerk > 0.0f) ? stop_jerk : max_jerk;
		jerk_limit = fabsf(jerk_limit);

		if (fabsf(*plan) <= decel_limit * CHASSIS_CONTROL_TIME + stop_epsilon)
		{
			*plan = 0.0f;
			*accel = 0.0f;
			return 0.0f;
		}

		if ((*plan) * (*accel) > 0.0f)
		{
			*accel = 0.0f;
		}

		accel_target = (*plan > 0.0f) ? -decel_limit : decel_limit;
		accel_step = chassis_limit_abs(accel_target - *accel, jerk_limit * CHASSIS_CONTROL_TIME);
		*accel = chassis_limit_abs(*accel + accel_step, decel_limit);
		next_plan = *plan + *accel * CHASSIS_CONTROL_TIME;
		if ((*plan) * next_plan <= 0.0f)
		{
			next_plan = 0.0f;
			*accel = 0.0f;
		}

		*plan = chassis_limit_abs(next_plan, max_speed);
		return *plan;
	}

	accel_target = chassis_limit_abs((target - *plan) / CHASSIS_CONTROL_TIME, max_accel);
	accel_step = chassis_limit_abs(accel_target - *accel, max_jerk * CHASSIS_CONTROL_TIME);
	*accel = chassis_limit_abs(*accel + accel_step, max_accel);

	next_plan = *plan + *accel * CHASSIS_CONTROL_TIME;
	if (((target - *plan) * (target - next_plan) <= 0.0f) &&
	    (fabsf(target - *plan) <= fabsf(*accel * CHASSIS_CONTROL_TIME) + 0.0001f))
	{
		next_plan = target;
		*accel = 0.0f;
	}

	*plan = chassis_limit_abs(next_plan, max_speed);
	return *plan;
}

/**
  * @brief          计算单个底盘电机速度 PID 输出
  * @retval         速度环输出
  */
static float chassis_speed_pi_calc(chassis_move_t *chassis_move_pi, uint8_t motor_idx, float error_v)
{
	float error_d;
	float out;

	if (chassis_move_pi == NULL || motor_idx >= CHASSIS_MODULE_NUM) return 0.0f;

	chassis_move_pi->speed_pi_iout[motor_idx] += CHASSIS_SPEED_PI_KI * error_v * CHASSIS_CONTROL_TIME;
	chassis_move_pi->speed_pi_iout[motor_idx] = chassis_limit_abs(chassis_move_pi->speed_pi_iout[motor_idx],
	                                                              CHASSIS_SPEED_PI_MAX_IOUT);
	error_d = -(chassis_move_pi->chassis_3508[motor_idx].speed_filtered -
	            chassis_move_pi->speed_pid_last_error[motor_idx]) / CHASSIS_CONTROL_TIME;
	chassis_move_pi->speed_pid_last_error[motor_idx] = chassis_move_pi->chassis_3508[motor_idx].speed_filtered;
	out = CHASSIS_SPEED_PI_KP * error_v +
	      chassis_move_pi->speed_pi_iout[motor_idx] +
	      CHASSIS_SPEED_PI_KD * error_d;

	return chassis_limit_abs(out, CHASSIS_SPEED_PI_MAX_OUT);
}

/**
  * @brief          判断底盘是否处于零速目标
  * @retval         1 表示零速目标，0 表示存在运动目标
  */
static uint8_t chassis_zero_speed_set(const chassis_move_t *chassis_move_zero_check)
{
	if (chassis_move_zero_check == NULL) return 0u;

	if ((fabsf(chassis_move_zero_check->vx_set) < 0.0001f) &&
	    (fabsf(chassis_move_zero_check->vy_set) < 0.0001f) &&
	    (fabsf(chassis_move_zero_check->wz_set) < 0.0001f))
	{
		return 1u;
	}

	return 0u;
}

/**
  * @brief          计算单轮模型前馈和速度环合成输出
  * @retval         none
  */
static float Model_Based_Control(uint8_t motor_idx, float set_speed, float ref_speed)
{
	float error_v = set_speed - ref_speed;
	float accel_raw = 0.0f;
	float I_accel = 0.0f;
	float I_viscous = 0.0f;
	float I_coulomb = 0.0f;
	float I_static = 0.0f;
	float I_brake = 0.0f;
	float I_pi = 0.0f;
	float out = 0.0f;
	float friction_ff_scale = 1.0f;
	uint8_t brake_mode = 0u;
	uint8_t zero_speed_target;

	if (motor_idx >= CHASSIS_MODULE_NUM) return 0.0f;
	zero_speed_target = chassis_zero_speed_set(&chassis_move);
	if (zero_speed_target == 0u)
	{
		chassis_move.stop_brake_active[motor_idx] = 0u;
	}
	else if (chassis_move.stop_brake_active[motor_idx] == 2u)
	{
		return 0.0f;
	}

	if ((zero_speed_target != 0u) &&
	    (fabsf(set_speed) < 0.0001f) &&
	    (fabsf(ref_speed) < CHASSIS_BRAKE_RELEASE_SPEED_EPS))
	{
		chassis_move.model_accel[motor_idx] = 0.0f;
		chassis_move.model_last_speed_set[motor_idx] = 0.0f;
		chassis_move.speed_pi_iout[motor_idx] = 0.0f;
		chassis_move.speed_pid_last_error[motor_idx] = 0.0f;
		chassis_move.stop_brake_active[motor_idx] = 2u;
		return 0.0f;
	}

	accel_raw = (set_speed - chassis_move.model_last_speed_set[motor_idx]) / CHASSIS_CONTROL_TIME;
	if ((fabsf(ref_speed) >= CHASSIS_BRAKE_ENTER_SPEED_EPS) &&
	    (((accel_raw * ref_speed) < 0.0f) || (zero_speed_target != 0u)))
	{
		chassis_move.stop_brake_active[motor_idx] = 1u;
	}
	else if ((chassis_move.stop_brake_active[motor_idx] != 0u) &&
	         ((fabsf(set_speed) >= 0.0001f) || (fabsf(ref_speed) < CHASSIS_BRAKE_RELEASE_SPEED_EPS)))
	{
		chassis_move.stop_brake_active[motor_idx] = 0u;
	}
	brake_mode = chassis_move.stop_brake_active[motor_idx];

	if (brake_mode != 0u)
	{
		friction_ff_scale = CHASSIS_BRAKE_FRICTION_FF_SCALE;
	}

	chassis_move.model_last_speed_set[motor_idx] = set_speed;

	I_accel = chassis_move.body_ff_current[motor_idx];
	if (zero_speed_target == 0u)
	{
		I_viscous = friction_ff_scale * CHASSIS_FF_VISCOUS_GAIN * set_speed;
		I_coulomb = friction_ff_scale * CHASSIS_FF_COULOMB_CURRENT * tanhf(set_speed / CHASSIS_FF_COULOMB_SPEED_EPS);
		I_static = friction_ff_scale * CHASSIS_FF_STATIC_CURRENT * tanhf(set_speed / CHASSIS_FF_STATIC_SPEED_EPS);
	}
	if (brake_mode != 0u)
	{
		I_brake = -(CHASSIS_BRAKE_FF_CURRENT_A / CHASSIS_CURRENT_CMD_TO_A) *
		          tanhf(ref_speed / CHASSIS_BRAKE_FF_SPEED_EPS);
	}

	I_pi = chassis_speed_pi_calc(&chassis_move, motor_idx, error_v);
	out = I_accel + I_viscous + I_coulomb + I_static + I_brake + I_pi;

	if (out > M3505_MOTOR_SPEED_PID_MAX_OUT)
	{
		out = M3505_MOTOR_SPEED_PID_MAX_OUT;
	}
	else if (out < -M3505_MOTOR_SPEED_PID_MAX_OUT)
	{
		out = -M3505_MOTOR_SPEED_PID_MAX_OUT;
	}

	return out;
}

/**
  * @brief          计算底盘四轮速度环输出
  * @retval         none
  */
static void PID_Calc_Jump(chassis_move_t *chassis_pid_calc)
{
	uint8_t i;
	uint8_t set_speed_all_zero = 1u;
	static uint16_t zero_speed_i_clear_count = 0u;

	if (chassis_pid_calc == NULL) return;

	for (i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		if (chassis_pid_calc->chassis_mode == CHASSIS_VECTOR_NO_MOVE)
		{
			chassis_pid_calc->chassis_3508[i].speed_set = 0.0f;
		}
		if (fabsf(chassis_pid_calc->chassis_3508[i].speed_set) >= 0.0001f)
		{
			set_speed_all_zero = 0u;
		}
		if ((chassis_zero_speed_set(chassis_pid_calc) != 0u) &&
		    (fabsf(chassis_pid_calc->chassis_3508[i].speed_set) < 0.0001f) &&
		    (fabsf(chassis_pid_calc->chassis_3508[i].speed_filtered) < CHASSIS_BRAKE_RELEASE_SPEED_EPS))
		{
			chassis_pid_calc->model_accel[i] = 0.0f;
			chassis_pid_calc->model_last_speed_set[i] = 0.0f;
			chassis_pid_calc->speed_pid_last_error[i] = 0.0f;
		}
	}

	if ((chassis_zero_speed_set(chassis_pid_calc) != 0u) && (set_speed_all_zero != 0u))
	{
		if (zero_speed_i_clear_count < CHASSIS_ZERO_SPEED_I_CLEAR_CYCLES)
		{
			zero_speed_i_clear_count++;
		}
	}
	else
	{
		zero_speed_i_clear_count = 0u;
	}

	if (zero_speed_i_clear_count >= CHASSIS_ZERO_SPEED_I_CLEAR_CYCLES)
	{
		for (i = 0; i < CHASSIS_MODULE_NUM; i++)
		{
			chassis_pid_calc->speed_pi_iout[i] = 0.0f;
			chassis_pid_calc->speed_pid_last_error[i] = 0.0f;
			chassis_pid_calc->model_accel[i] = 0.0f;
			chassis_pid_calc->model_last_speed_set[i] = 0.0f;
		}
	}

	for (i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		chassis_pid_calc->model_3508_out[i] = Model_Based_Control(i, chassis_pid_calc->chassis_3508[i].speed_set, chassis_pid_calc->chassis_3508[i].speed_filtered);
	}
}

/* chassis dynamic current limit */
static void chassis_dynamic_current_limit_update(chassis_move_t *chassis_move_limit)
{
	float demand_a[CHASSIS_MODULE_NUM] = {0.0f};
	float total_demand_a = 0.0f;
	float max_step_a = CHASSIS_CURRENT_LIMIT_SLEW_A_PER_S * CHASSIS_CONTROL_TIME;
	uint8_t i;

	if (chassis_move_limit == NULL) return;

	for (i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		demand_a[i] = fabsf(chassis_move_limit->model_3508_out[i]) * CHASSIS_CURRENT_CMD_TO_A;
		total_demand_a += demand_a[i];
	}

	for (i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		float target_limit_a = CHASSIS_CURRENT_BASE_LIMIT_A;
		float limit_step_a;
		float limit_cmd;

		if (total_demand_a > CHASSIS_CURRENT_DEMAND_EPS_A)
		{
			target_limit_a += CHASSIS_CURRENT_DYNAMIC_POOL_A * demand_a[i] / total_demand_a;
		}

		target_limit_a = fp32_constrain(target_limit_a,
		                                CHASSIS_CURRENT_BASE_LIMIT_A,
		                                CHASSIS_CURRENT_SINGLE_MAX_A);
		limit_step_a = fp32_constrain(target_limit_a - chassis_move_limit->last_motor_current_limit_a[i],
		                               -max_step_a,
		                                max_step_a);
		chassis_move_limit->motor_current_limit_a[i] =
			chassis_move_limit->last_motor_current_limit_a[i] + limit_step_a;
		chassis_move_limit->last_motor_current_limit_a[i] =
			chassis_move_limit->motor_current_limit_a[i];

		limit_cmd = chassis_move_limit->motor_current_limit_a[i] / CHASSIS_CURRENT_CMD_TO_A;
		chassis_move_limit->model_3508_out[i] =
			fp32_constrain(chassis_move_limit->model_3508_out[i], -limit_cmd, limit_cmd);
	}
}

/**
  * @brief          底盘控制循环
  * @note           无力模式或在线门控失败时直接清控制状态并返回，正常零输入仍保留零速速度环。
  * @param[out]     chassis_move_control_loop: 底盘控制结构体指针
  * @retval         none
  */
__attribute__((used)) void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
	float wheel_speed[CHASSIS_MODULE_NUM] = {0.0f};
	float wheel_angle[CHASSIS_MODULE_NUM] = {0.0f};
	uint8_t i;
	uint8_t zero_speed_cmd;

	if (chassis_move_control_loop == NULL) return;

	if ((chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_NO_MOVE) ||
	    (chassis_control_online() == 0u))
	{
		/* 无力模式和离线保护只清状态并输出 0，不进入零速速度环。 */
		chassis_zero_force_clear(chassis_move_control_loop);
		return;
	}

	chassis_move_control_loop->last_vx_set = chassis_move_control_loop->vx_set;
	chassis_move_control_loop->last_vy_set = chassis_move_control_loop->vy_set;
	chassis_move_control_loop->last_wz_set = chassis_move_control_loop->wz_set;

	zero_speed_cmd = ((fabsf(chassis_move_control_loop->vx_set) < 0.0001f) &&
	                  (fabsf(chassis_move_control_loop->vy_set) < 0.0001f) &&
	                  (fabsf(chassis_move_control_loop->wz_set) < 0.0001f)) ? 1u : 0u;

	if ((zero_speed_cmd != 0u) &&
	    (chassis_move_control_loop->spin_exit_transition_active == 0U))
	{
		chassis_move_control_loop->vx_plan = 0.0f;
		chassis_move_control_loop->vy_plan = 0.0f;
		chassis_move_control_loop->wz_plan = 0.0f;
		chassis_move_control_loop->vx_plan_accel = 0.0f;
		chassis_move_control_loop->vy_plan_accel = 0.0f;
		chassis_move_control_loop->wz_plan_accel = 0.0f;
		chassis_body_feedforward_clear(chassis_move_control_loop);
	}
	else
	{
		chassis_move_control_loop->vx_plan = chassis_s_curve_update(chassis_move_control_loop->vx_set,
		                                                            &chassis_move_control_loop->vx_plan,
		                                                            &chassis_move_control_loop->vx_plan_accel,
		                                                            chassis_move_control_loop->vx_max_speed,
		                                                            CHASSIS_MAX_ACCEL,
		                                                            CHASSIS_MAX_JERK,
		                                                            CHASSIS_STOP_DECEL,
		                                                            CHASSIS_STOP_JERK);
		chassis_move_control_loop->vy_plan = chassis_s_curve_update(chassis_move_control_loop->vy_set,
		                                                            &chassis_move_control_loop->vy_plan,
		                                                            &chassis_move_control_loop->vy_plan_accel,
		                                                            chassis_move_control_loop->vy_max_speed,
		                                                            CHASSIS_MAX_ACCEL,
		                                                            CHASSIS_MAX_JERK,
		                                                            CHASSIS_STOP_DECEL,
		                                                            CHASSIS_STOP_JERK);
		if ((chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) ||
		    (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_YAW_HOLD))
		{
			if (chassis_move_control_loop->spin_exit_transition_active != 0U)
			{
				float last_wz_plan = chassis_move_control_loop->wz_plan;

				chassis_move_control_loop->wz_plan *=
					(1.0f - CHASSIS_SPIN_EXIT_WZ_FILTER_ALPHA);
				chassis_move_control_loop->wz_plan_accel =
					(chassis_move_control_loop->wz_plan - last_wz_plan) / CHASSIS_CONTROL_TIME;
				if (fabsf(chassis_move_control_loop->wz_plan) <= CHASSIS_SPIN_EXIT_WZ_EPS)
				{
					chassis_move_control_loop->wz_plan = 0.0f;
					chassis_move_control_loop->wz_plan_accel = 0.0f;
				}
				if ((fabsf(chassis_move_control_loop->wz_plan) <= CHASSIS_SPIN_EXIT_WZ_EPS) &&
				    (fabsf(chassis_move_control_loop->wz) <= CHASSIS_SPIN_EXIT_WZ_EPS))
				{
					chassis_move_control_loop->spin_exit_transition_active = 0U;
				}
			}
			else
			{
				chassis_move_control_loop->wz_plan =
					fp32_constrain(chassis_move_control_loop->wz_set, -CHASSIS_WZ_MAX_SPEED, CHASSIS_WZ_MAX_SPEED);
				chassis_move_control_loop->wz_plan_accel = 0.0f;
			}
		}
		else
		{
			chassis_move_control_loop->wz_plan = chassis_s_curve_update(chassis_move_control_loop->wz_set,
			                                                            &chassis_move_control_loop->wz_plan,
			                                                            &chassis_move_control_loop->wz_plan_accel,
			                                                            CHASSIS_WZ_MAX_SPEED,
			                                                            CHASSIS_WZ_MAX_ACCEL,
			                                                            CHASSIS_WZ_MAX_JERK,
			                                                            CHASSIS_WZ_MAX_ACCEL,
			                                                            CHASSIS_WZ_MAX_JERK);
		}
		if (chassis_move_control_loop->spin_exit_transition_active == 0U)
		{
			chassis_move_control_loop->wz_plan = chassis_limit_wz_by_lateral_accel(chassis_move_control_loop->vx_plan,
			                                                                       chassis_move_control_loop->vy_plan,
			                                                                       chassis_move_control_loop->wz_plan);
		}
		chassis_body_feedforward_update(chassis_move_control_loop);
	}
	chassis_body_velocity_brake_update(chassis_move_control_loop);

	chas_inv_cal(chassis_move_control_loop->vx_plan,
	             chassis_move_control_loop->vy_plan,
	             chassis_move_control_loop->wz_plan,
	             wheel_angle,
	             wheel_speed);

	for (i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		float angle_error = rad_format(wheel_angle[i] - chassis_move_control_loop->chassis_6020[i].angle);
		float steer_speed_set;

		chassis_move_control_loop->chassis_6020[i].angle_set = wheel_angle[i];
		chassis_move_control_loop->chassis_3508[i].speed_set = wheel_speed[i] *
			fp32_constrain(cosf(angle_error), 0.0f, 1.0f);
		steer_speed_set = PID_Calc(&chassis_move_control_loop->steer_angle_pid[i],
		                           chassis_move_control_loop->chassis_6020[i].angle,
		                           chassis_move_control_loop->chassis_6020[i].angle + angle_error);
		chassis_move_control_loop->chassis_6020[i].give_current = (int16_t)PID_Calc(
			&chassis_move_control_loop->steer_speed_pid[i],
			chassis_move_control_loop->chassis_6020[i].speed, steer_speed_set);
	}

	slip_control(chassis_move_control_loop);
	PID_Calc_Jump(chassis_move_control_loop);
	chassis_dynamic_current_limit_update(chassis_move_control_loop);
	for (i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		chassis_move_control_loop->chassis_3508[i].give_current = (int16_t)chassis_move_control_loop->model_3508_out[i];
	}
	chassis_power_control(chassis_move_control_loop);
	for (i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		float angle_error = rad_format(chassis_move_control_loop->chassis_6020[i].angle_set -
		                                 chassis_move_control_loop->chassis_6020[i].angle);
		chassis_move_control_loop->chassis_3508[i].give_current =
			(int16_t)((float)chassis_move_control_loop->chassis_3508[i].give_current *
			          fp32_constrain(cosf(angle_error), 0.0f, 1.0f));
	}
}

/**
  * @brief          发送底盘电流命令
  * @note           无力模式、DBUS 离线或任一底盘电机离线时发送 0 电流。
  * @param[in]      chassis_move_send: 底盘控制结构体指针
  * @retval         none
  */
__attribute__((used)) void chassis_send_cmd(chassis_move_t *chassis_move_send)
{
	if (chassis_move_send == NULL)
	{
		return;
	}

	if ((chassis_move_send->chassis_mode == CHASSIS_VECTOR_NO_MOVE) ||
	    (chassis_control_online() == 0u))
	{
		CAN_cmd_CHASSIS_ALL(0, 0, 0, 0);
		CAN_cmd_CHASSIS_STEER(0, 0, 0, 0);
		return;
	}

	CAN_cmd_CHASSIS_ALL(chassis_move_send->chassis_3508[0].give_current,
	                    chassis_move_send->chassis_3508[1].give_current,
	                    chassis_move_send->chassis_3508[2].give_current,
	                    chassis_move_send->chassis_3508[3].give_current);
	CAN_cmd_CHASSIS_STEER(chassis_move_send->chassis_6020[0].give_current,
	                       chassis_move_send->chassis_6020[1].give_current,
	                       chassis_move_send->chassis_6020[2].give_current,
	                       chassis_move_send->chassis_6020[3].give_current);
}
#endif
