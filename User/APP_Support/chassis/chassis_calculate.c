/**
  * @file       chassis_calculate.c
  * @brief      底盘运动学与限幅计算
  * @note       提供速度正解、逆解、坐标旋转和横向加速度限幅。
  */
#include "bsp_usart.h"
#include "chassis_calculate.h"
#include "chassis_task.h"
#include "bsp_fdcan.h"
#include "robot_param.h"
#include "user_lib.h"
#include <math.h>
#include "stdlib.h"

/**
  * @brief          初始化舵轮角度零偏
  * @retval         none
  */
void chassis_wheel_angle_offset_init(void)
{
	static const float offset_rad[CHASSIS_MODULE_NUM] = {
		CHASSIS_STEER_OFFSET_LF_RAD, CHASSIS_STEER_OFFSET_LB_RAD,
		CHASSIS_STEER_OFFSET_RB_RAD, CHASSIS_STEER_OFFSET_RF_RAD
	};
	for(uint8_t i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		chassis_move.wheel_angle_offset.now[i] = offset_rad[i];
		chassis_move.wheel_angle_offset.last[i] = offset_rad[i];
		chassis_move.wheel_angle_offset.initial[i] = offset_rad[i];
	}
}

/**
  * @brief          旋转二维向量
  * @retval         none
  */
void vector_rotate(float angle, float *vector)
{
	if(vector == NULL) return;

	float x_temp = vector[0];
	angle = rad_format(angle);

	float cos_value = cosf(angle);
	float sin_value = sinf(angle);

	vector[0] = cos_value * x_temp - sin_value * vector[1];
	vector[1] = sin_value * x_temp + cos_value * vector[1];
}

/**
  * @brief          限制底盘四轮速度比例
  * @retval         none
  */
static void limit_chassis_wheel_speed(float *wheel_speed)
{
	float max_speed = 0.0f;

	for(uint8_t i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		float speed_abs = fabsf(wheel_speed[i]);
		if(speed_abs > max_speed)
		{
			max_speed = speed_abs;
		}
	}

	if(max_speed > MAX_WHEEL_SPEED)
	{
		float scale = MAX_WHEEL_SPEED / max_speed;
		for(uint8_t i = 0; i < CHASSIS_MODULE_NUM; i++)
		{
			wheel_speed[i] *= scale;
		}
	}
}

float offset = 0.0f;

/**
  * @brief          执行底盘打滑补偿
  * @retval         none
  */
void slip_control(chassis_move_t *chassis_move)
{
	(void)chassis_move;
}

/**
  * @brief          按横向加速度限制底盘角速度
  * @retval         限幅后的角速度
  */
float chassis_limit_wz_by_lateral_accel(float vx, float vy, float wz)
{
	float translational_speed = sqrtf(vx * vx + vy * vy);
	float wz_limit;

	if(translational_speed < CHASSIS_LAT_SPEED_EPS)
	{
		return wz;
	}

	wz_limit = CHASSIS_LAT_ACCEL_LIMIT / translational_speed;
	return fp32_constrain(wz, -wz_limit, wz_limit);
}

/**
  * @brief          执行底盘逆运动学解算
  * @retval         none
  */
void chas_inv_cal(float vx_set, float vy_set, float wz_set, float *wheel_angle, float *wheel_speed)
{
	if((wheel_angle == NULL) || (wheel_speed == NULL)) return;
	static const float x_m[CHASSIS_MODULE_NUM] = {
		CHASSIS_STEER_HALF_LENGTH_M, -CHASSIS_STEER_HALF_LENGTH_M,
		-CHASSIS_STEER_HALF_LENGTH_M, CHASSIS_STEER_HALF_LENGTH_M
	};
	static const float y_m[CHASSIS_MODULE_NUM] = {
		CHASSIS_STEER_HALF_WIDTH_M, CHASSIS_STEER_HALF_WIDTH_M,
		-CHASSIS_STEER_HALF_WIDTH_M, -CHASSIS_STEER_HALF_WIDTH_M
	};

	for(uint8_t i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		float wheel_vx = vx_set - wz_set * y_m[i];
		float wheel_vy = vy_set + wz_set * x_m[i];
		float angle_error;

		wheel_speed[i] = sqrtf(wheel_vx * wheel_vx + wheel_vy * wheel_vy);
		if(wheel_speed[i] < CHASSIS_STEER_STOP_SPEED_MPS)
		{
			wheel_angle[i] = chassis_move.chassis_6020[i].angle_set;
			wheel_speed[i] = 0.0f;
			continue;
		}
		wheel_angle[i] = rad_format(atan2f(wheel_vy, wheel_vx));
		angle_error = rad_format(wheel_angle[i] - chassis_move.chassis_6020[i].angle);
		if(angle_error > PI * 0.5f)
		{
			wheel_angle[i] = rad_format(wheel_angle[i] - PI);
			wheel_speed[i] = -wheel_speed[i];
		}
		else if(angle_error < -PI * 0.5f)
		{
			wheel_angle[i] = rad_format(wheel_angle[i] + PI);
			wheel_speed[i] = -wheel_speed[i];
		}
	}

	limit_chassis_wheel_speed(wheel_speed);
}

/**
  * @brief          执行底盘正运动学解算
  * @retval         none
  */
void chas_for_cal(float *wheel_angle, float *wheel_speed, float *vx, float *vy, float *wz)
{
	static const float x_m[CHASSIS_MODULE_NUM] = {
		CHASSIS_STEER_HALF_LENGTH_M, -CHASSIS_STEER_HALF_LENGTH_M,
		-CHASSIS_STEER_HALF_LENGTH_M, CHASSIS_STEER_HALF_LENGTH_M
	};
	static const float y_m[CHASSIS_MODULE_NUM] = {
		CHASSIS_STEER_HALF_WIDTH_M, CHASSIS_STEER_HALF_WIDTH_M,
		-CHASSIS_STEER_HALF_WIDTH_M, -CHASSIS_STEER_HALF_WIDTH_M
	};
	float yaw_numerator = 0.0f;
	float yaw_denominator = 0.0f;

	if((wheel_angle == NULL) || (wheel_speed == NULL) || (vx == NULL) || (vy == NULL) || (wz == NULL)) return;
	*vx = 0.0f;
	*vy = 0.0f;
	for(uint8_t i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		float wheel_vx = wheel_speed[i] * cosf(wheel_angle[i]);
		float wheel_vy = wheel_speed[i] * sinf(wheel_angle[i]);
		*vx += wheel_vx;
		*vy += wheel_vy;
		yaw_numerator += -y_m[i] * wheel_vx + x_m[i] * wheel_vy;
		yaw_denominator += x_m[i] * x_m[i] + y_m[i] * y_m[i];
	}
	*vx /= (float)CHASSIS_MODULE_NUM;
	*vy /= (float)CHASSIS_MODULE_NUM;
	*wz = (yaw_denominator > 0.0f) ? yaw_numerator / yaw_denominator : 0.0f;
}
