/**
  * @file       chassis_behaviour.c
  * @brief      底盘行为模式设置
  * @note       行为层负责把遥控器、键鼠和联动状态转换为底盘控制模式与速度给定。
  */
#include "chassis_behaviour.h"
#include "chassis_power_control.h"
#include "chassis_task.h"
#include "cmsis_os.h"
#include "gimbal_behaviour.h"
#include "robot_param.h"
#include "stm32h7xx_hal.h"
#include <math.h>
#include <stdbool.h>

/**
  * @brief          无力模式速度给定清零
  * @retval         none
  */
static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          云台跟随模式速度给定
  * @retval         none
  */
static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          底盘航向保持模式速度给定
  * @retval         none
  */
static void chassis_yaw_hold_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          小陀螺模式速度给定
  * @retval         none
  */
static void chassis_spin_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

chassis_behaviour_e chassis_behaviour_mode = CHASSIS_NO_MOVE;
extern super_cap_mode_e super_cap_mode;

/**
  * @brief          设置底盘行为模式
  * @retval         none
  */
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
	if (chassis_move_mode == NULL) return;

	/* s0 mode state is provided by the gimbal behavior layer. */
	switch (gimbal_total_mode_get())
	{
	case GIMBAL_TOTAL_FOLLOW:
		chassis_behaviour_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
		break;
	case GIMBAL_TOTAL_SPIN:
		chassis_behaviour_mode = CHASSIS_SPIN;
		break;
	case GIMBAL_TOTAL_AUTO_SENTINEL:
		chassis_behaviour_mode = CHASSIS_NO_MOVE;
		break;
	case GIMBAL_TOTAL_MOTIONLESS:
	default:
		chassis_behaviour_mode = CHASSIS_NO_MOVE;
		break;
	}


	// 根据行为模式选择底盘控制模式。
	if (chassis_behaviour_mode == CHASSIS_NO_MOVE)	//闈欐妯″紡
	{
		chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_MOVE;	// 模式说明
	}
	else if (chassis_behaviour_mode == CHASSIS_FOLLOW_GIMBAL_YAW)	// 跟随云台模式
	{
		chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;	// 模式说明
	}
	else if (chassis_behaviour_mode == CHASSIS_YAW_HOLD)
	{
		chassis_move_mode->chassis_mode = CHASSIS_VECTOR_YAW_HOLD;	// 模式说明
	}
	else if(chassis_behaviour_mode == CHASSIS_SPIN)	// 小陀螺模式
	{
		chassis_move_mode->chassis_mode = CHASSIS_VECTOR_SPIN;	// 模式说明
	}
//	else if (chassis_behaviour_mode == CHASSIS_RETURN)
//	{
//	}

	{
// 预留底盘回正模式。
		static bool pressed_Q = false, last_pressed_Q = false;
		pressed_Q = (chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_Q);
		if(pressed_Q && !last_pressed_Q && super_cap_mode >= SUPER_CAP_PREPARED)
		{
			super_cap_mode = (super_cap_mode == SUPER_CAP_PREPARED) ? SUPER_CAP_USING : SUPER_CAP_PREPARED;
		}
		last_pressed_Q = pressed_Q;
	}
}

/**
  * @brief          根据底盘行为模式生成速度给定
  * @retval         none
  */
void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL) return;

	if (chassis_behaviour_mode != CHASSIS_SPIN)
	{
		chassis_move_rc_to_vector->spin_wave_active = 0U;
	}

	if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
	{
		chassis_no_move_control(vx_set, vy_set, wz_set, chassis_move_rc_to_vector);
	}
	else if (chassis_behaviour_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
	{
		chassis_infantry_follow_gimbal_yaw_control(vx_set, vy_set, wz_set, chassis_move_rc_to_vector);
	}
	else if (chassis_behaviour_mode == CHASSIS_YAW_HOLD)
	{
		chassis_yaw_hold_control(vx_set, vy_set, wz_set, chassis_move_rc_to_vector);
	}
	else if(chassis_behaviour_mode == CHASSIS_SPIN)
	{
		chassis_spin_control(vx_set, vy_set, wz_set, chassis_move_rc_to_vector);
	}
}

/**
  * @brief          无力模式速度给定清零
  * @retval         none
  */
static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL) return;
    *vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}

// 预留底盘回正模式。
fp32 chassis_wz_rc_sen = CHASSIS_WZ_RC_SEN;

/**
  * @brief          云台跟随模式速度给定
  * @retval         none
  */
static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL) return;

	// 根据遥控器通道和键盘输入生成平移速度给定。
	chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

	int16_t wz_channel = 0;
	rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL], wz_channel, CHASSIS_RC_DEADLINE);

	*wz_set = (fp32)wz_channel * chassis_wz_rc_sen;
}

/**
  * @brief          底盘航向保持模式速度给定
  * @retval         none
  */
static void chassis_yaw_hold_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL) return;

	chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

	int16_t wz_channel = 0;
	rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL], wz_channel, CHASSIS_RC_DEADLINE);

	*wz_set = (fp32)wz_channel;
}

/**
  * @brief          小陀螺模式速度给定
  * @retval         none
  */
static void chassis_spin_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
	uint32_t elapsed_ms;
	float phase;

	if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL) return;

	// 根据遥控器通道和键盘输入生成平移速度给定。
	chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

	// 首次进入小陀螺时记录正弦变速起点。
	if (!chassis_move_rc_to_vector->spin_wave_active)
	{
		chassis_move_rc_to_vector->spin_start_tick = HAL_GetTick();
		chassis_move_rc_to_vector->spin_wave_active = 1U;
	}

	elapsed_ms = HAL_GetTick() - chassis_move_rc_to_vector->spin_start_tick;
	phase = 2.0f * PI *
			((float)(elapsed_ms % CHASSIS_SPIN_PERIOD_MS) / (float)CHASSIS_SPIN_PERIOD_MS);
	*wz_set = CHASSIS_SPIN_MIN_SPEED +
			0.5f * (CHASSIS_SPIN_SPEED - CHASSIS_SPIN_MIN_SPEED) * (1.0f + sinf(phase));

	return;
}

/**
  * @brief          将遥控器输入转换为底盘平移速度
  * @retval         none
  */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL) return;

	int16_t vx_channel, vy_channel;
	fp32 vx_set_channel, vy_set_channel;
	fp32 slope_percentage = 0.30f;
	static uint8_t orientation_count[4] = {0};

	rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
	rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);
	vx_set_channel = vx_channel * (CHASSIS_VX_RC_SEN);
	vy_set_channel = vy_channel * (CHASSIS_VY_RC_SEN);

	if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
	{
		if (orientation_count[0] < 210){	orientation_count[0]++; }
		vx_set_channel = chassis_move_rc_to_vector->vx_max_speed * (slope_percentage + (((fp32)orientation_count[0]) / 300.0f));
	}
	else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
	{
		if (orientation_count[1] < 210){	orientation_count[1]++; }
		vx_set_channel = chassis_move_rc_to_vector->vx_min_speed * (slope_percentage + (((fp32)orientation_count[1]) / 300.0f));
	}

	if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
	{
		if (orientation_count[2] < 210){	orientation_count[2]++; }
		vy_set_channel = chassis_move_rc_to_vector->vy_max_speed * (slope_percentage + (((fp32)orientation_count[2]) / 300.0f));
	}
	else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
	{
		if (orientation_count[2] < 210){	orientation_count[3]++; }
		vy_set_channel = chassis_move_rc_to_vector->vy_min_speed * (slope_percentage + (((fp32)orientation_count[3]) / 300.0f));
	}

	if (!(chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)){ orientation_count[0] = 0; }
	if (!(chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY )){ orientation_count[1] = 0; }
	if (!(chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY )){ orientation_count[2] = 0; }
	if (!(chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)){ orientation_count[3] = 0; }

	// 娑撯偓闂冭埖鎶ゅ▔銏犻挬濠婃垿鈧喎瀹抽幐鍥︽姢
	first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
	first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);

	if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
	{
		chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
	}
	if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
	{
		chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
	}

	*vx_set =  chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
	*vy_set = -chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}
