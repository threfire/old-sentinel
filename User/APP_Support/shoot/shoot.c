/**
  * @file       shoot.c
  * @brief      3508 摩擦轮与 MIT 拨弹控制
  * @note       实现摩擦轮 ADRC 控制、掉速前馈、拨弹力矩控制和在线门控。
  */
#include "shoot.h"

#include <math.h>
#include <string.h>

#include "fdcan.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"

#if (ROBOT_FRICTION == ROBOT_FRICTION_3508)

extern motor_measure_t DJI_MOTOR_MEASURE[8];

shoot_task_control_t shoot_task_control;

static void shoot_task_init_control(shoot_task_control_t *control);
static void shoot_task_set_mode(shoot_task_control_t *control);
static void shoot_task_update_feedback(shoot_task_control_t *control);
static bool shoot_task_friction_online(void);
static bool shoot_task_strum_online(void);
static void shoot_task_control_friction(shoot_task_control_t *control);
static void shoot_task_stop_friction(shoot_task_control_t *control);
static void shoot_task_control_strum(shoot_task_control_t *control);
static void shoot_task_send_friction_current(int16_t fric1_current, int16_t fric2_current);
static void shoot_task_send_strum_torque(float torque_nm);
static void shoot_task_send_strum_mit(float pos, float vel, float kp, float kd, float torque_nm);
static void shoot_task_motor_init(shoot_task_motor_t *motor,
                                  const motor_measure_t *measure,
                                  float direction,
                                  float b0,
                                  float response_time_s,
                                  float observer_ratio,
                                  float output_rate_limit);
static void shoot_task_motor_reset(shoot_task_motor_t *motor);
static void shoot_task_motor_hot_reset(shoot_task_motor_t *motor);
static float shoot_task_motor_calc(shoot_task_motor_t *motor, float target_speed_rpm);
static float shoot_task_limit_current_a(float current_a);
static int16_t shoot_task_current_a_to_current_ma(float current_a);
static float shoot_task_current_ma_to_current_a(int16_t current_ma);
static int16_t shoot_task_current_ma_to_esc_cmd(int16_t current_ma);
static float shoot_task_feedback_cmd_to_current_a(int16_t current_cmd);
static float shoot_task_current_a_to_input_torque_nm(float current_a);
static void shoot_task_motor_update_current_physics(shoot_task_motor_t *motor);
static void shoot_task_motor_finalize_current(shoot_task_motor_t *motor);
static bool shoot_task_motor_ready(const shoot_task_motor_t *motor, uint32_t now);
static bool shoot_task_motor_should_trigger_feedforward(const shoot_task_motor_t *motor,
                                                        float trigger_drop_rpm,
                                                        float min_speed_ratio);
static void shoot_task_motor_apply_feedforward(shoot_task_motor_t *motor);
static void shoot_task_update_history(shoot_task_control_t *control);
static void shoot_task_update_bullet_speed_estimate(shoot_task_control_t *control);
static void shoot_task_update_fire_detect(shoot_task_control_t *control);
static void shoot_task_update_heat_model(shoot_task_control_t *control);
static bool shoot_task_should_start_bullet_speed_estimate(const shoot_task_control_t *control);
static void shoot_task_start_bullet_speed_estimate(shoot_task_control_t *control);
static bool shoot_task_should_start_fire_detect(const shoot_task_control_t *control, float *speed_drop_rpm);
static float shoot_task_get_fire_detect_current_a(const shoot_task_control_t *control);
static void shoot_task_set_fire_detected(shoot_task_control_t *control);
static bool shoot_task_fire_heat_would_over_limit(const shoot_task_control_t *control);
static uint16_t shoot_task_ms_to_ticks(uint16_t ms);
static float shoot_task_avg2(float a, float b);
static float shoot_task_max2(float a, float b);
static float shoot_task_min_float(float a, float b);
static float shoot_task_clamp_float(float value, float min_value, float max_value);

/**
  * @brief          初始化 3508 摩擦轮和 MIT 拨弹控制状态
  * @note           发射模块挂在云台任务内运行，初始化函数只建立状态，不做阻塞等待。
  * @retval         none
  */
__attribute__((used)) void shoot_init(void)
{
    shoot_task_init_control(&shoot_task_control);
}

/**
  * @brief          发射控制周期入口
  * @note           STOP 无力模式直接零输出；READY 模式按在线状态进入摩擦轮和拨弹控制。
  * @retval         none
  */
__attribute__((used)) void shoot_control_loop(void)
{
    shoot_task_set_mode(&shoot_task_control);
    shoot_task_update_feedback(&shoot_task_control);
    shoot_task_update_heat_model(&shoot_task_control);

    if (shoot_task_control.mode == SHOOT_TASK_READY_FRIC)
    {
        shoot_task_control_friction(&shoot_task_control);
    }
    else
    {
        shoot_task_stop_friction(&shoot_task_control);
    }

    shoot_task_control_strum(&shoot_task_control);

    shoot_task_control.last_mode = shoot_task_control.mode;
}


static void shoot_task_init_control(shoot_task_control_t *control)
{
    if (control == NULL)
    {
        return;
    }

    memset(control, 0, sizeof(*control));

    control->rc = get_remote_control_point();
    control->mode = SHOOT_TASK_STOP;
    control->last_mode = SHOOT_TASK_STOP;

    shoot_task_motor_init(&control->fric1,
                          &DJI_MOTOR_MEASURE[0],
                          SHOOT_FRIC1_DIRECTION,
                          SHOOT_FRIC1_B0,
                          SHOOT_FRIC1_RESPONSE_TIME_S,
                          SHOOT_FRIC1_OBSERVER_RATIO,
                          SHOOT_FRIC1_OUTPUT_RATE_LIMIT);
    shoot_task_motor_init(&control->fric2,
                          &DJI_MOTOR_MEASURE[1],
                          SHOOT_FRIC2_DIRECTION,
                          SHOOT_FRIC2_B0,
                          SHOOT_FRIC2_RESPONSE_TIME_S,
                          SHOOT_FRIC2_OBSERVER_RATIO,
                          SHOOT_FRIC2_OUTPUT_RATE_LIMIT);
    Motor_ENABLE(&hfdcan2, DM_STRUM_CAN_ID);
}

static void shoot_task_set_mode(shoot_task_control_t *control)
{

    if (control == NULL || control->rc == NULL)
    {
        return;
    }

    control->friction_enable = gimbal_friction_is_enabled();

    if (control->friction_enable)
    {

        control->mode = SHOOT_TASK_READY_FRIC;
    }
    else
    {
        control->mode = SHOOT_TASK_STOP;
    }

    control->friction_control_active = (control->mode == SHOOT_TASK_READY_FRIC);
}

static void shoot_task_update_feedback(shoot_task_control_t *control)
{
    if (control == NULL)
    {
        return;
    }

    if (control->fric1.measure != NULL)
    {

        control->fric1.speed_rpm = (float)control->fric1.measure->speed_rpm * control->fric1.direction;
        control->fric1.speed_mps = control->fric1.speed_rpm * SHOOT_FRIC_RPM_TO_MPS;
    }

    if (control->fric2.measure != NULL)
    {
        control->fric2.speed_rpm = (float)control->fric2.measure->speed_rpm * control->fric2.direction;
        control->fric2.speed_mps = control->fric2.speed_rpm * SHOOT_FRIC_RPM_TO_MPS;
    }

    shoot_task_motor_update_current_physics(&control->fric1);
    shoot_task_motor_update_current_physics(&control->fric2);
}

/**
  * @brief          摩擦轮在线状态
  * @retval         true: 两个摩擦轮均在线，false: 任一摩擦轮离线
  */
static bool shoot_task_friction_online(void)
{
    return (toe_is_error(FRIC1_MOTOR_TOE) == 0U) &&
           (toe_is_error(FRIC2_MOTOR_TOE) == 0U);
}

/**
  * @brief          拨弹 MIT 电机在线状态
  * @retval         true: 拨弹电机在线，false: 拨弹电机离线
  */
static bool shoot_task_strum_online(void)
{
    return (toe_is_error(PLUCK_MOTOR_TOE) == 0U);
}

/**
  * @brief          摩擦轮闭环控制
  * @note           两个摩擦轮均在线且 READY 模式有效时进入 ADRC 控制，离线时发 0 并清前馈状态。
  * @param[out]     control: 发射控制结构体指针
  * @retval         none
  */
static void shoot_task_control_friction(shoot_task_control_t *control)
{
    uint32_t now;
    bool friction_online;
    bool feedback_ready;

    if (control == NULL)
    {
        return;
    }

    now = HAL_GetTick();
    friction_online = shoot_task_friction_online();
    feedback_ready = shoot_task_motor_ready(&control->fric1, now) &&
                     shoot_task_motor_ready(&control->fric2, now);
    control->friction_online = friction_online;
    control->friction_feedback_ready = feedback_ready;
    if (!friction_online || !feedback_ready)
    {
        shoot_task_stop_friction(control);
        return;
    }

    if (control->last_mode != SHOOT_TASK_READY_FRIC)
    {

        shoot_task_motor_hot_reset(&control->fric1);
        shoot_task_motor_hot_reset(&control->fric2);
        control->fric1.ff_ticks = 0U;
        control->fric1.ff_cooldown_ticks = 0U;
        control->fric1.ff_current = 0;
        control->fric2.ff_ticks = 0U;
        control->fric2.ff_cooldown_ticks = 0U;
        control->fric2.ff_current = 0;
        shoot_task_update_history(control);
    }

    control->fric1.speed_set_rpm = SHOOT_FRIC_TARGET_SPEED_RPM;
    control->fric2.speed_set_rpm = SHOOT_FRIC_TARGET_SPEED_RPM;

    control->fric1.give_current_a = shoot_task_motor_calc(&control->fric1, control->fric1.speed_set_rpm);
    control->fric2.give_current_a = shoot_task_motor_calc(&control->fric2, control->fric2.speed_set_rpm);

    if ((control->fric1.ff_cooldown_ticks == 0U) &&
        shoot_task_motor_should_trigger_feedforward(&control->fric1,
                                                    SHOOT_FRIC1_FF_TRIGGER_DROP_RPM,
                                                    SHOOT_FRIC1_FF_MIN_SPEED_RATIO))
    {
        control->fric1.ff_current = SHOOT_FRIC1_FF_CURRENT;
        control->fric1.ff_ticks = SHOOT_FRIC1_FF_DURATION_MS;
        control->fric1.ff_cooldown_ticks = SHOOT_FRIC1_FF_COOLDOWN_MS;
    }

    if ((control->fric2.ff_cooldown_ticks == 0U) &&
        shoot_task_motor_should_trigger_feedforward(&control->fric2,
                                                    SHOOT_FRIC2_FF_TRIGGER_DROP_RPM,
                                                    SHOOT_FRIC2_FF_MIN_SPEED_RATIO))
    {
        control->fric2.ff_current = SHOOT_FRIC2_FF_CURRENT;
        control->fric2.ff_ticks = SHOOT_FRIC2_FF_DURATION_MS;
        control->fric2.ff_cooldown_ticks = SHOOT_FRIC2_FF_COOLDOWN_MS;
    }

    if (control->fric1.ff_ticks > 0U)
    {
        shoot_task_motor_apply_feedforward(&control->fric1);
        control->fric1.ff_ticks--;
    }
    if (control->fric2.ff_ticks > 0U)
    {
        shoot_task_motor_apply_feedforward(&control->fric2);
        control->fric2.ff_ticks--;
    }
    if (control->fric1.ff_cooldown_ticks > 0U)
    {
        control->fric1.ff_cooldown_ticks--;
    }
    if (control->fric2.ff_cooldown_ticks > 0U)
    {
        control->fric2.ff_cooldown_ticks--;
    }
    shoot_task_motor_finalize_current(&control->fric1);
    shoot_task_motor_finalize_current(&control->fric2);

    shoot_task_motor_update_current_physics(&control->fric1);
    shoot_task_motor_update_current_physics(&control->fric2);

    shoot_task_send_friction_current(control->fric1.give_current,
                                     control->fric2.give_current);
    shoot_task_update_bullet_speed_estimate(control);
    shoot_task_update_fire_detect(control);
    shoot_task_update_history(control);
}

/**
  * @brief          摩擦轮零输出
  * @note           STOP 无力模式调用该函数，清目标、清前馈、清开火检测并发送 0 电流。
  * @param[out]     control: 发射控制结构体指针
  * @retval         none
  */
static void shoot_task_stop_friction(shoot_task_control_t *control)
{
    if (control == NULL)
    {
        return;
    }

    control->fric1.speed_set_rpm = 0.0f;
    control->fric2.speed_set_rpm = 0.0f;
    control->fric1.give_current = 0;
    control->fric2.give_current = 0;
    control->fric1.give_current_a = 0.0f;
    control->fric2.give_current_a = 0.0f;
    shoot_task_motor_update_current_physics(&control->fric1);
    shoot_task_motor_update_current_physics(&control->fric2);
    control->fric1.ff_ticks = 0U;
    control->fric1.ff_cooldown_ticks = 0U;
    control->fric1.ff_current = 0;
    control->fric2.ff_ticks = 0U;
    control->fric2.ff_cooldown_ticks = 0U;
    control->fric2.ff_current = 0;
    control->friction_feedback_ready = false;
    control->bullet_speed_est_active = false;
    control->bullet_speed_est_ticks = 0U;
    control->fire_detect_active = false;
    control->fire_detected = false;
    control->fire_detect_latched = false;
    control->fire_detect_ticks = 0U;
    control->fire_detect_latch_ticks = 0U;
    control->fire_detect_speed_drop_rpm = 0.0f;
    control->fire_detect_current_a = 0.0f;

    if (control->last_mode != SHOOT_TASK_STOP)
    {

        shoot_task_motor_reset(&control->fric1);
        shoot_task_motor_reset(&control->fric2);
    }

    shoot_task_send_friction_current(0, 0);
}

/**
  * @brief          拨弹控制
  * @note           STOP 无力模式和拨弹离线时发送 0 力矩；摩擦轮就绪后允许速度控制。
  * @param[out]     control: 发射控制结构体指针
  * @retval         none
  */
static void shoot_task_control_strum(shoot_task_control_t *control)
{
    MITMeasure_t *strum_measure;
    uint32_t now;
    bool feedback_ready;
    bool heat_blocked;
    bool press_l;
    bool strum_ready;

    if (control == NULL || control->rc == NULL)
    {
        return;
    }

    press_l = (control->rc->mouse.press_l != 0U) || gimbal_strum_is_enabled();
    heat_blocked = shoot_task_fire_heat_would_over_limit(control);
    now = HAL_GetTick();
    strum_ready = (control->mode == SHOOT_TASK_READY_FRIC) &&
                  shoot_task_friction_online() &&
                  shoot_task_motor_ready(&control->fric1, now) &&
                  shoot_task_motor_ready(&control->fric2, now);
    control->heat_limit_active = heat_blocked;
    strum_measure = &MIT_MOTOR_MEASURE[SHOOT_STRUM_MIT_INDEX];
    feedback_ready = shoot_task_strum_online() &&
                     (strum_measure->fdb.last_fdb_time != 0U) &&
                     ((now - strum_measure->fdb.last_fdb_time) <= SHOOT_STRUM_FDB_TIMEOUT);
    if (!strum_ready || !feedback_ready)
    {
        strum_measure->set.POS = 0.0f;
        strum_measure->set.VEL = 0.0f;
        strum_measure->set.KP = 0.0f;
        strum_measure->set.KD = 0.0f;
        strum_measure->set.TOR = 0.0f;
        /* STOP 无力模式和拨弹离线保护只发送 0 力矩，不进入拨弹闭环。 */
        shoot_task_send_strum_torque(0.0f);
        return;
    }

    /* 保持 18 Hz 弹频：12 发/圈对应 1.5 圈/s，即 3*pi rad/s。
     * 位置目标固定为 0，避免反馈角跨过 ±pi 时引入位置跳变。 */
    if (press_l)
    {
        strum_measure->set.POS = 0.0f;
        strum_measure->set.VEL = SHOOT_STRUM_DIRECTION * SHOOT_STRUM_SPEED_RAD_S;
        strum_measure->set.KP = SHOOT_STRUM_SPEED_KP;
        strum_measure->set.KD = SHOOT_STRUM_SPEED_KD;
        strum_measure->set.TOR = SHOOT_STRUM_DIRECTION * SHOOT_STRUM_SPEED_TORQUE_FF_NM;
        shoot_task_send_strum_mit(strum_measure->set.POS,
                                  strum_measure->set.VEL,
                                  strum_measure->set.KP,
                                  strum_measure->set.KD,
                                  strum_measure->set.TOR);
        return;
    }

    strum_measure->set.POS = strum_measure->fdb.pos;
    strum_measure->set.VEL = 0.0f;
    strum_measure->set.KP = 0.0f;
    strum_measure->set.KD = 0.0f;
    strum_measure->set.TOR = 0.0f;
    shoot_task_send_strum_mit(strum_measure->set.POS, 0.0f, 0.0f, 0.0f, 0.0f);
}

/**
  * @brief          发送两路摩擦轮电流
  * @param[in]      fric1_current: 摩擦轮 1 电流，单位 mA
  * @param[in]      fric2_current: 摩擦轮 2 电流，单位 mA
  * @retval         none
  */
static void shoot_task_send_friction_current(int16_t fric1_current, int16_t fric2_current)
{
    uint8_t data[8];
    int16_t fric1_cmd = shoot_task_current_ma_to_esc_cmd(fric1_current);
    int16_t fric2_cmd = shoot_task_current_ma_to_esc_cmd(fric2_current);

    data[0] = (uint8_t)((uint16_t)fric1_cmd >> 8);
    data[1] = (uint8_t)fric1_cmd;
    data[2] = (uint8_t)((uint16_t)fric2_cmd >> 8);
    data[3] = (uint8_t)fric2_cmd;
    data[4] = 0U;
    data[5] = 0U;
    data[6] = 0U;
    data[7] = 0U;

    shoot_task_control.friction_can_cmd[0] = fric1_cmd;
    shoot_task_control.friction_can_cmd[1] = fric2_cmd;
    shoot_task_control.friction_can_send_status =
        canx_send_data(&hfdcan3, SHOOT_FRICTION_CMD_ID, data, 8U);
}

/**
  * @brief          发送拨弹 MIT 力矩命令
  * @param[in]      torque_nm: 拨弹输入力矩命令，单位 N*m
  * @retval         none
  */
static void shoot_task_send_strum_torque(float torque_nm)
{
    torque_nm = shoot_task_clamp_float(torque_nm, DM2325_T_MIN, DM2325_T_MAX);
    CAN_cmd_MIT(&hfdcan2, DM_STRUM_CAN_ID, 0.0f, 0.0f, 0.0f, 0.0f, torque_nm);
}

static void shoot_task_send_strum_mit(float pos, float vel, float kp, float kd, float torque_nm)
{
    CAN_cmd_MIT(&hfdcan2, DM_STRUM_CAN_ID, pos, vel, kp, kd, torque_nm);
}

static void shoot_task_motor_init(shoot_task_motor_t *motor,
                                  const motor_measure_t *measure,
                                  float direction,
                                  float b0,
                                  float response_time_s,
                                  float observer_ratio,
                                  float output_rate_limit)
{
    adrc_param_t param;

    if (motor == NULL)
    {
        return;
    }

    memset(motor, 0, sizeof(*motor));
    motor->measure = measure;
    motor->direction = direction;

    param.sample_time_s = (float)SHOOT_CONTROL_TIME * 0.001f;
    param.b0 = b0;
    param.controller_bandwidth = 5.0f / response_time_s;
    param.observer_bandwidth_ratio = observer_ratio;
    param.tracking_gain = 0.0f;
    param.max_out = SHOOT_FRIC_MAX_CURRENT;
    param.output_rate_limit = output_rate_limit;
    param.error_linear_zone = SHOOT_FRIC_ERROR_LINEAR_ZONE;
    param.alpha1 = SHOOT_FRIC_ALPHA1;
    param.alpha2 = SHOOT_FRIC_ALPHA2;

    ADRC_init(&motor->speed_adrc, &param);
    ADRC_reset(&motor->speed_adrc, 0.0f, 0.0f);
}

static void shoot_task_motor_reset(shoot_task_motor_t *motor)
{
    if (motor == NULL)
    {
        return;
    }

    ADRC_reset(&motor->speed_adrc, motor->speed_rpm, 0.0f);
}

static void shoot_task_motor_hot_reset(shoot_task_motor_t *motor)
{
    if (motor == NULL)
    {
        return;
    }

    ADRC_hot_reset(&motor->speed_adrc,
                   motor->speed_rpm,
                   SHOOT_FRIC_TARGET_SPEED_RPM,
                   motor->give_current_a * motor->direction);
}

static float shoot_task_limit_current_a(float current_a)
{
    if (current_a > SHOOT_FRIC_MAX_CURRENT)
    {
        current_a = SHOOT_FRIC_MAX_CURRENT;
    }
    else if (current_a < -SHOOT_FRIC_MAX_CURRENT)
    {
        current_a = -SHOOT_FRIC_MAX_CURRENT;
    }

    return current_a;
}

static int16_t shoot_task_current_a_to_current_ma(float current_a)
{
    float current_ma;

    current_ma = current_a * SHOOT_FRIC_MA_PER_A;
    return (int16_t)((current_ma >= 0.0f) ? (current_ma + 0.5f) : (current_ma - 0.5f));
}

static float shoot_task_current_ma_to_current_a(int16_t current_ma)
{
    return (float)current_ma / SHOOT_FRIC_MA_PER_A;
}

static int16_t shoot_task_current_ma_to_esc_cmd(int16_t current_ma)
{
    const float current_a = shoot_task_current_ma_to_current_a(current_ma);
    const float current_cmd = (current_a / SHOOT_FRIC_CURRENT_FULL_SCALE_A) *
                              SHOOT_FRIC_CURRENT_CMD_FULL_SCALE;

    return (int16_t)((current_cmd >= 0.0f) ? (current_cmd + 0.5f) : (current_cmd - 0.5f));
}

static float shoot_task_feedback_cmd_to_current_a(int16_t current_cmd)
{
    return ((float)current_cmd / SHOOT_FRIC_CURRENT_CMD_FULL_SCALE) * SHOOT_FRIC_CURRENT_FULL_SCALE_A;
}

static float shoot_task_current_a_to_input_torque_nm(float current_a)
{
    const float output_torque_nm = current_a * SHOOT_FRIC_OUTPUT_TORQUE_CONSTANT_NM_PER_A;

    return output_torque_nm / SHOOT_FRIC_REDUCTION_RATIO;
}

static void shoot_task_motor_update_current_physics(shoot_task_motor_t *motor)
{
    int16_t given_current = 0;

    if (motor == NULL)
    {
        return;
    }

    if (motor->measure != NULL)
    {
        given_current = motor->measure->given_current;
    }

    motor->given_current = given_current;
    motor->given_current_a = shoot_task_feedback_cmd_to_current_a(given_current);
    motor->give_input_torque_nm = shoot_task_current_a_to_input_torque_nm(motor->give_current_a);
    motor->given_input_torque_nm = shoot_task_current_a_to_input_torque_nm(motor->given_current_a);
}

static void shoot_task_motor_finalize_current(shoot_task_motor_t *motor)
{
    if (motor == NULL)
    {
        return;
    }

    motor->give_current_a = shoot_task_limit_current_a(motor->give_current_a);
    motor->give_current = shoot_task_current_a_to_current_ma(motor->give_current_a);
}

static float shoot_task_motor_calc(shoot_task_motor_t *motor, float target_speed_rpm)
{
    float current_output_a;

    if (motor == NULL)
    {
        return 0;
    }

    current_output_a = ADRC_Calc(&motor->speed_adrc, motor->speed_rpm, target_speed_rpm);
    current_output_a *= motor->direction;

    return current_output_a;
}

static bool shoot_task_motor_should_trigger_feedforward(const shoot_task_motor_t *motor,
                                                        float trigger_drop_rpm,
                                                        float min_speed_ratio)
{
#if (SHOOT_FRIC_FF_ENABLE == 0)
    (void)motor;
    (void)trigger_drop_rpm;
    (void)min_speed_ratio;
    return false;
#else
    float min_speed;
    float speed_drop;

    if (motor == NULL)
    {
        return false;
    }

    min_speed = SHOOT_FRIC_TARGET_SPEED_RPM * min_speed_ratio;
    speed_drop = motor->prev_speed_rpm - motor->speed_rpm;

    return ((motor->prev_speed_rpm >= min_speed) &&
            (speed_drop >= trigger_drop_rpm));
#endif
}

static void shoot_task_motor_apply_feedforward(shoot_task_motor_t *motor)
{
#if (SHOOT_FRIC_FF_ENABLE != 0)
    if (motor == NULL)
    {
        return;
    }

    {
        motor->give_current_a += (float)motor->ff_current * motor->direction;
        motor->give_current_a = shoot_task_limit_current_a(motor->give_current_a);
    }
#else
    (void)motor;
#endif
}

static void shoot_task_update_history(shoot_task_control_t *control)
{
    if (control == NULL)
    {
        return;
    }

    control->fric1.prev_speed_rpm = control->fric1.last_speed_rpm;
    control->fric2.prev_speed_rpm = control->fric2.last_speed_rpm;
    control->fric1.last_speed_rpm = control->fric1.speed_rpm;
    control->fric2.last_speed_rpm = control->fric2.speed_rpm;
}

static void shoot_task_update_bullet_speed_estimate(shoot_task_control_t *control)
{
    if (control == NULL)
    {
        return;
    }

    if (!control->bullet_speed_est_active &&
        shoot_task_should_start_bullet_speed_estimate(control))
    {
        shoot_task_start_bullet_speed_estimate(control);
    }

    if (!control->bullet_speed_est_active)
    {
        return;
    }

    control->bullet_speed_min_fric1_rpm =
        shoot_task_min_float(control->bullet_speed_min_fric1_rpm,
                             control->fric1.speed_rpm);
    control->bullet_speed_min_fric2_rpm =
        shoot_task_min_float(control->bullet_speed_min_fric2_rpm,
                             control->fric2.speed_rpm);
    if (control->bullet_speed_est_ticks > 0U)
    {
        control->bullet_speed_est_ticks--;
    }

    if (control->bullet_speed_est_ticks == 0U)
    {
        float speed_drop_rpm;

        control->bullet_speed_min_avg_rpm =
            shoot_task_avg2(control->bullet_speed_min_fric1_rpm,
                            control->bullet_speed_min_fric2_rpm);
        speed_drop_rpm =
            control->bullet_speed_start_avg_rpm -
            control->bullet_speed_min_avg_rpm;
        if (speed_drop_rpm < 0.0f)
        {
            speed_drop_rpm = 0.0f;
        }

        control->estimated_bullet_speed_mps =
            speed_drop_rpm * SHOOT_BULLET_SPEED_EST_COEFF_MPS_PER_RPM;
        control->bullet_speed_est_active = false;
    }
}

static bool shoot_task_should_start_bullet_speed_estimate(const shoot_task_control_t *control)
{
    float stable_speed_avg_rpm;

    if (control == NULL)
    {
        return false;
    }

    stable_speed_avg_rpm =
        shoot_task_avg2(control->fric1.last_speed_rpm,
                        control->fric2.last_speed_rpm);
    if (stable_speed_avg_rpm <
        SHOOT_FRIC_TARGET_SPEED_RPM * SHOOT_BULLET_SPEED_EST_MIN_SPEED_RATIO)
    {
        return false;
    }

    return (((control->fric1.last_speed_rpm - control->fric1.speed_rpm) >=
             SHOOT_BULLET_SPEED_EST_TRIGGER_DROP_RPM) ||
            ((control->fric2.last_speed_rpm - control->fric2.speed_rpm) >=
             SHOOT_BULLET_SPEED_EST_TRIGGER_DROP_RPM));
}

static void shoot_task_start_bullet_speed_estimate(shoot_task_control_t *control)
{
    if (control == NULL)
    {
        return;
    }

    control->bullet_speed_est_active = true;
    control->bullet_speed_est_ticks =
        shoot_task_ms_to_ticks(SHOOT_BULLET_SPEED_EST_WINDOW_MS);
    control->bullet_speed_start_avg_rpm =
        shoot_task_avg2(control->fric1.last_speed_rpm,
                        control->fric2.last_speed_rpm);
    control->bullet_speed_min_fric1_rpm = control->fric1.speed_rpm;
    control->bullet_speed_min_fric2_rpm = control->fric2.speed_rpm;
    control->bullet_speed_min_avg_rpm =
        shoot_task_avg2(control->bullet_speed_min_fric1_rpm,
                        control->bullet_speed_min_fric2_rpm);
}

static void shoot_task_update_fire_detect(shoot_task_control_t *control)
{
    float speed_drop_rpm = 0.0f;

    if (control == NULL)
    {
        return;
    }

    control->fire_detected = false;

    if (control->fire_detect_latch_ticks > 0U)
    {
        control->fire_detect_latch_ticks--;
        if (control->fire_detect_latch_ticks == 0U)
        {
            control->fire_detect_latched = false;
        }
    }

    if (!control->fire_detect_latched &&
        !control->fire_detect_active &&
        shoot_task_should_start_fire_detect(control, &speed_drop_rpm))
    {
        control->fire_detect_active = true;
        control->fire_detect_ticks =
            shoot_task_ms_to_ticks(SHOOT_FIRE_DETECT_WINDOW_MS);
        control->fire_detect_speed_drop_rpm = speed_drop_rpm;
        control->fire_detect_current_a = 0.0f;
    }

    if (!control->fire_detect_active)
    {
        return;
    }

    control->fire_detect_current_a =
        shoot_task_max2(control->fire_detect_current_a,
                        shoot_task_get_fire_detect_current_a(control));

    if (control->fire_detect_current_a >= SHOOT_FIRE_DETECT_CURRENT_A)
    {
        shoot_task_set_fire_detected(control);
        return;
    }

    if (control->fire_detect_ticks > 0U)
    {
        control->fire_detect_ticks--;
    }

    if (control->fire_detect_ticks == 0U)
    {
        control->fire_detect_active = false;
    }
}

/**
  * @brief          更新发射热量自然冷却和禁发状态
  * @note           热量每 50 ms 下降 1，对应每秒下降 20；预测下一发达到上限时置位禁发。
  * @retval         none
  */
static void shoot_task_update_heat_model(shoot_task_control_t *control)
{
    uint16_t decay_ticks;

    if (control == NULL)
    {
        return;
    }

    decay_ticks = shoot_task_ms_to_ticks(SHOOT_HEAT_DECAY_INTERVAL_MS);
    if (control->heat > 0U)
    {
        if (control->heat_cool_ticks < decay_ticks)
        {
            control->heat_cool_ticks++;
        }
        else
        {
            control->heat--;
            control->heat_cool_ticks = 0U;
        }
    }
    else
    {
        control->heat_cool_ticks = 0U;
    }

    control->heat_limit_active = shoot_task_fire_heat_would_over_limit(control);
}

static uint16_t shoot_task_ms_to_ticks(uint16_t ms)
{
    uint16_t ticks;

    ticks = (uint16_t)((ms + SHOOT_CONTROL_TIME - 1U) / SHOOT_CONTROL_TIME);
    if (ticks == 0U)
    {
        ticks = 1U;
    }

    return ticks;
}

static float shoot_task_avg2(float a, float b)
{
    return (a + b) / 2.0f;
}

static float shoot_task_max2(float a, float b)
{
    return (a > b) ? a : b;
}

static bool shoot_task_should_start_fire_detect(const shoot_task_control_t *control, float *speed_drop_rpm)
{
    float stable_speed_avg_rpm;
    float max_speed_drop_rpm;

    if (control == NULL)
    {
        return false;
    }

    stable_speed_avg_rpm =
        shoot_task_avg2(control->fric1.last_speed_rpm,
                        control->fric2.last_speed_rpm);
    if (stable_speed_avg_rpm <
        SHOOT_FRIC_TARGET_SPEED_RPM * SHOOT_FIRE_DETECT_MIN_SPEED_RATIO)
    {
        return false;
    }

    max_speed_drop_rpm =
        shoot_task_max2(control->fric1.last_speed_rpm - control->fric1.speed_rpm,
                        control->fric2.last_speed_rpm - control->fric2.speed_rpm);
    if (speed_drop_rpm != NULL)
    {
        *speed_drop_rpm = max_speed_drop_rpm;
    }

    return (max_speed_drop_rpm >= SHOOT_FIRE_DETECT_TRIGGER_DROP_RPM);
}

static float shoot_task_get_fire_detect_current_a(const shoot_task_control_t *control)
{
    if (control == NULL)
    {
        return 0.0f;
    }

    return shoot_task_max2(fabsf(control->fric1.given_current_a),
                           fabsf(control->fric2.given_current_a));
}

static void shoot_task_set_fire_detected(shoot_task_control_t *control)
{
    if (control == NULL)
    {
        return;
    }

    control->fire_detected = true;
    control->fire_detect_latched = true;
    control->fire_detect_latch_ticks =
        shoot_task_ms_to_ticks(SHOOT_FIRE_DETECT_LATCH_MS);
    control->fired_bullet_count++;
    if ((uint32_t)control->heat + SHOOT_HEAT_PER_BULLET >= SHOOT_HEAT_LIMIT)
    {
        control->heat = SHOOT_HEAT_LIMIT;
    }
    else
    {
        control->heat = (uint16_t)(control->heat + SHOOT_HEAT_PER_BULLET);
    }
    control->heat_limit_active = shoot_task_fire_heat_would_over_limit(control);
    control->fire_detect_active = false;
    control->fire_detect_ticks = 0U;
}

/**
  * @brief          预测下一发是否会触发热量禁发
  * @retval         true 表示下一发会达到或超过热量上限
  */
static bool shoot_task_fire_heat_would_over_limit(const shoot_task_control_t *control)
{
    if (control == NULL)
    {
        return true;
    }

    return ((uint32_t)control->heat + SHOOT_HEAT_PER_BULLET) >= SHOOT_HEAT_LIMIT;
}

static float shoot_task_min_float(float a, float b)
{
    return (a < b) ? a : b;
}

static float shoot_task_clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        value = min_value;
    }
    else if (value > max_value)
    {
        value = max_value;
    }

    return value;
}

static bool shoot_task_motor_ready(const shoot_task_motor_t *motor, uint32_t now)
{
    if (motor == NULL || motor->measure == NULL)
    {
        return false;
    }

    if ((now - motor->measure->last_fdb_time) > SHOOT_FRIC_FDB_TIMEOUT)
    {
        return false;
    }

    if (motor->measure->temperate >= SHOOT_FRIC_TEMP_LIMIT)
    {
        return false;
    }

    return true;
}
#endif
