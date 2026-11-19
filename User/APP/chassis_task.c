/**
  * @file       chassis_task.c
  * @brief      底盘任务入口与弱接口
  * @note       串联底盘模式、反馈、控制、发送的周期任务链路。
  */
#include "chassis_task.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include <string.h>

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

chassis_move_t chassis_move;

static uint16_t chassis_odometer_crc16(const uint8_t *data, uint8_t len)
{

    uint16_t crc = 0xFFFFU;
    uint8_t i;
    uint8_t bit;
    for (i = 0U; i < len; i++)
    {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; bit++)
        {
            crc = (crc & 1U) != 0U ? (uint16_t)((crc >> 1) ^ 0xA001U) : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

void chassis_odometer_can_rx(uint16_t can_id, const uint8_t *data, uint8_t len)
{
    if (data == NULL || len == 0U)
    {
        return;
    }

    chassis_move.odometer.can_id = can_id;
    chassis_move.odometer.data_len = len;
    if (len <= sizeof(chassis_move.odometer.raw))
    {
        memcpy(chassis_move.odometer.raw, data, len);
    }

    /* 里程计完整帧沿用例程格式：A5 5A + 6 个 float + CRC16，共 28 字节。 */
    if (len < 28U || data[0] != 0xA5U || data[1] != 0x5AU ||
        chassis_odometer_crc16(data, 26U) !=
        ((uint16_t)data[26] | ((uint16_t)data[27] << 8)))
    {
        return;
    }
    memcpy(&chassis_move.odometer.x_mm, &data[2], sizeof(float));
    memcpy(&chassis_move.odometer.y_mm, &data[6], sizeof(float));
    memcpy(&chassis_move.odometer.yaw_deg, &data[10], sizeof(float));
    memcpy(&chassis_move.odometer.gyro_dps, &data[14], sizeof(float));
    memcpy(&chassis_move.odometer.vx_mps, &data[18], sizeof(float));
    memcpy(&chassis_move.odometer.vy_mps, &data[22], sizeof(float));
    chassis_move.odometer.vx_mps *= 0.001f;
    chassis_move.odometer.vy_mps *= 0.001f;
    chassis_move.odometer.last_update_tick = HAL_GetTick();
    chassis_move.odometer.valid = 1U;
}

/**
  * @brief          底盘任务入口
  * @retval         none
  */
void chassis_task(void *pvParameters)
{
	vTaskDelay(CHASSIS_TASK_INIT_TIME);
	chassis_init(&chassis_move);

	while (1)
	{
		chassis_set_mode(&chassis_move);
		chassis_mode_change_control_transit(&chassis_move);
		chassis_feedback_update(&chassis_move);
		chassis_set_contorl(&chassis_move);
		chassis_control_loop(&chassis_move);
		chassis_send_cmd(&chassis_move);

		osDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
		chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
}

/**
  * @brief          底盘初始化弱接口
  * @retval         none
  */
__weak void chassis_init(chassis_move_t *chassis_move_init)
{
	(void)chassis_move_init;
}

/**
  * @brief          底盘模式设置弱接口
  * @retval         none
  */
__weak void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
	(void)chassis_move_mode;
}

/**
  * @brief          底盘模式切换过渡弱接口
  * @retval         none
  */
__weak void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
	(void)chassis_move_transit;
}

/**
  * @brief          底盘反馈更新弱接口
  * @retval         none
  */
__weak void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
	(void)chassis_move_update;
}

/**
  * @brief          底盘控制量设置弱接口
  * @retval         none
  */
__weak void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
	(void)chassis_move_control;
}

/**
  * @brief          底盘闭环控制弱接口
  * @retval         none
  */
__weak void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
	(void)chassis_move_control_loop;
}

/**
  * @brief          底盘电机命令发送弱接口
  * @retval         none
  */
__weak void chassis_send_cmd(chassis_move_t *chassis_move_send)
{
	(void)chassis_move_send;
}
