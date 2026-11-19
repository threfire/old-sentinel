/**
 * @file    bsp_fdcan.c
 * @brief   FDCAN 收发及接收帧分发。
 */
#include "bsp_fdcan.h"
#include "stdint.h"
#include "robot_param.h"
#include "detect_task.h"
#include "pm01_api.h"
#include "wattmeter_api.h"
#include "chassis_task.h"

__IO CAN_t can = {0};
__IO CAN_ErrorStatus can_error_status = CAN_ERROR_NONE;

uint8_t len1,len2,len3;

uint8_t rx_data1[8] = {0};
uint16_t rec_id1;
uint8_t rx_data2[8] = {0};
uint16_t rec_id2;
uint8_t rx_data3[8] = {0};
uint16_t rec_id3;

uint8_t MOTOR_Data[8]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 电机数据

uint8_t MOTOR_Enable[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};   // 电机使能命令
uint8_t MOTOR_Save_zero[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE}; // 电机保存零点命令
uint8_t RS_MOTOR_PRE_MODE[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFD}; // 灵足电机私有模式
uint8_t RS_MOTOR_MIT_MODE[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x02, 0xFD}; // 灵足电机mit模式
/**
************************************************************************
* @brief:      	bsp_can_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN初始化
************************************************************************
**/
void bsp_can_init(void)
{
	motor_feedback_init();
	can1_filter_init();
	can2_filter_init();
	can3_filter_init();
	HAL_FDCAN_Start(&hfdcan1);                               //启动FDCAN
	HAL_FDCAN_Start(&hfdcan2);
	HAL_FDCAN_Start(&hfdcan3);
	HAL_FDCAN_ActivateNotification(&hfdcan1, 
                               FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                               FDCAN_IT_ERROR_WARNING |
                               FDCAN_IT_ERROR_PASSIVE |
                               FDCAN_IT_BUS_OFF |
                               FDCAN_IT_ARB_PROTOCOL_ERROR |
                               FDCAN_IT_DATA_PROTOCOL_ERROR,
                               0);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/**
************************************************************************
* @brief:      	can_filter_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN滤波器初始化
************************************************************************
**/
void can1_filter_init(void)
{
	FDCAN_FilterTypeDef fdcan_filter;
	
	fdcan_filter.IdType = FDCAN_STANDARD_ID;                       // 改为扩展ID
	fdcan_filter.FilterIndex = 0;                                  // 滤波器索引
	fdcan_filter.FilterType = FDCAN_FILTER_MASK;                   
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           // 过滤器关联到FIFO0
	fdcan_filter.FilterID1 = 0x0000;                               // 滤波器ID1
	fdcan_filter.FilterID2 = 0x0000;                               // 滤波器ID2

	HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter);
	
	// 配置全局滤波器：拒绝所有不匹配的帧
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, 
//		FDCAN_REJECT, 
//		FDCAN_REJECT, 
		FDCAN_ACCEPT_IN_RX_FIFO0,  // 接收所有标准帧
		FDCAN_ACCEPT_IN_RX_FIFO0,  // 接收所有扩展帧
		FDCAN_FILTER_REMOTE, 
		FDCAN_FILTER_REMOTE);
		
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);
}

void can2_filter_init(void)
{
	FDCAN_FilterTypeDef fdcan_filter;
	
	fdcan_filter.IdType = FDCAN_STANDARD_ID;                       // 改为扩展ID
	fdcan_filter.FilterIndex = 0;                                  // 滤波器索引
	fdcan_filter.FilterType = FDCAN_FILTER_MASK;                   
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;           // 过滤器关联到FIFO1
	fdcan_filter.FilterID1 = 0x0000;                               // 滤波器ID1
	fdcan_filter.FilterID2 = 0x0000;                               // 滤波器ID2

	HAL_FDCAN_ConfigFilter(&hfdcan2, &fdcan_filter);
	
	// 配置全局滤波器：拒绝所有不匹配的帧
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, 
//		FDCAN_REJECT, 
//		FDCAN_REJECT, 
		FDCAN_ACCEPT_IN_RX_FIFO1,  // 接收所有标准帧
		FDCAN_ACCEPT_IN_RX_FIFO1,  // 接收所有扩展帧
		FDCAN_FILTER_REMOTE, 
		FDCAN_FILTER_REMOTE);
		
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO1, 1);
}

void can3_filter_init(void)
{
	FDCAN_FilterTypeDef fdcan_filter;
	
	fdcan_filter.IdType = FDCAN_STANDARD_ID;                       // 改为扩展ID
	fdcan_filter.FilterIndex = 0;                                  // 滤波器索引
	fdcan_filter.FilterType = FDCAN_FILTER_MASK;                   
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           // 过滤器关联到FIFO0
	fdcan_filter.FilterID1 = 0x0000;                               // 滤波器ID1
	fdcan_filter.FilterID2 = 0x0000;                               // 滤波器ID2

	HAL_FDCAN_ConfigFilter(&hfdcan3, &fdcan_filter);
	
	// 配置全局滤波器：拒绝所有不匹配的帧
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, 
//		FDCAN_REJECT, 
//		FDCAN_REJECT, 
		FDCAN_ACCEPT_IN_RX_FIFO0,  // 接收所有标准帧
		FDCAN_ACCEPT_IN_RX_FIFO0,  // 接收所有扩展帧
		FDCAN_FILTER_REMOTE, 
		FDCAN_FILTER_REMOTE);
		
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan3, FDCAN_CFG_RX_FIFO0, 1);
}

static inline int8_t get_mit_motor_index(uint32_t can_id)
{
    switch(can_id)
    {
        case DM_STRUM_MASTER_ID: return SHOOT_STRUM_MIT_INDEX;
        case DM_BIG_YAW_MASTER_ID: return GIMBAL_YAW_MIT_INDEX;
        default: return -1;
    }
}

static inline int8_t get_dji_motor_index(uint32_t can_id)
{
    switch(can_id)
    {
        case CAN_SMALL_YAW_ID: return GIMBAL_SMALL_YAW_DJI_INDEX;
        case CAN_PITCH_6020_ID: return GIMBAL_PITCH_DJI_INDEX;
        case CAN_FRIC1_ID: return SHOOT_FRIC1_DJI_INDEX;
        case CAN_FRIC2_ID: return SHOOT_FRIC2_DJI_INDEX;
        default: return -1;
    }
}
static uint32_t fdcan_len_to_dlc(uint32_t len)
{
    switch(len)
    {
        case 0:  return FDCAN_DLC_BYTES_0;
        case 1:  return FDCAN_DLC_BYTES_1;
        case 2:  return FDCAN_DLC_BYTES_2;
        case 3:  return FDCAN_DLC_BYTES_3;
        case 4:  return FDCAN_DLC_BYTES_4;
        case 5:  return FDCAN_DLC_BYTES_5;
        case 6:  return FDCAN_DLC_BYTES_6;
        case 7:  return FDCAN_DLC_BYTES_7;
        case 8:  return FDCAN_DLC_BYTES_8;
        case 12: return FDCAN_DLC_BYTES_12;
        case 16: return FDCAN_DLC_BYTES_16;
        case 20: return FDCAN_DLC_BYTES_20;
        case 24: return FDCAN_DLC_BYTES_24;
        case 32: return FDCAN_DLC_BYTES_32;
        case 48: return FDCAN_DLC_BYTES_48;
        case 64: return FDCAN_DLC_BYTES_64;
        default: return FDCAN_DLC_BYTES_8;
    }
}
static uint8_t fdcan_dlc_to_len(uint32_t dlc)
{
    switch(dlc)
    {
        case FDCAN_DLC_BYTES_0:  return 0;
        case FDCAN_DLC_BYTES_1:  return 1;
        case FDCAN_DLC_BYTES_2:  return 2;
        case FDCAN_DLC_BYTES_3:  return 3;
        case FDCAN_DLC_BYTES_4:  return 4;
        case FDCAN_DLC_BYTES_5:  return 5;
        case FDCAN_DLC_BYTES_6:  return 6;
        case FDCAN_DLC_BYTES_7:  return 7;
        case FDCAN_DLC_BYTES_8:  return 8;
        case FDCAN_DLC_BYTES_12: return 12;
        case FDCAN_DLC_BYTES_16: return 16;
        case FDCAN_DLC_BYTES_20: return 20;
        case FDCAN_DLC_BYTES_24: return 24;
        case FDCAN_DLC_BYTES_32: return 32;
        case FDCAN_DLC_BYTES_48: return 48;
        case FDCAN_DLC_BYTES_64: return 64;
        default: return 0;
    }
}
/**
************************************************************************
* @brief:      	fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcan：FDCAN句柄
* @param:       id：CAN设备ID
* @param:       data：要发送的数据
* @param:       len：要发送的数据长度
* @retval:     	0-成功, 1-失败
* @details:    	发送数据
************************************************************************
**/
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{	
    FDCAN_TxHeaderTypeDef pTxHeader;
    pTxHeader.Identifier = id;
    pTxHeader.IdType = FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType = FDCAN_DATA_FRAME;
    
    // 经典CAN模式只支持最大8字节数据长度
    if(len > 8) {
        len = 8; // 限制8字节
    }
    pTxHeader.DataLength = fdcan_len_to_dlc(len); // 经典CAN模式下数据长度配置
    
    pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch = FDCAN_BRS_OFF; // 经典CAN模式关闭比特率切换
    pTxHeader.FDFormat = FDCAN_CLASSIC_CAN;   // 经典CAN帧格式
    pTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker = 0;
 
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data) != HAL_OK) 
		return 1; // 失败
	return 0; // 成功	
}
void Motor_ENABLE(FDCAN_HandleTypeDef *hcan, uint16_t id)
{
	canx_send_data(hcan, id, MOTOR_Enable, 8);
}

void Motor_save_zero(FDCAN_HandleTypeDef *hcan, uint16_t id)
{
	canx_send_data(hcan, id, MOTOR_Save_zero, 8);
}
void Motor_MIT_MODE(FDCAN_HandleTypeDef *hcan, uint16_t id)
{
	canx_send_data(hcan, id, RS_MOTOR_MIT_MODE, 8);
}


/**
************************************************************************
* @brief:      	fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint16_t *rec_id, uint8_t *buf)
* @param:       hfdcan：FDCAN句柄
* @param:       rec_id：接收到的ID（扩展ID的高16位）
* @param:       buf：接收数据缓冲区
* @retval:     	接收到的数据长度
* @details:    	接收数据
************************************************************************
**/
uint8_t can1_cnt = 0;
uint8_t can2_cnt = 0;
uint32_t rx1free_level;
uint8_t fdcan1_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf)
{	
	FDCAN_RxHeaderTypeDef pRxHeader;
	uint8_t len = 0;

	if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &pRxHeader, buf) == HAL_OK)
	{
		uint32_t fdb_time = HAL_GetTick();
		*rec_id = (uint16_t)pRxHeader.Identifier;

		len = fdcan_dlc_to_len(pRxHeader.DataLength);
		if(len > 8)
		{
			len = 8;
		}

		switch (pRxHeader.Identifier)
		{
			case DM_BIG_YAW_MASTER_ID:
			{
				int8_t motor_index = get_mit_motor_index(pRxHeader.Identifier);
				if(motor_index < 0 || len != 8U)
				{
					return len;
				}

				can1_cnt = (uint8_t)motor_index;
				motor_feedback_update_mit(can1_cnt, buf, fdb_time);
				detect_hook(YAW_GIMBAL_MOTOR_TOE);
				break;
			}
			case CAN_M1_3508_ID:
			case CAN_M2_3508_ID:
			case CAN_M3_3508_ID:
			case CAN_M4_3508_ID:
			case CAN_M1_6020_ID:
			case CAN_M2_6020_ID:
			case CAN_M3_6020_ID:
			case CAN_M4_6020_ID:
			{
				uint8_t motor_index = (uint8_t)(pRxHeader.Identifier - CAN_M1_3508_ID);
				if (len != 8U)
				{
					break;
				}
				motor_feedback_update_dji(&CHASSIS_MOTOR_MEASURE[motor_index], buf, fdb_time);
				detect_hook((uint8_t)(CHASSIS_MOTOR1_TOE + motor_index));
				break;
			}
			case 0x600:
			case 0x601:
			case 0x602:
			case 0x603:
			case 0x610:
			case 0x611:
			case 0x612:
			case 0x613:
			{
				pm01_response_handle((uint16_t)pRxHeader.Identifier, buf);
				break;
			}
			case WATTMETER_CAN_ID:
			{
				wattmeter_feedback_handle((uint16_t)pRxHeader.Identifier, buf, len);
				break;
			}
			default:
			{
				break;
			}
		}
	}

	return len;
}

uint8_t fdcan2_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf)
{ 
	FDCAN_RxHeaderTypeDef pRxHeader;  
	uint8_t len = 0;

	if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &pRxHeader, buf) == HAL_OK)
	{
		uint32_t fdb_time = HAL_GetTick();

		*rec_id = (uint16_t)pRxHeader.Identifier;

		len = fdcan_dlc_to_len(pRxHeader.DataLength);
		if(len > 8)
		{
			len = 8;
		}

		switch (pRxHeader.Identifier)
		{
			case DM_STRUM_MASTER_ID:
			{
				int8_t motor_index = get_mit_motor_index(pRxHeader.Identifier);
				if(motor_index < 0 || len != 8U)
				{
					return len;
				}

				can2_cnt = (uint8_t)motor_index;
				motor_feedback_update_mit(can2_cnt, buf, fdb_time);
				detect_hook(PLUCK_MOTOR_TOE);
				break;
			}
			case CAN_SMALL_YAW_ID:
			{
				if (len != 8U)
				{
					break;
				}
				motor_feedback_update_dji(&DJI_MOTOR_MEASURE[GIMBAL_SMALL_YAW_DJI_INDEX], buf, fdb_time);
				break;
			}
            case CAN_PITCH_6020_ID:
            {
                if (len != 8U)
                {
                    break;
                }
                motor_feedback_update_dji(&DJI_MOTOR_MEASURE[GIMBAL_PITCH_DJI_INDEX], buf, fdb_time);
                detect_hook(PITCH_GIMBAL_MOTOR_TOE);
                break;
            }
			default:
			{
				break;
			}
		}
	}

	return len;
}

uint8_t fdcan3_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf)
{
	FDCAN_RxHeaderTypeDef pRxHeader;
	uint8_t len = 0;

	if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &pRxHeader, buf) == HAL_OK)
	{
		*rec_id = (uint16_t)pRxHeader.Identifier;

		len = fdcan_dlc_to_len(pRxHeader.DataLength);
		if(len > 8) {
			len = 8;
		}

		if (pRxHeader.Identifier == CAN_FRIC1_ID ||
			pRxHeader.Identifier == CAN_FRIC2_ID)
		{
			int8_t motor_index = get_dji_motor_index(pRxHeader.Identifier);
			if (motor_index >= 0)
			{
				if (len == 8U)
				{
					motor_feedback_update_dji(&DJI_MOTOR_MEASURE[motor_index], buf, HAL_GetTick());
					detect_hook((uint8_t)(FRIC1_MOTOR_TOE + motor_index));
				}
			}
		}

		return len;
	}
	return 0;
}

void fdcan1_rx_callback(void)
{
	len1 = fdcan1_receive(&hfdcan1, &rec_id1, rx_data1);  // 获取实际数据长度
	chassis_odometer_can_rx(rec_id1, rx_data1, len1);

}
void fdcan2_rx_callback(void)
{
	len2 = fdcan2_receive(&hfdcan2, &rec_id2, rx_data2);
}
void fdcan3_rx_callback(void)
{
	len3 = fdcan3_receive(&hfdcan3, &rec_id3, rx_data3);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if(hfdcan == &hfdcan1)
	{
		fdcan1_rx_callback();
	}
    else if(hfdcan == &hfdcan3)
	{
		fdcan3_rx_callback();
	}
}
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if(hfdcan == &hfdcan2)
	{
		fdcan2_rx_callback();
	}
}
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == &hfdcan1) {
		
        uint32_t hal_error = HAL_FDCAN_GetError(hfdcan);
		
        /* 清除旧状态（可根据需求选择清除全部或保留累积状态） */
        can_error_status = CAN_ERROR_NONE;
        
        /* 映射 HAL 错误到自定义状态*/
        if (hal_error & FDCAN_IT_ERROR_WARNING)   can_error_status |= CAN_ERROR_WARNING;
        if (hal_error & FDCAN_IT_ERROR_PASSIVE)   can_error_status |= CAN_ERROR_PASSIVE;
        if (hal_error & FDCAN_IT_BUS_OFF)   can_error_status |= CAN_ERROR_BUS_OFF;
        if (hal_error & FDCAN_IT_ARB_PROTOCOL_ERROR) can_error_status |= CAN_ERROR_PROTOCOL_ARB;
        if (hal_error & FDCAN_IT_DATA_PROTOCOL_ERROR) can_error_status |= CAN_ERROR_PROTOCOL_DATA;
       
    }
}

void CAN_cmd_MIT(FDCAN_HandleTypeDef *hcan,uint16_t id, float _pos, float _vel,
float _KP, float _KD, float _torq)
 {
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	const bool is_dm2325 = (id == DM_STRUM_CAN_ID);
	const float p_min = is_dm2325 ? DM2325_P_MIN : P_MIN;
	const float p_max = is_dm2325 ? DM2325_P_MAX : P_MAX;
	const float v_min = is_dm2325 ? DM2325_V_MIN : V_MIN;
	const float v_max = is_dm2325 ? DM2325_V_MAX : V_MAX;
	const float kp_min = is_dm2325 ? DM2325_KP_MIN : KP_MIN;
	const float kp_max = is_dm2325 ? DM2325_KP_MAX : KP_MAX;
	const float kd_min = is_dm2325 ? DM2325_KD_MIN : KD_MIN;
	const float kd_max = is_dm2325 ? DM2325_KD_MAX : KD_MAX;
	const float t_min = is_dm2325 ? DM2325_T_MIN : T_MIN;
	const float t_max = is_dm2325 ? DM2325_T_MAX : T_MAX;

	pos_tmp = float_to_uint(_pos, p_min, p_max, 16);
	vel_tmp = float_to_uint(_vel, v_min, v_max, 12);
	kp_tmp = float_to_uint(_KP, kp_min, kp_max, 12);
	kd_tmp = float_to_uint(_KD, kd_min, kd_max, 12);
	tor_tmp = float_to_uint(_torq, t_min, t_max, 12);

	
	MOTOR_Data[0] = (pos_tmp >> 8);
	MOTOR_Data[1] = pos_tmp;
	MOTOR_Data[2] = (vel_tmp >> 4);
	MOTOR_Data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	MOTOR_Data[4] = kp_tmp;
	MOTOR_Data[5] = (kd_tmp >> 4);
	MOTOR_Data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	MOTOR_Data[7] = tor_tmp;
	
	canx_send_data(hcan, id , MOTOR_Data, 8);
 }

void CAN_cmd_CHAS_3508(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	uint8_t data[8];

	data[0] = (uint8_t)((uint16_t)motor1 >> 8);
	data[1] = (uint8_t)motor1;
	data[2] = (uint8_t)((uint16_t)motor2 >> 8);
	data[3] = (uint8_t)motor2;
	data[4] = (uint8_t)((uint16_t)motor3 >> 8);
	data[5] = (uint8_t)motor3;
	data[6] = (uint8_t)((uint16_t)motor4 >> 8);
	data[7] = (uint8_t)motor4;

	canx_send_data(&hfdcan1, CAN_CHASSIS_ALL_ID, data, 8U);
}


void CAN_cmd_CHASSIS_ALL(int16_t motor201, int16_t motor202, int16_t motor203, int16_t motor204)
{
	uint8_t data[8];

	data[0] = (uint8_t)((uint16_t)motor201 >> 8);
	data[1] = (uint8_t)motor201;
	data[2] = (uint8_t)((uint16_t)motor202 >> 8);
	data[3] = (uint8_t)motor202;
	data[4] = (uint8_t)((uint16_t)motor203 >> 8);
	data[5] = (uint8_t)motor203;
	data[6] = (uint8_t)((uint16_t)motor204 >> 8);
	data[7] = (uint8_t)motor204;
	canx_send_data(&hfdcan1, CAN_CHASSIS_ALL_ID, data, 8U);
}

void CAN_cmd_CHASSIS_STEER(int16_t motor205, int16_t motor206, int16_t motor207, int16_t motor208)
{
	uint8_t data[8];

	data[0] = (uint8_t)((uint16_t)motor205 >> 8);
	data[1] = (uint8_t)motor205;
	data[2] = (uint8_t)((uint16_t)motor206 >> 8);
	data[3] = (uint8_t)motor206;
	data[4] = (uint8_t)((uint16_t)motor207 >> 8);
	data[5] = (uint8_t)motor207;
	data[6] = (uint8_t)((uint16_t)motor208 >> 8);
	data[7] = (uint8_t)motor208;

	canx_send_data(&hfdcan1, CAN_CHASSIS_STEER_ID, data, 8U);
}

 uint32_t g_can_fail_len = 0xFFFF; 
 uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{
	g_can_fail_len = len;
	FDCAN_TxHeaderTypeDef TxHeader = {0};

	if (hcan == NULL || data == NULL || len > 8U)
	{
		return 1U;
	}
	
	TxHeader.Identifier = id;                 // CAN ID
  TxHeader.IdType =  FDCAN_STANDARD_ID ;        
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;  
	TxHeader.DataLength = fdcan_len_to_dlc(len);
	TxHeader.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;//比特率切换关闭，不适用于经典CAN
  TxHeader.FDFormat =  FDCAN_CLASSIC_CAN;            // CANFD
  TxHeader.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;  
  TxHeader.MessageMarker = 0;//消息标记

   // 发送CAN指令
  if(HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, data) != HAL_OK)
  {
       return 1;
  }
  return 0;

}
int float_to_uint(float x, float x_min, float x_max, int bits){
 float span = x_max - x_min;
 float offset = x_min;
 return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}
