#ifndef __BSP_FDCAN_H__
#define __BSP_FDCAN_H__

#include "main.h"
#include "fdcan.h"
#include "stdbool.h"
#include "motor_feedback.h"

#define hcan_t FDCAN_HandleTypeDef

typedef struct
{
    __IO bool rxFrameFlag;
} CAN_t;

extern __IO CAN_t can;

typedef enum
{
    CAN_ERROR_NONE          = 0x00,
    CAN_ERROR_WARNING       = 0x01,
    CAN_ERROR_PASSIVE       = 0x02,
    CAN_ERROR_BUS_OFF       = 0x04,
    CAN_ERROR_PROTOCOL_ARB  = 0x08,
    CAN_ERROR_PROTOCOL_DATA = 0x10,
    CAN_ERROR_STUFF         = 0x20,
    CAN_ERROR_FORM          = 0x40,
    CAN_ERROR_ACK           = 0x80,
    CAN_ERROR_CRC           = 0x100,
    CAN_ERROR_SEND          = 0x200,
} CAN_ErrorStatus;

extern __IO CAN_ErrorStatus can_error_status;

void bsp_can_init(void);
void can1_filter_init(void);
void can2_filter_init(void);
void can3_filter_init(void);
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t fdcan1_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf);
uint8_t fdcan2_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf);
uint8_t fdcan3_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf);
void fdcan1_rx_callback(void);
void fdcan2_rx_callback(void);
void fdcan3_rx_callback(void);
uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);
void CAN_cmd_MIT(FDCAN_HandleTypeDef *hcan, uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq);
void CAN_cmd_CHAS_3508(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN_cmd_CHASSIS_ALL(int16_t motor201, int16_t motor202, int16_t motor203, int16_t motor204);
void CAN_cmd_CHASSIS_STEER(int16_t motor205, int16_t motor206, int16_t motor207, int16_t motor208);
void Motor_save_zero(FDCAN_HandleTypeDef *hcan, uint16_t id);
void Motor_ENABLE(FDCAN_HandleTypeDef *hcan, uint16_t id);
void Motor_MIT_MODE(FDCAN_HandleTypeDef *hcan, uint16_t id);
extern int float_to_uint(float x, float x_min, float x_max, int bits);

#endif
