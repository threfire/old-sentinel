#ifndef ROBOT_MAP_H
#define ROBOT_MAP_H

/* CAN1: 大 yaw MIT 电机与四组 3508 行走、GM6020 舵向电机。 */
#define DM_BIG_YAW_CAN_ID           0x02U
#define DM_BIG_YAW_MASTER_ID        0x52U
#define DM_YAW_CAN_ID               DM_BIG_YAW_CAN_ID
#define DM_YAW_MASTER_ID            DM_BIG_YAW_MASTER_ID
#define GIMBAL_YAW_MIT_INDEX        1U

#define CAN_CHASSIS_ALL_ID          0x200U
#define CAN_CHASSIS_STEER_ID        0x1FFU
#define CAN_M1_3508_ID              0x201U
#define CAN_M2_3508_ID              0x202U
#define CAN_M3_3508_ID              0x203U
#define CAN_M4_3508_ID              0x204U
#define CAN_M1_6020_ID              0x205U
#define CAN_M2_6020_ID              0x206U
#define CAN_M3_6020_ID              0x207U
#define CAN_M4_6020_ID              0x208U
#define CHASSIS_MODULE_NUM          4U

/* CAN2: 拨弹 MIT 电机、小 yaw 和 pitch GM6020 电机。 */
#define DM_STRUM_CAN_ID             0x01U
#define DM_STRUM_MASTER_ID          0x51U
#define SHOOT_STRUM_MIT_INDEX       0U
#define CAN_SMALL_YAW_ID            0x205U
#define GIMBAL_SMALL_YAW_DJI_INDEX  3U
#define CAN_PITCH_6020_ID           0x206U
#define GIMBAL_PITCH_DJI_INDEX      4U
#define CAN_SMALL_YAW_CURRENT_CMD_ID 0x1FEU

/* CAN3: 两个摩擦轮 3508 电机。 */
#define CAN_FRIC1_ID                0x201U
#define CAN_FRIC2_ID                0x202U
#define SHOOT_FRIC1_DJI_INDEX       0U
#define SHOOT_FRIC2_DJI_INDEX       1U

#endif
