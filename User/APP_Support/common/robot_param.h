/**
  * @file       robot_param.h
  * @brief      机器人全局参数宏
  * @note       集中定义机器人类型、CAN ID、云台、底盘、发射和功率控制参数。
  */
#ifndef ROBOT_PARAM_H
#define ROBOT_PARAM_H

#include "robot_map.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PI 3.14159265358979323846f
#define GIMBAL_PI PI

/* ========================= 机器人全局配置 ========================= */
/* 代码运行模式 */
#define debug   0
#define release 1

/* 超级电容开关 */
#define Cap_off 0X00
#define Cap_on  0X01

#define ROBOT_MODE        debug     // 当前代码模式：debug/release
#define ROBOT_CAP         Cap_off   // 当前超级电容状态

#define ROBOT_TYPE_HERO                  1
#define ROBOT_GIMBAL_YAW_PITCH_DIRECT   1
#define ROBOT_CHASSIS_OMNI              1
#define ROBOT_CHASSIS_STEER             2
#define ROBOT_FRICTION_3508             1

#define ROBOT_TYPE      ROBOT_TYPE_HERO
#define ROBOT_GIMBAL    ROBOT_GIMBAL_YAW_PITCH_DIRECT
#define ROBOT_CHASSIS   ROBOT_CHASSIS_STEER
#define ROBOT_FRICTION  ROBOT_FRICTION_3508


/* ========================= 云台角度 PID 参数 ========================= */
/* yaw 轴陀螺仪绝对角控制 PID */
#define YAW_GYRO_ABSOLUTE_PID_KP           2.555f
#define YAW_GYRO_ABSOLUTE_PID_KI           0.0f
#define YAW_GYRO_ABSOLUTE_PID_KD           0.64f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT      10.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT     0.0f

/* pitch 轴编码器相对角控制 PID */
#define PITCH_6020_CURRENT_LIMIT_A        3.0f
#define PITCH_6020_ANGLE_PID_KP           5.50f
#define PITCH_6020_ANGLE_PID_KI           0.0f
#define PITCH_6020_ANGLE_PID_KD           0.08f
#define PITCH_6020_ANGLE_PID_MAX_OUT      2.0f
#define PITCH_6020_SPEED_PID_KP           1.0f
#define PITCH_6020_SPEED_PID_KI           0.02f
#define PITCH_6020_SPEED_PID_KD           0.1f
#define PITCH_6020_SPEED_PID_MAX_OUT      3.0f
#define PITCH_6020_SPEED_PID_MAX_IOUT     0.2f
#define PITCH_6020_FEEDBACK_SIGN          1.0f

/* yaw 轴编码器相对角控制 PID */
#define YAW_ENCODE_RELATIVE_PID_KP         YAW_GYRO_ABSOLUTE_PID_KP
#define YAW_ENCODE_RELATIVE_PID_KI         YAW_GYRO_ABSOLUTE_PID_KI
#define YAW_ENCODE_RELATIVE_PID_KD         YAW_GYRO_ABSOLUTE_PID_KD
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT    YAW_GYRO_ABSOLUTE_PID_MAX_OUT
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT   YAW_GYRO_ABSOLUTE_PID_MAX_IOUT

/* 小 yaw GM6020 双环 PID 与电流限幅 */
#define SMALL_YAW_6020_CURRENT_LIMIT_A    3.0f
#define SMALL_YAW_ANGLE_PID_KP            5.80f
#define SMALL_YAW_ANGLE_PID_KI            0.0f
#define SMALL_YAW_ANGLE_PID_KD            0.08f
#define SMALL_YAW_ANGLE_PID_MAX_OUT       2.0f
#define SMALL_YAW_SPEED_PID_KP            1.4f
#define SMALL_YAW_SPEED_PID_KI            0.02f
#define SMALL_YAW_SPEED_PID_KD            0.1f
#define SMALL_YAW_SPEED_PID_MAX_OUT       SMALL_YAW_6020_CURRENT_LIMIT_A
#define SMALL_YAW_SPEED_PID_MAX_IOUT      0.2f
#define SMALL_YAW_SPEED_FEEDBACK_ALPHA    0.05f
#define SMALL_YAW_SPEED_ERROR_LIMIT_RAD_S 3.5f

/* ========================= 云台前馈与输出配置 ========================= */
#define YAW_REF_VEL_FILTER_ALPHA           0.05f   // yaw 目标角差分速度低通系数
#define YAW_REF_ACCEL_LIMIT                100.0f  // yaw 惯量前馈参考加速度限幅
#define PITCH_VELOCITY_FF_GAIN             0.1f    // pitch 速度前馈系数，单位 N*m/(rad/s)

/* 惯量前馈：torque_ff = J * alpha_ref */
#define YAW_INERTIA_KGM2                   0.18   // yaw 转动惯量 J，单位 kg*m^2
#define PITCH_EQ_MASS_KG                   1.5f    // pitch 重力补偿使用的等效质量
#define PITCH_INERTIA_KGM2                 0.00245 // pitch 转动惯量 J，单位 kg*m^2

/* ========================= 云台静摩擦补偿配置 ========================= */
#define GIMBAL_YAW_STATIC_FRICTION_COMP           0.28f
#define GIMBAL_YAW_STATIC_FRICTION_DEADBAND       0.002f
#define GIMBAL_YAW_STATIC_FRICTION_FULLBAND       0.006f
#define GIMBAL_YAW_STATIC_FRICTION_FILTER_ALPHA   0.2f

#define GIMBAL_PITCH_STATIC_FRICTION_COMP_UP      0.1f
#define GIMBAL_PITCH_STATIC_FRICTION_COMP_DOWN    0.00f
#define GIMBAL_PITCH_STATIC_FRICTION_DEADBAND     0.004f
#define GIMBAL_PITCH_STATIC_FRICTION_FULLBAND     0.008f
#define GIMBAL_PITCH_STATIC_FRICTION_FILTER_ALPHA 0.1f

/** @brief 视觉状态字 bit0：1 表示检测到有效目标，自动哨兵切换自瞄跟随。 */
#define AUTO_AIM_STATUS_TARGET_VALID       0x0001U

/* 自动哨兵正弦扫描参数：开关、角度幅值、周期和线性扫描速度。 */
#define GIMBAL_DEBUG_SMALL_YAW_SINE_ENABLE 1U
#define GIMBAL_DEBUG_PITCH_SINE_ENABLE     1U
#define SMALL_YAW_SCAN_LIMIT_RAD           (30.0f * PI / 180.0f) /* 小 yaw 幅值，单位 rad */
#define PITCH_SCAN_LIMIT_RAD               (30.0f * PI / 180.0f) /* pitch 幅值，单位 rad */
#define SMALL_YAW_DEBUG_SINE_PERIOD_MS     2000U               /* 小 yaw 正弦周期，单位 ms */
#define PITCH_DEBUG_SINE_PERIOD_MS         1000U               /* pitch 正弦周期，单位 ms */
#define SMALL_YAW_SCAN_SPEED_RAD_S         (20.0f * PI / 180.0f) /* 小 yaw 扫描速度，单位 rad/s */
#define PITCH_SCAN_SPEED_RAD_S             (12.0f * PI / 180.0f) /* pitch 扫描速度，单位 rad/s */

/* ========================= 遥控与鼠标输入配置 ========================= */
#define GIMBAL_ANGLE_Z_RC_SEN              0.0000005f // 小陀螺底盘旋转角速度输入灵敏度
#define YAW_CHANNEL                        2          // yaw 遥控通道
#define PITCH_CHANNEL                      3          // pitch 遥控通道
#define GIMBAL_MODE_CHANNEL                0          // 云台模式切换通道
#define WZ_CHANNEL                         2          // 底盘旋转通道
#define TURN_KEYBOARD                      KEY_PRESSED_OFFSET_F // 小陀螺按键
#define TURN_SPEED                         0.04f      // 小陀螺旋转速度
#define TEST_KEYBOARD                      KEY_PRESSED_OFFSET_B // 测试按键
#define RC_DEADBAND                        10         // 遥控器死区
#define YAW_RC_SEN                         -0.0000025f // yaw 遥控灵敏度
#define PITCH_RC_SEN                       -0.000003f // pitch 遥控灵敏度
#define YAW_MOUSE_SEN                      0.000006f  // yaw 鼠标灵敏度
#define PITCH_MOUSE_SEN                    -0.000006f // pitch 鼠标灵敏度
#define YAW_ENCODE_SEN                     0.01f      // yaw 编码器模式输入灵敏度
#define PITCH_ENCODE_SEN                   0.01f      // pitch 编码器模式输入灵敏度

/* ========================= 云台任务与反馈索引配置 ========================= */
#define GIMBAL_TASK_INIT_TIME              500    // 云台任务启动延时，单位 ms
#define GIMBAL_CONTROL_TIME                1      // 云台控制周期，单位 ms
#define INS_YAW_ADDRESS_OFFSET             0      // INS yaw 角数组索引
#define INS_PITCH_ADDRESS_OFFSET           1      // INS pitch 角数组索引
#define INS_ROLL_ADDRESS_OFFSET            2      // INS roll 角数组索引
#define INS_GYRO_X_ADDRESS_OFFSET          0      // INS gyro x 索引
#define INS_GYRO_Y_ADDRESS_OFFSET          1      // INS gyro y 索引
#define INS_GYRO_Z_ADDRESS_OFFSET          2      // INS gyro z 索引
#define GIMBAL_MIT_FEEDBACK_INIT_DELAY     100U   // MIT 电机反馈初始化延时，单位 ms

/* ========================= 拨弹控制参数 ========================= */
#define SHOOT_STRUM_LONG_PRESS_MS          500U        // 长按判定时间，单位 ms
#define SHOOT_STRUM_DIRECTION             -1.0f        // 拨弹方向符号
#define SHOOT_STRUM_FDB_TIMEOUT            100U        // 拨弹反馈超时，单位 ms

/* ========================= 云台机械限位与初始化配置 ========================= */
#define PITCH_HEAD_UP_MOTOR_ECD            6600U        // pitch 抬头边界，GM6020 编码器值
#define PITCH_HEAD_DOWN_MOTOR_ECD          7350U       // pitch 低头边界，GM6020 编码器值
#define HALF_ECD_RANGE                     4096        // 编码器半量程
#define ECD_RANGE                          8191        // 编码器总量程
#define GIMBAL_INIT_ANGLE_ERROR            0.1f        // 初始化目标角允许误差
#define GIMBAL_INIT_STOP_TIME              100         // 初始化停止判定时间
#define GIMBAL_INIT_TIME                   6000        // 初始化总超时时间
#define GIMBAL_CALI_REDUNDANT_ANGLE        0.1f        // 校准冗余角度
#define GIMBAL_INIT_PITCH_SPEED            0.004f      // pitch 初始化速度
#define GIMBAL_INIT_YAW_SPEED              0.005f      // yaw 初始化速度
#define GIMBAL_CALI_MOTOR_SET              8000        // 校准时电机输出
#define GIMBAL_CALI_STEP_TIME              2000        // 校准步骤持续时间
#define GIMBAL_CALI_GYRO_LIMIT             0.1f        // 校准静止角速度阈值
#define GIMBAL_CALI_PITCH_MAX_STEP         1           // pitch 最大角校准步骤
#define GIMBAL_CALI_PITCH_MIN_STEP         2           // pitch 最小角校准步骤
#define GIMBAL_CALI_YAW_MAX_STEP           3           // yaw 最大角校准步骤
#define GIMBAL_CALI_YAW_MIN_STEP           4           // yaw 最小角校准步骤
#define GIMBAL_CALI_START_STEP             GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP               5
#define GIMBAL_MOTIONLESS_RC_DEADLINE      10          // 进入静止行为的遥控死区阈值
#define GIMBAL_MOTIONLESS_TIME_MAX         3000        // 静止行为最大保持时间

#define INIT_YAW_SET                       0.0f        // yaw 初始化目标角
#define INIT_PITCH_SET                     1.425f      // pitch 初始化绝对目标角（rad）

/* ========================= pitch 重力补偿配置 ========================= */
#define PITCH_GRAVITY_COMP_MASS_KG         PITCH_EQ_MASS_KG // pitch 重力补偿质量
#define PITCH_GRAVITY_COMP_COM_FORWARD_M   0.04f            // 质心前向距离
#define PITCH_GRAVITY_COMP_COM_UP_M        0.02f            // 质心上向距离
#define PITCH_GRAVITY_COMP_OUTPUT_LIMIT    (T_MAX / 5.0f)   // 重力补偿输出限幅

/* ========================= 射速估计参数 ========================= */
#define SHOOT_FRIC_WHEEL_RADIUS_M          0.020f
#define SHOOT_BULLET_42MM_MASS_KG          0.0404f
#define SHOOT_FRIC_ROTATING_MASS_KG        0.1793f
#define SHOOT_FRIC_ROTATING_INERTIA_KGM2   (0.5f * SHOOT_FRIC_ROTATING_MASS_KG * SHOOT_FRIC_WHEEL_RADIUS_M * SHOOT_FRIC_WHEEL_RADIUS_M)
#define SHOOT_BULLET_SPEED_EST_TRIGGER_DROP_RPM 600.0f
#define SHOOT_BULLET_SPEED_EST_MIN_SPEED_RATIO  0.85f
#define SHOOT_BULLET_SPEED_EST_WINDOW_MS   20U
#define SHOOT_BULLET_SPEED_EST_COEFF_MPS_PER_RPM \
    ((2.0f * SHOOT_FRIC_ROTATING_INERTIA_KGM2 * 2.0f * PI) / \
     (60.0f * SHOOT_BULLET_42MM_MASS_KG * SHOOT_FRIC_WHEEL_RADIUS_M))
#define SHOOT_FIRE_DETECT_TRIGGER_DROP_RPM 600.0f // 开火检测掉速阈值，单位 rpm
#define SHOOT_FIRE_DETECT_MIN_SPEED_RATIO  0.85f  // 摩擦轮达到目标转速比例后允许检测
#define SHOOT_FIRE_DETECT_CURRENT_A        2.0f   // 开火检测反馈电流阈值，单位 A
#define SHOOT_FIRE_DETECT_WINDOW_MS        12U    // 掉速后等待电流响应的窗口，单位 ms
#define SHOOT_FIRE_DETECT_LATCH_MS         50U    // 开火检测结果保持时间，单位 ms

/* ========================= 串口与裁判系统配置 ========================= */
#define USART_RX_BUF_LENGHT                64    // 串口接收缓冲区长度
#define REFEREE_FIFO_BUF_LENGTH            1024  // 裁判系统 FIFO 长度
#ifndef REF_PROTOCOL_FRAME_MAX_SIZE
#define REF_PROTOCOL_FRAME_MAX_SIZE        192   // 裁判系统最大帧长
#endif

/* ========================= 云台机械角度配置 ========================= */
#define GIMBAL_YAW_SAFE_ANGLE_RAD          2.57f        // 大 yaw 朝前机械角（rad）
#define SMALL_YAW_CENTER_ECD                3400U // 小 yaw 中位零点，GM6020 编码器值
#define SMALL_YAW_CCW_LIMIT_ECD             4200U // 小 yaw 逆时针限幅，GM6020 编码器值
#define SMALL_YAW_CW_LIMIT_ECD              2600U // 小 yaw 顺时针限幅，GM6020 编码器值
#define SMALL_YAW_FORWARD_ECD               SMALL_YAW_CENTER_ECD // 小 yaw 朝前零点

/* ========================= 底盘几何与电机映射 ========================= */
/* 四舵轮顺序：左前、左后、右后、右前；+x 前进，+y 左移。 */
#define CHASSIS_STEER_HALF_LENGTH_M      0.20f              // 轮心纵向半距，参考工程 20 cm
#define CHASSIS_STEER_HALF_WIDTH_M       0.175f             // 轮心横向半距，参考工程 17.5 cm
#define CHASSIS_STEER_ECD_TO_RAD         0.000766990394f    // 6020 编码器计数转 rad
#define CHASSIS_STEER_OFFSET_LF_RAD      1.95f              // 左前轮正前方向的安装零偏，待实车校准
#define CHASSIS_STEER_OFFSET_LB_RAD     -1.80f              // 左后轮安装零偏，待实车校准
#define CHASSIS_STEER_OFFSET_RB_RAD      0.90f              // 右后轮安装零偏，待实车校准
#define CHASSIS_STEER_OFFSET_RF_RAD      3.30f              // 右前轮安装零偏，待实车校准
#define CHASSIS_STEER_ANGLE_KP         100.0f              // 6020 角度环输出 rpm/rad
#define CHASSIS_STEER_ANGLE_KI           0.01f             // 6020 角度环积分
#define CHASSIS_STEER_ANGLE_KD           0.0f              // 6020 角度环微分
#define CHASSIS_STEER_ANGLE_MAX_RPM    120.0f              // 6020 目标转速限幅，rpm
#define CHASSIS_STEER_ANGLE_MAX_IOUT   10.0f              // 6020 角度环积分限幅，rpm
#define CHASSIS_STEER_SPEED_KP          55.0f              // 6020 速度环比例
#define CHASSIS_STEER_SPEED_KI           2.4f              // 6020 速度环积分
#define CHASSIS_STEER_SPEED_KD          80.0f              // 6020 速度环微分
#define CHASSIS_STEER_CURRENT_MAX     8000.0f              // 6020 电流命令限幅
#define CHASSIS_STEER_CURRENT_MAX_IOUT 4000.0f             // 6020 电流命令积分限幅
#define CHASSIS_STEER_STOP_SPEED_MPS    0.02f              // 低于此轮速保持舵向目标，m/s
#define Wheel_Radius                      0.0567f            // 由参考轮速换算与 19.2032 减速比推得，单位 m

/* ========================= 底盘任务周期与输入映射 ========================= */
#define CHASSIS_TASK_INIT_TIME            500                // 底盘任务启动延时，单位 ms
#define CHASSIS_CONTROL_TIME_MS           1                  // 底盘控制周期，单位 ms
#define CHASSIS_CONTROL_TIME              0.001f             // 底盘控制周期，单位 s
#define CHASSIS_CONTROL_FREQUENCE         1000.0f            // 底盘控制频率，单位 Hz

#define CHASSIS_X_CHANNEL                 1                  // 底盘 x 方向遥控通道
#define CHASSIS_Y_CHANNEL                 0                  // 底盘 y 方向遥控通道
#define CHASSIS_WZ_CHANNEL                2                  // 底盘旋转遥控通道
#define CHASSIS_MODE_CHANNEL              0                  // 底盘模式切换遥控通道

#define CHASSIS_VX_RC_SEN                 0.004625f          // 底盘 x 方向遥控灵敏度，单位 (m/s)/遥控计数
#define CHASSIS_VY_RC_SEN                 0.004625f          // 底盘 y 方向遥控灵敏度，单位 (m/s)/遥控计数
#define CHASSIS_WZ_RC_SEN                 0.011f             // 底盘旋转摇杆灵敏度，单位 rad/s/遥控计数
#define CHASSIS_YAW_HOLD_RC_SEN           0.0042f            // yaw 保持模式摇杆积分灵敏度，单位 rad/s/遥控计数
#define CHASSIS_RC_DEADLINE               25                 // 底盘遥控死区

#define CHASSIS_FRONT_KEY                 KEY_PRESSED_OFFSET_W // 底盘前进键
#define CHASSIS_BACK_KEY                  KEY_PRESSED_OFFSET_S // 底盘后退键
#define CHASSIS_LEFT_KEY                  KEY_PRESSED_OFFSET_A // 底盘左移键
#define CHASSIS_RIGHT_KEY                 KEY_PRESSED_OFFSET_D // 底盘右移键

/* ========================= 底盘速度上限与规划限幅 ========================= */
#define NORMAL_MAX_CHASSIS_SPEED_X        2.2f               // 底盘 x 方向最大速度，单位 m/s
#define NORMAL_MAX_CHASSIS_SPEED_Y        2.2f               // 底盘 y 方向最大速度，单位 m/s
#define MAX_WHEEL_SPEED                   3.2f               // 单轮目标速度上限，单位 m/s
#define CHASSIS_SPIN_SPEED                3.28f              // 小陀螺最高旋转角速度，单位 rad/s
#define CHASSIS_SPIN_MIN_SPEED            1.0f               // 小陀螺正弦变速波谷角速度，单位 rad/s
#define CHASSIS_SPIN_PERIOD_MS            2000U              // 小陀螺正弦变速周期，单位 ms
#define CHASSIS_SPIN_EXIT_WZ_FILTER_ALPHA 0.003f             // 小陀螺退出角速度低通斜坡系数
#define CHASSIS_SPIN_EXIT_WZ_EPS          0.05f              // 小陀螺退出过渡结束角速度，单位 rad/s

#define CHASSIS_ACCEL_X_NUM               0.1f               // x 方向遥控速度一阶滤波系数，无量纲
#define CHASSIS_ACCEL_Y_NUM               0.1f               // y 方向遥控速度一阶滤波系数，无量纲
#define CHASSIS_MAX_ACCEL                 10.0f              // 底盘平移最大加速度，单位 m/s^2
#define CHASSIS_MAX_JERK                  35.0f              // 底盘平移最大加加速度，单位 m/s^3
#define CHASSIS_STOP_DECEL                28.0f              // 底盘松杆停车减速度，单位 m/s^2
#define CHASSIS_STOP_JERK                 1000.0f            // 底盘松杆停车减速度变化率，单位 m/s^3
#define CHASSIS_WZ_MAX_SPEED              CHASSIS_SPIN_SPEED // chassis maximum angular speed, rad/s
#define CHASSIS_WZ_MAX_ACCEL              18.0f              // 底盘旋转最大角加速度，单位 rad/s^2
#define CHASSIS_WZ_MAX_JERK               160.0f             // 底盘旋转最大角加加速度，单位 rad/s^3
#define CHASSIS_LAT_ACCEL_LIMIT           9.0f               // 底盘高速转弯横向加速度上限，单位 m/s^2
#define CHASSIS_LAT_SPEED_EPS             1.0f               // 横向加速度限幅启用的最小平移速度，单位 m/s

/* ========================= 底盘量纲换算 ========================= */
#define CHASSIS_CURRENT_CMD_FULL_SCALE    16384.0f           // 3508 电流命令满量程，单位 电流命令计数
#define CHASSIS_CURRENT_FULL_SCALE_A      20.0f              // 3508 电流满量程，单位 A
#define CHASSIS_CURRENT_CMD_TO_A          (CHASSIS_CURRENT_FULL_SCALE_A / CHASSIS_CURRENT_CMD_FULL_SCALE) // 电流命令转 A 系数，单位 A/电流命令计数

#define MPS_to_RPM                        3234.16461897f     // 参考工程行走轮速度换算，单位 rpm/(m/s)
#define CHASSIS_RPM_TO_RAD_PER_SEC        0.104719755f       // 电机 rpm 转 rad/s 系数，单位 (rad/s)/rpm

/* ========================= 底盘轮速 PID 与积分清理 ========================= */
#define CHASSIS_SPEED_PI_KP               2400.0f            // 底盘轮速 PID 比例系数，单位 电流命令计数/(m/s)
#define CHASSIS_SPEED_PI_KI               45.0f              // 底盘轮速 PID 积分系数，单位 电流命令计数/m
#define CHASSIS_SPEED_PI_KD               0.0f               // 底盘轮速 PID 微分系数，单位 电流命令计数/(m/s^2)
#define CHASSIS_SPEED_PI_MAX_OUT          4800.0f            // 底盘轮速 PID 输出限幅，单位 电流命令计数
#define CHASSIS_SPEED_PI_MAX_IOUT         650.0f             // 底盘轮速 PID 积分限幅，单位 电流命令计数
#define CHASSIS_ZERO_SPEED_I_CLEAR_CYCLES 1U                 // 零输入且轮速归零后立即清理速度 PI 积分

/* ========================= 底盘电流与功率限幅 ========================= */
#define CHASSIS_CURRENT_BASE_LIMIT_A      4.5f               // 底盘单电机基础电流限幅，单位 A
#define CHASSIS_CURRENT_DYNAMIC_POOL_A    8.0f               // 底盘四电机共享动态电流池，单位 A
#define CHASSIS_CURRENT_SINGLE_MAX_A      9.0f               // 底盘单电机动态限幅上限，单位 A
#define CHASSIS_CURRENT_LIMIT_SLEW_A_PER_S 200.0f            // 底盘单电机限幅变化率，单位 A/s
#define CHASSIS_CURRENT_DEMAND_EPS_A      0.05f              // 底盘动态限幅需求电流阈值，单位 A
#define M3505_MOTOR_SPEED_PID_MAX_OUT     16000.0f           // 底盘 3508 电流输出绝对限幅，单位 电流命令计数

/* ========================= 底盘 yaw 控制 ========================= */
#define CHASSIS_ANGLE_PD_KP               18.0f               // 底盘角度 PD 比例系数，单位 (rad/s)/rad
#define CHASSIS_ANGLE_PD_KD               0.40f              // 底盘角度 PD 微分系数
#define CHASSIS_ANGLE_PD_DEADBAND         0.02f              // 底盘角度 PD 误差死区，单位 rad
#define CHASSIS_ANGLE_PD_MAX_OUT          8.0f               // 底盘角度 PD 输出限幅，单位 rad/s
#define CHASSIS_WZ_SPEED_PI_KP            0.8f               // 底盘整车 wz 速度 PI 比例系数
#define CHASSIS_WZ_SPEED_PI_KI            4.0f               // 底盘整车 wz 速度 PI 积分系数，单位 1/s
#define CHASSIS_WZ_SPEED_PI_MAX_OUT       4.0f               // 底盘整车 wz 速度 PI 修正限幅，单位 rad/s
#define CHASSIS_WZ_SPEED_PI_MAX_IOUT      2.0f               // 底盘整车 wz 速度 PI 积分限幅，单位 rad/s
#define CHASSIS_YAW_RATE_FILTER_TAU       0.01f              // 底盘 yaw 角速度低通时间常数，单位 s
#define CHASSIS_WHEEL_SPEED_FILTER_TAU    0.003f             // 底盘轮速低通时间常数，单位 s
#define CHASSIS_WZ_OUTPUT_DEADBAND        0.05f              // 跟随模式 wz 输出死区，单位 rad/s
#define CHASSIS_YAW_RATE_FEEDBACK_SIGN    1.0f              // 底盘 yaw 陀螺仪角速度反馈方向系数
#define GIMBAL_YAW_FEEDBACK_SIGN          (-1.0f)           // 大 yaw 方向系数：顺时针负、逆时针正
#define CHASSIS_GIMBAL_YAW_FEEDBACK_SIGN  (-1.0f)          // 底盘跟随使用的大 yaw 相对角方向系数

/* ========================= 底盘整车前馈与速度制动补偿 ========================= */
#define ROBOT_MASS                        15.0f               // 整车质量，单位 kg
#define CHASSIS_BODY_FF_YAW_INERTIA_KGM2  0.21f              // 底盘 yaw 轴转动惯量前馈参数，单位 kg*m^2
#define CHASSIS_BODY_FF_YAW_ACCEL_LIMIT   10.0f              // 底盘 yaw 规划角加速度限幅，单位 rad/s^2
#define CHASSIS_BODY_FF_MAX_CURRENT_CMD   2000.0f            // 底盘整车动力学前馈单轮电流命令限幅
#define CHASSIS_BODY_VEL_BRAKE_ENABLE     1U                 // 1: 使能整车速度误差制动补偿
#define CHASSIS_BODY_VEL_BRAKE_LOW_SPEED  0.1f               // 平移制动补偿低速增益结束速度，单位 m/s
#define CHASSIS_BODY_VEL_BRAKE_HIGH_SPEED 0.15f               // 平移制动补偿高速增益开始速度，单位 m/s
#define CHASSIS_BODY_VEL_BRAKE_XY_KP_LOW  150.0f              // 低速平移速度误差转期望减速度系数，单位 (m/s^2)/(m/s)
#define CHASSIS_BODY_VEL_BRAKE_XY_KP_HIGH 300.2f             // 高速平移速度误差转期望减速度系数，单位 (m/s^2)/(m/s)
#define CHASSIS_BODY_VEL_BRAKE_WZ_KP      6.0f               // 旋转速度误差转期望角减速度系数，单位 (rad/s^2)/(rad/s)
#define CHASSIS_BODY_VEL_BRAKE_SPEED_EPS  0.05f              // 平移制动补偿启用速度死区，单位 m/s
#define CHASSIS_BODY_VEL_BRAKE_WZ_EPS     0.05f              // 旋转制动补偿启用角速度死区，单位 rad/s
#define CHASSIS_BODY_VEL_BRAKE_ACCEL_LIMIT 6.0f              // 平移制动补偿期望减速度限幅，单位 m/s^2
#define CHASSIS_BODY_VEL_BRAKE_WZ_ACCEL_LIMIT 30.0f          // 旋转制动补偿期望角减速度限幅，单位 rad/s^2
#define CHASSIS_BODY_VEL_BRAKE_CURRENT_CMD_LIMIT 2400.0f     // 整车速度误差制动补偿单轮电流命令限幅

/* ========================= 底盘单轮前馈与制动状态 ========================= */
#define M3508_TORQUE_CONSTANT             0.3f               // M3508 电机转矩常数，用于转矩换算电流，单位 N*m/A
#define M3508_REDUCTION_RATIO             19.2032f           // M3508 减速比
#define CHASSIS_EFFICIENCY                0.92f              // 底盘传动效率
#define M3508_MAX_CONT_TORQUE             2.8f               // M3508 电机持续输出转矩上限，单位 N*m
#define CHASSIS_FF_VISCOUS_GAIN           0.100f             // 黏性摩擦前馈系数，单位 电流命令计数/(m/s)
#define CHASSIS_FF_COULOMB_CURRENT        0.80f              // 库仑摩擦前馈电流，单位 电流命令计数
#define CHASSIS_FF_COULOMB_SPEED_EPS      0.08f              // 库仑摩擦平滑速度阈值，单位 m/s
#define CHASSIS_FF_STATIC_CURRENT         40.0f              // 静摩擦前馈电流，单位 电流命令计数
#define CHASSIS_FF_STATIC_SPEED_EPS       0.05f              // 静摩擦平滑速度阈值，单位 m/s
#define CHASSIS_BRAKE_FRICTION_FF_SCALE   0.0f               // 制动状态摩擦前馈保留比例
#define CHASSIS_BRAKE_FF_CURRENT_A        0.0f               // 制动状态固定制动前馈电流，单位 A
#define CHASSIS_BRAKE_FF_SPEED_EPS        0.005f             // 制动前馈方向平滑速度阈值，单位 m/s
#define CHASSIS_BRAKE_ENTER_SPEED_EPS     0.02f              // 制动进入实测轮速阈值，单位 m/s
#define CHASSIS_BRAKE_RELEASE_SPEED_EPS   0.01f              // 制动释放实测轮速阈值，单位 m/s

/* ========================= MIT 协议参数范围 ========================= */
#define P_MIN                              -3.14159f    // 位置映射下限（rad），与电机端 PMAX 对应
#define P_MAX                               3.14159f    // 位置映射上限（rad），与电机端 PMAX 一致
#define V_MIN                              -30          // 速度最小值
#define V_MAX                               30          // 速度最大值
#define KP_MIN                              0.0         // Kp 最小值
#define KP_MAX                              500.0       // Kp 最大值
#define KD_MIN                              0.0         // Kd 最小值
#define KD_MAX                              5.0         // Kd 最大值
#define T_MIN                              -10.0f       // 力矩最小值
#define T_MAX                               10.0f       // 力矩最大值

/* DM2325 拨弹电机的 MIT 协议参数范围 */
#define DM2325_P_MIN                       -12.5f
#define DM2325_P_MAX                        12.5f
#define DM2325_V_MIN                       -45.0f
#define DM2325_V_MAX                        45.0f
#define DM2325_KP_MIN                       0.0f
#define DM2325_KP_MAX                     500.0f
#define DM2325_KD_MIN                       0.0f
#define DM2325_KD_MAX                       5.0f
#define DM2325_T_MIN                      -18.0f
#define DM2325_T_MAX                       18.0f

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_PARAM_H */
