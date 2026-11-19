/** 
	*	@file       chassis_calculate.h
  * @brief      四舵轮底盘运动学正/逆解算
  *			
	*     000000000000000     00               00    00     00   00         00 
	*           00     0      00                00    00   00     00       00  
	*       00  00000        00000000000000      00  000000000   00000000000000
	*       00  00          00           00    00    000000000     00     00   
	*      00000000000     00  000000    00     000     00           000000    
	*     00    00   0000     00    00   00      00   000000           00      
	*       00000000000       00    00   00           000000           00      
	*       0   00    0       0000000 00 00       00    00       00000000000000
	*       00000000000       00       000       00 00000000000        00      
	*           00            00                000 00000000000        00      
	*           00  00        00          0    000      00             00      
	*      000000000000        00        000  000       00          00 00      
	*       00        00        000000000000            00            00       
	********************************************************************************/

/****** 坐标轴示意图 ******

							 0^ x
						\		|
						 \	|<-------- 芯片板子放在此处
							\ |
			y				 \|
PI/2	<―――――――――― -PI/2
								|`
								|  `
								|    `
							PI|-PI   `

角度范围: [-PI, PI]
两个角度分别∈(0, PI/2)和(-PI/2, -PI)
***************************/

#ifndef __CHASSIS_CALCULATE_H
#define __CHASSIS_CALCULATE_H

#include "main.h"
#include "chassis_task.h"

/**
  * @brief          初始化舵轮角度零偏
  * @retval         none
  */
void chassis_wheel_angle_offset_init(void);

/**
  * @brief          旋转二维向量
  * @retval         none
  */
void vector_rotate(float angle, float *vector);

/**
  * @brief          执行底盘打滑补偿
  * @retval         none
  */
void slip_control(chassis_move_t *chassis_move);

/**
  * @brief          按横向加速度限制底盘角速度
  * @retval         限幅后的角速度
  */
float chassis_limit_wz_by_lateral_accel(float vx, float vy, float wz);

/**
  * @brief          执行底盘逆运动学解算
  * @retval         none
  */
void chas_inv_cal(float vx_set, float vy_set, float wz_set, float *wheel_angle, float *wheel_speed);

/**
  * @brief          执行底盘正运动学解算
  * @retval         none
  */
void chas_for_cal(float *wheel_angle, float *wheel_speed, float *vx, float *vy, float *wz);


#endif
