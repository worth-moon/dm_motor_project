#ifndef __DMOTOR_H
#define __DMOTOR_H

#include "main.h"

#define P_MIN -12.5f
#define P_MAX 12.5f
#define SPEED_MODE		0x200
#define POS_MODE		0x100
#define MIT_MODE 		0x000
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

// 新增的宏定义
#define MOTOR_PMAX 12.5f
#define MOTOR_VMAX 30.0f
#define MOTOR_TMAX 10.0f

void mit_ctrl(hcan_t* hcan,uint16_t motor_id, float pos, float vel, float kp, float kd, float tor);
void speed_ctrl(hcan_t* hcan, uint16_t motor_id, float vel);
void pos_speed_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel);

#endif
