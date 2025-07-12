#ifndef __DMOTOR_H
#define __DMOTOR_H

#include "main.h"
#include "stdbool.h"
#include "can_bsp.h"

// 电机工作模式定义
typedef enum {
    MOTOR_MODE_MIT = 0x01,           // MIT模式
    MOTOR_MODE_POSITION_SPEED = 0x02, // 位置速度模式
    MOTOR_MODE_SPEED = 0x03          // 速度模式
} motor_work_mode_t;

// 电机配置
#define MAX_MOTOR_COUNT 10
#define MOTOR_ID_COUNT 2

#define BROADCAST_CAN_ID 0x7FF
#define MODE_CHANGE_DELAY_MS 100

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

bool is_motor_index_valid_for_control(uint8_t motor_index);

// 单个电机使能/失能
void motor_enable(hcan_t* hcan, uint8_t motor_index);
void motor_disable(hcan_t* hcan, uint8_t motor_index);

// 批量电机使能/失能
void all_motors_enable(hcan_t* hcan);
void all_motors_disable(hcan_t* hcan);

// 指定电机列表使能/失能
void motors_enable_by_list(hcan_t* hcan, uint8_t motor_indices[], uint8_t count);
void motors_disable_by_list(hcan_t* hcan, uint8_t motor_indices[], uint8_t count);

// 数据转换函数
int float_to_uint(float x_float, float x_min, float x_max, int bits);

// 电机控制函数
void mit_ctrl(hcan_t* hcan, uint8_t motor_index, float pos, float vel, float kp, float kd, float tor);
void speed_ctrl(hcan_t* hcan, uint8_t motor_index, float vel);
void pos_speed_ctrl(hcan_t* hcan, uint8_t motor_index, float pos, float vel);

// 电机模式切换
void motor_change_work_mode(hcan_t* hcan, uint8_t motor_index, motor_work_mode_t work_mode);

#endif
