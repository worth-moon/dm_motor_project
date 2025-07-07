#include "dmotor.h"

// 注意：以下宏定义应该在头文件中已定义
// #define P_MIN -12.5f
// #define P_MAX 12.5f
// #define SPEED_MODE 0x200
// #define POS_MODE 0x100
// #define MIT_MODE 0x000
// #define KP_MIN 0.0f
// #define KP_MAX 500.0f
// #define KD_MIN 0.0f
// #define KD_MAX 5.0f
// #define MOTOR_PMAX 12.5f
// #define MOTOR_VMAX 30.0f
// #define MOTOR_TMAX 10.0f

// 电机配置
#define MAX_MOTOR_COUNT 10
#define MOTOR_ID_COUNT 6

// 电机CAN ID映射表 - 基础ID，控制模式会在此基础上加偏移
const uint16_t MOTOR_CAN_ID_MAP[MOTOR_ID_COUNT] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};

// 数据转换函数
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    /* Converts a float to an unsigned int, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

// 检查电机索引有效性
bool is_motor_index_valid_for_control(uint8_t motor_index)
{
    return (motor_index < MOTOR_ID_COUNT);
}

// MIT模式控制 - 基于索引
void mit_ctrl(hcan_t* hcan, uint8_t motor_index, float pos, float vel, float kp, float kd, float tor) 
{
    // 检查索引有效性
    if (!is_motor_index_valid_for_control(motor_index)) 
    {
        return;  // 无效索引，直接返回
    }
    
    uint8_t data[8];
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    uint16_t id = MOTOR_CAN_ID_MAP[motor_index] + MIT_MODE;
    
    // 参数转换为整数
    pos_tmp = float_to_uint(pos, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(vel, -MOTOR_VMAX, MOTOR_VMAX, 12);
    tor_tmp = float_to_uint(tor, -MOTOR_TMAX, MOTOR_TMAX, 12);
    kp_tmp  = float_to_uint(kp,  KP_MIN, KP_MAX, 12);
    kd_tmp  = float_to_uint(kd,  KD_MIN, KD_MAX, 12);
    
    // 数据打包
    data[0] = (pos_tmp >> 8);
    data[1] = pos_tmp;
    data[2] = (vel_tmp >> 4);
    data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    data[4] = kp_tmp;
    data[5] = (kd_tmp >> 4);
    data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    data[7] = tor_tmp;
    
    fdcanx_send_data(hcan, id, data, 8);
}

// 速度控制 - 基于索引
void speed_ctrl(hcan_t* hcan, uint8_t motor_index, float vel)
{
    // 检查索引有效性
    if (!is_motor_index_valid_for_control(motor_index)) 
    {
        return;  // 无效索引，直接返回
    }
    
    uint16_t id;
    uint8_t* vbuf;
    uint8_t data[4];

    id = MOTOR_CAN_ID_MAP[motor_index] + SPEED_MODE;
    vbuf = (uint8_t*)&vel;

    data[0] = *vbuf;
    data[1] = *(vbuf + 1);
    data[2] = *(vbuf + 2);
    data[3] = *(vbuf + 3);

    fdcanx_send_data(hcan, id, data, 4);
}

// 位置速度控制 - 基于索引
void pos_speed_ctrl(hcan_t* hcan, uint8_t motor_index, float pos, float vel)
{
    // 检查索引有效性
    if (!is_motor_index_valid_for_control(motor_index)) 
    {
        return;  // 无效索引，直接返回
    }
    
    uint16_t id;
    uint8_t* pbuf, * vbuf;
    uint8_t data[8];

    id = MOTOR_CAN_ID_MAP[motor_index] + POS_MODE;
    pbuf = (uint8_t*)&pos;
    vbuf = (uint8_t*)&vel;

    data[0] = *pbuf;
    data[1] = *(pbuf + 1);
    data[2] = *(pbuf + 2);
    data[3] = *(pbuf + 3);

    data[4] = *vbuf;
    data[5] = *(vbuf + 1);
    data[6] = *(vbuf + 2);
    data[7] = *(vbuf + 3);

    fdcanx_send_data(hcan, id, data, 8);
}

// 批量控制函数 - MIT模式
void mit_ctrl_all_motors(hcan_t* hcan, float pos[], float vel[], float kp[], float kd[], float tor[])
{
    for (uint8_t i = 0; i < MOTOR_ID_COUNT; i++) 
    {
        mit_ctrl(hcan, i, pos[i], vel[i], kp[i], kd[i], tor[i]);
    }
}

// 批量控制函数 - 速度模式
void speed_ctrl_all_motors(hcan_t* hcan, float vel[])
{
    for (uint8_t i = 0; i < MOTOR_ID_COUNT; i++) 
    {
        speed_ctrl(hcan, i, vel[i]);
    }
}

// 批量控制函数 - 位置速度模式
void pos_speed_ctrl_all_motors(hcan_t* hcan, float pos[], float vel[])
{
    for (uint8_t i = 0; i < MOTOR_ID_COUNT; i++) 
    {
        pos_speed_ctrl(hcan, i, pos[i], vel[i]);
    }
}

// 单个电机停止 - 发送零值指令
void motor_stop(hcan_t* hcan, uint8_t motor_index)
{
    mit_ctrl(hcan, motor_index, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

// 所有电机停止
void all_motors_stop(hcan_t* hcan)
{
    for (uint8_t i = 0; i < MOTOR_ID_COUNT; i++) 
    {
        motor_stop(hcan, i);
    }
}

// 获取电机CAN ID (通过索引)
uint16_t get_motor_can_id(uint8_t motor_index)
{
    if (motor_index >= MOTOR_ID_COUNT) 
    {
        return 0x00;  // 返回无效ID
    }
    return MOTOR_CAN_ID_MAP[motor_index];
}

// ===== 使用示例 =====
/*
void motor_control_example(void)
{
    // 控制第0个电机 (对应CAN ID 0x01)
    mit_ctrl(&hfdcan1, 0, 1.5f, 0.0f, 50.0f, 1.0f, 0.0f);
    
    // 控制第1个电机速度 (对应CAN ID 0x02)
    speed_ctrl(&hfdcan1, 1, 10.0f);
    
    // 控制第2个电机位置和速度 (对应CAN ID 0x03)
    pos_speed_ctrl(&hfdcan1, 2, 3.14f, 5.0f);
    
    // 批量控制所有电机
    float positions[MOTOR_ID_COUNT] = {0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
    float velocities[MOTOR_ID_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float kp_values[MOTOR_ID_COUNT] = {50.0f, 50.0f, 50.0f, 50.0f, 50.0f, 50.0f};
    float kd_values[MOTOR_ID_COUNT] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    float torques[MOTOR_ID_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    
    mit_ctrl_all_motors(&hfdcan1, positions, velocities, kp_values, kd_values, torques);
    
    // 停止所有电机
    all_motors_stop(&hfdcan1);
    
    // 检查电机索引是否有效
    if (is_motor_index_valid_for_control(3)) 
    {
        printf("Motor index 3 is valid, CAN ID: 0x%02X\n", get_motor_can_id(3));
    }
}
*/