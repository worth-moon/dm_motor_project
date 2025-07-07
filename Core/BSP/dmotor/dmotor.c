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

#define BROADCAST_CAN_ID 0x7FF
#define MODE_CHANGE_DELAY_MS 100

// 电机工作模式定义
typedef enum {
    MOTOR_MODE_MIT = 0x01,           // MIT模式
    MOTOR_MODE_POSITION_SPEED = 0x02, // 位置速度模式
    MOTOR_MODE_SPEED = 0x03          // 速度模式
} motor_work_mode_t;


// 电机CAN ID映射表 - 基础ID，控制模式会在此基础上加偏移
const uint16_t MOTOR_CAN_ID_MAP[MOTOR_ID_COUNT] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};

// 电机使能/失能命令数据
const uint8_t MOTOR_ENABLE_DATA[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
const uint8_t MOTOR_DISABLE_DATA[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

// 单个电机使能 - 基于索引
void motor_enable(hcan_t* hcan, uint8_t motor_index)
{
    // 检查索引有效性
    if (!is_motor_index_valid_for_control(motor_index)) 
    {
        return;  // 无效索引，直接返回
    }
    
    uint16_t can_id = MOTOR_CAN_ID_MAP[motor_index];
    fdcanx_send_data(hcan, can_id, (uint8_t*)MOTOR_ENABLE_DATA, 8);
    HAL_Delay(10);  // 建议的延时
}

// 单个电机失能 - 基于索引
void motor_disable(hcan_t* hcan, uint8_t motor_index)
{
    // 检查索引有效性
    if (!is_motor_index_valid_for_control(motor_index)) 
    {
        return;  // 无效索引，直接返回
    }
    
    uint16_t can_id = MOTOR_CAN_ID_MAP[motor_index];
    fdcanx_send_data(hcan, can_id, (uint8_t*)MOTOR_DISABLE_DATA, 8);
    HAL_Delay(10);  // 建议的延时
}

// 批量电机使能
void all_motors_enable(hcan_t* hcan)
{
    for (uint8_t i = 0; i < MOTOR_ID_COUNT; i++) 
    {
        motor_enable(hcan, i);
    }
}

// 批量电机失能
void all_motors_disable(hcan_t* hcan)
{
    for (uint8_t i = 0; i < MOTOR_ID_COUNT; i++) 
    {
        motor_disable(hcan, i);
    }
}

// 指定电机列表使能
void motors_enable_by_list(hcan_t* hcan, uint8_t motor_indices[], uint8_t count)
{
    for (uint8_t i = 0; i < count; i++) 
    {
        if (motor_indices[i] < MOTOR_ID_COUNT) 
        {
            motor_enable(hcan, motor_indices[i]);
        }
    }
}

// 指定电机列表失能
void motors_disable_by_list(hcan_t* hcan, uint8_t motor_indices[], uint8_t count)
{
    for (uint8_t i = 0; i < count; i++) 
    {
        if (motor_indices[i] < MOTOR_ID_COUNT) 
        {
            motor_disable(hcan, motor_indices[i]);
        }
    }
}

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

// 保存参数指令格式：[CAN_ID低八位] [CAN_ID高三位] AA 01 00 00 00 00

/**
 * @brief 修改电机工作模式
 * @param hcan CAN句柄指针
 * @param motor_index 电机索引 (0-5对应电机ID 1-6)
 * @param work_mode 工作模式 (MIT=0x01, 位置速度=0x02, 速度=0x03)
 * @note 执行完整的模式修改序列：模式切换指令 -> 失能 -> 保存参数 -> 使能
 */
void motor_change_work_mode(hcan_t* hcan, uint8_t motor_index, motor_work_mode_t work_mode)
{
    // 检查电机索引有效性
    if (!is_motor_index_valid_for_control(motor_index)) 
    {
        return;  // 无效索引，直接返回
    }
	// 获取电机的CAN ID
    uint16_t motor_can_id = MOTOR_CAN_ID_MAP[motor_index];
    
    // 构建工作模式修改指令数据
    // 格式: 01 00 55 0A [模式] 00 00 00
    // 第5个字节(索引4)为工作模式：01=MIT, 02=位置速度, 03=速度
    uint8_t mode_change_cmd[8] = {(uint8_t)(motor_can_id & 0xFF),        // CAN ID低八位
        (uint8_t)((motor_can_id >> 8) & 0x07), 0x55, 0x0A, work_mode, 0x00, 0x00, 0x00};
    
    
    
    // 构建保存参数指令数据
    // 格式: [CAN_ID低八位] [CAN_ID高三位] AA 01 00 00 00 00
    uint8_t save_params_cmd[8] = {
        (uint8_t)(motor_can_id & 0xFF),        // CAN ID低八位
        (uint8_t)((motor_can_id >> 8) & 0x07), // CAN ID高三位 (只取低3位)
        0xAA, 0x01, 0x00, 0x00, 0x00, 0x00
    };
    
    // 步骤1: 广播模式修改指令 (ID: 7FF)
    fdcanx_send_data(hcan, BROADCAST_CAN_ID, mode_change_cmd, 8);
    
    // 步骤2: 延时等待，不检测返回数据
    HAL_Delay(MODE_CHANGE_DELAY_MS);
    
    // 步骤3: 发送失能指令给指定电机
    motor_disable(hcan, motor_index);
	HAL_Delay(50);
    
    // 步骤4: 广播保存电机参数指令 (ID: 7FF)
    fdcanx_send_data(hcan, BROADCAST_CAN_ID, save_params_cmd, 8);
    HAL_Delay(50);  // 短暂延时确保参数保存完成
    
    // 步骤5: 发送使能指令 (使用基础CAN ID 0x0001，无偏置)
	motor_enable(hcan,motor_index);
    // uint16_t base_can_id = 0x0001;
    // fdcanx_send_data(hcan, base_can_id, (uint8_t*)MOTOR_ENABLE_DATA, 8);
    HAL_Delay(10);  // 建议的延迟
}
// ===== 使用示例 =====
/*
void motor_control_example(void)
{
    // === 电机使能/失能示例 ===
    
    // 使能单个电机 (索引0对应第一个电机)
    motor_enable(&hfdcan1, 0);
    
    // 使能多个指定电机
    uint8_t motor_list[] = {0, 2, 4};  // 使能第0、2、4个电机
    motors_enable_by_list(&hfdcan1, motor_list, 3);
    
    // 使能所有电机
    all_motors_enable(&hfdcan1);
    
    // === 电机控制示例 ===
    
    // 控制第0个电机 (对应CAN ID 0x01)
    mit_ctrl(&hfdcan1, 0, 1.5f, 0.0f, 50.0f, 1.0f, 0.0f);
    
    // 控制第1个电机速度 (对应CAN ID 0x02)
    speed_ctrl(&hfdcan1, 1, 10.0f);
    
    // 控制第2个电机位置和速度 (对应CAN ID 0x03)
    pos_speed_ctrl(&hfdcan1, 2, 3.14f, 5.0f);
    
    // === 电机失能示例 ===
    
    // 失能指定电机
    motor_disable(&hfdcan1, 0);
    
    // 失能所有电机
    all_motors_disable(&hfdcan1);
    
    // === 实用功能示例 ===
    
    // 检查电机索引是否有效
    if (is_motor_index_valid_for_control(3)) 
    {
        motor_enable(&hfdcan1, 3);
        mit_ctrl(&hfdcan1, 3, 0.0f, 0.0f, 100.0f, 2.0f, 0.0f);
    }
}

// 完整的电机初始化和控制流程示例
void motor_system_init_and_control(void)
{
    // 1. 使能所有电机
    all_motors_enable(&hfdcan1);
    HAL_Delay(100);  // 等待电机准备就绪
    
    // 2. 设置初始位置 (归零)
    for (uint8_t i = 0; i < MOTOR_ID_COUNT; i++)
    {
        mit_ctrl(&hfdcan1, i, 0.0f, 0.0f, 100.0f, 5.0f, 0.0f);
    }
    HAL_Delay(500);  // 等待到达目标位置
    
    // 3. 执行运动控制
    mit_ctrl(&hfdcan1, 0, 1.0f, 0.0f, 50.0f, 2.0f, 0.0f);  // 电机0移动到1.0弧度
    speed_ctrl(&hfdcan1, 1, 5.0f);                           // 电机1以5rad/s速度运行
    
    // 4. 系统关闭时的清理
    // 停止电机可以发送零控制指令
    for (uint8_t i = 0; i < MOTOR_ID_COUNT; i++)
    {
        mit_ctrl(&hfdcan1, i, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }
    HAL_Delay(100);
    all_motors_disable(&hfdcan1);  // 失能所有电机
}
*/