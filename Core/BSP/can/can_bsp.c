#include "can_bsp.h"
#include "stdbool.h"
/**
************************************************************************
* @brief:      	can_bsp_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN 使能
************************************************************************
**/
void can_bsp_init(void)
{
	can_filter_init();
	HAL_FDCAN_Start(&hfdcan1);                               //开启FDCAN
//	HAL_FDCAN_Start(&hfdcan2);
//	HAL_FDCAN_Start(&hfdcan3);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
//	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
//	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}
/**
************************************************************************
* @brief:      	can_filter_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN滤波器初始化
************************************************************************
**/
void can_filter_init(void)
{
	FDCAN_FilterTypeDef fdcan_filter;
	
	fdcan_filter.IdType = FDCAN_STANDARD_ID;                       //标准ID
	fdcan_filter.FilterIndex = 0;                                  //滤波器索引                   
	fdcan_filter.FilterType = FDCAN_FILTER_MASK;                   
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           //过滤器0关联到FIFO0  
	fdcan_filter.FilterID1 = 0x00;                               
	fdcan_filter.FilterID2 = 0x00;                               
	HAL_FDCAN_ConfigFilter(&hfdcan1,&fdcan_filter); 		 				  //接收ID2
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);
}
/**
************************************************************************
* @brief:      	fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcan：FDCAN句柄
* @param:       id：CAN设备ID
* @param:       data：发送的数据
* @param:       len：发送的数据长度
* @retval:     	void
* @details:    	发送数据
************************************************************************
**/
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len) 
{	 
    FDCAN_TxHeaderTypeDef TxHeader; 
    uint32_t timeout = 1000; // 超时计数，防止死循环
    
    // 参数检查
    if(hfdcan == NULL || data == NULL || len > 8) 
    {
        return 1;
    }
    
    // 检查FIFO是否有空间
    if(HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0) 
    {
        return 1; // FIFO已满，无法发送
    }
   
    // 配置发送头部
    TxHeader.Identifier = id;
    TxHeader.IdType = FDCAN_STANDARD_ID;                    // 标准ID
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;                // 数据帧
    TxHeader.DataLength = len << 16;                        // 发送数据长度(需要左移16位)
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;        // 设置错误状态指示
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;                 // 不开启可变波特率
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;                  // 普通CAN格式
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;       // 不存储发送事件
    TxHeader.MessageMarker = 0x00;                          // 消息标记
    
    // 将消息添加到发送FIFO
    if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data) != HAL_OK) 
    {
        return 1; // 发送失败
    }
    
    // 等待消息发送完成 - 检查邮箱是否有三个空闲
    while(timeout > 0) 
    {
        if(HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) >= 3) 
        {
            break;
        }
        timeout--;
    }
    
    if(timeout == 0) {
        return 1; // 超时，发送可能失败
    }
    
    return 0; // 发送成功
}
/**
************************************************************************
* @brief:      	fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
* @param:       hfdcan：FDCAN句柄
* @param:       buf：接收数据缓存
* @retval:     	接收的数据长度
* @details:    	接收数据
************************************************************************
**/
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint16_t *rec_id, uint8_t *buf)
{	
	FDCAN_RxHeaderTypeDef fdcan_RxHeader;
  if(HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0, &fdcan_RxHeader, buf)==HAL_OK)
	{
		*rec_id = fdcan_RxHeader.Identifier;
		return fdcan_RxHeader.DataLength>>16;//接收数据
	}
  return 0;	
}
/**
************************************************************************
* @brief:      	HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
* @param:       hfdcan；FDCAN句柄
* @param:       RxFifo0ITs：中断标志位
* @retval:     	void
* @details:    	HAL库的FDCAN中断回调函数
************************************************************************
**/
void fdcan1_rx_callback(void);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
		if(hfdcan == &hfdcan1)
		{
			fdcan1_rx_callback();
		}
	}
}
/*============================== 以此为界，上半部分为CAN外设的二级配置，主要是滤波器配置，开启CAN外设，调通基础的can发送和can接收，并且在fifo0的接收中断调用下列的数据处理函数 ==============================*/
// 电机系统配置宏定义
#define MAX_MOTOR_COUNT 10
#define MOTOR_ID_COUNT 6  // 实际使用的电机数量

// 电机ID映射表 - 集中管理，便于修改
const uint8_t MOTOR_ID_MAP[MOTOR_ID_COUNT] = {0x01, 0x03, 0x05, 0x07, 0x05, 0x06};

// 数据变量定义
uint16_t v_int, t_int;

// 扩展为10个电机的数据数组
float motor_state[MAX_MOTOR_COUNT], motor_pos[MAX_MOTOR_COUNT], motor_vel[MAX_MOTOR_COUNT];
float motor_tor[MAX_MOTOR_COUNT], motor_Tmos[MAX_MOTOR_COUNT], motor_Tcoil[MAX_MOTOR_COUNT];
float pos[MAX_MOTOR_COUNT], jd_pos[MAX_MOTOR_COUNT];
uint16_t pos_int[MAX_MOTOR_COUNT];
float jxb_motor_pos[MAX_MOTOR_COUNT + 1];  // 根据需要调整
float xita[MAX_MOTOR_COUNT];




// 数据转换函数保持不变
float uint_to_float(int x_int, float x_min, float x_max, int bits) 
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

// 主要的数据解析函数 - 基于索引
void analysis_data(uint8_t rx_data[8], uint8_t motor_index) 
{
    // 检查索引有效性
    if (motor_index >= MAX_MOTOR_COUNT) 
    {
        return;
    }
    
    // 解析位置数据 (16位)
    motor_pos[motor_index] = uint_to_float((rx_data[1] << 8) | rx_data[2], -12.5, 12.5, 16);
    
    // 解析温度数据 (8位)
    motor_Tmos[motor_index] = (float)(rx_data[6]);
    motor_Tcoil[motor_index] = (float)(rx_data[7]);
    
    // 解析速度数据 (12位)
    v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    motor_vel[motor_index] = uint_to_float(v_int, -45.0, 45.0, 12);
    
    // 解析扭矩数据 (12位)
    t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
    motor_tor[motor_index] = uint_to_float(t_int, -18.0, 18.0, 12);
}

// CAN接收回调专用函数 - 通过CAN ID查找对应的电机索引
void can_motor_data_handler(uint8_t *rx_data, uint32_t can_id) 
{
    // 查找CAN ID对应的电机索引
    for (uint8_t i = 0; i < MOTOR_ID_COUNT; i++) 
    {
        if (MOTOR_ID_MAP[i] == (uint8_t)can_id) 
        {
            // 找到对应的电机，解析数据
            analysis_data(rx_data, i);
            return;  // 找到后直接返回
        }
    }
    
    // 如果到这里说明CAN ID不在映射表中，可以记录错误或忽略
    // printf("Warning: Unknown CAN ID: 0x%02X\n", (uint8_t)can_id);
}

// 电机数据访问函数
float get_motor_position(uint8_t motor_index) 
{
    if (motor_index >= MAX_MOTOR_COUNT) return 0.0f;
    return motor_pos[motor_index];
}

float get_motor_velocity(uint8_t motor_index) 
{
    if (motor_index >= MAX_MOTOR_COUNT) return 0.0f;
    return motor_vel[motor_index];
}

float get_motor_torque(uint8_t motor_index) 
{
    if (motor_index >= MAX_MOTOR_COUNT) return 0.0f;
    return motor_tor[motor_index];
}

float get_motor_tmos(uint8_t motor_index) 
{
    if (motor_index >= MAX_MOTOR_COUNT) return 0.0f;
    return motor_Tmos[motor_index];
}

float get_motor_tcoil(uint8_t motor_index) 
{
    if (motor_index >= MAX_MOTOR_COUNT) return 0.0f;
    return motor_Tcoil[motor_index];
}

bool is_motor_index_valid(uint8_t motor_index) 
{
    return (motor_index < MOTOR_ID_COUNT);
}

// ===== 使用示例 =====
/*
// 在CAN接收中断/回调中使用
void can_rx_callback(uint32_t can_id, uint8_t rx_data[8]) 
{
    // 直接调用电机数据处理函数
    can_motor_data_handler(rx_data, can_id);
}
*/


void fdcan1_rx_callback(void)
{
    uint16_t rec_id;
    uint8_t rx_data[8] = { 0 };
    int motor_idx;
    
    fdcanx_receive(&hfdcan1, &rec_id, rx_data);
    can_motor_data_handler(rx_data, rec_id);
}




