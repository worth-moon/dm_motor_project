#include "can_bsp.h"
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
float normalize(float value);

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;//25
	float offset = x_min;//-12.5
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

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
	
  TxHeader.Identifier = id;
  TxHeader.IdType = FDCAN_STANDARD_ID;																// 标准ID 
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;														// 数据帧 
  TxHeader.DataLength = len << 16;																		// 发送数据长度 
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;										// 设置错误状态指示 								
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;															// 不开启可变波特率 
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;															// 普通CAN格式 
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;										// 用于发送事件FIFO控制, 不存储 
  TxHeader.MessageMarker = 0x00; 			// 用于复制到TX EVENT FIFO的消息Maker来识别消息状态，范围0到0xFF                
    
  if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data)!=HAL_OK) 
		return 1;//发送
	return 0;	
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
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
		if(hfdcan == &hfdcan1)
		{
			fdcan1_rx_callback();
		}
//		if(hfdcan == &hfdcan2)
//		{
//			fdcan2_rx_callback();
//		}
//		if(hfdcan == &hfdcan3)
//		{
//			fdcan3_rx_callback();
//		}
	}
}
/**
************************************************************************
* @brief:      	fdcan_rx_callback(void)
* @param:       void
* @retval:     	void
* @details:    	供用户调用的接收弱函数
************************************************************************
**/

//motor->para.id = (rx_data[0]) & 0x0F;
//motor->para.state = (rx_data[0]) >> 4;
//motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
//motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
//motor->para.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
//motor->para.pos = uint_to_float(motor->para.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
//motor->para.vel = uint_to_float(motor->para.v_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
//motor->para.tor = uint_to_float(motor->para.t_int, T_MIN, T_MAX, 12);  // (-18.0,18.0)
//motor->para.Tmos = (float)(rx_data[6]);
//motor->para.Tcoil = (float)(rx_data[7]);

uint16_t v_int, t_int;
float motor2_state, motor2_pos, motor2_vel, motor2_tor, motor2_Tmos, motor2_Tcoil;
float pos, jd_pos;
uint16_t pos_int;//这里竟然不会因为符号而错判
float jxb_motor_pos[7];
float xita;
void fdcan1_rx_callback(void)
{
	uint16_t rec_id;
	uint8_t rx_data[8] = { 0 };
	fdcanx_receive(&hfdcan1, &rec_id, rx_data);
	switch (rec_id)
	{
		case 0x01:
			pos_int = (rx_data[1] << 8) | rx_data[2];
			pos = uint_to_float(pos_int, P_MIN, P_MAX, 16);
			//jxb_motor_pos[1] = pos;
			xita = pos;
			motor2_state = (rx_data[0]) >> 4;
			motor2_Tmos = (float)(rx_data[6]);
			motor2_Tcoil = (float)(rx_data[7]);
			
			v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
			motor2_vel = uint_to_float(v_int, -45.0, 45.0, 12); // (-45.0,45.0)
			t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
			motor2_tor = uint_to_float(t_int, -18.0, 18.0, 12);  // (-18.0,18.0)
			break;
		default:
			break;
	}
}

float normalize(float value)
{
	if (value >= 0 && value < 6.28)
	{
		return value; // 0~6.28区间，直接返回
	}
	else if (value >= 6.28 && value < 12.56)
	{
		return value - 6.28; // 6.28~12.56区间，减去6.28
	}
	else if (value >= -6.28 && value < 0)
	{
		return value + 6.28; // -6.28~0区间，加上6.28
	}
	else if (value >= -12.56 && value < -6.28)
	{
		return value + 12.56; // -12.56~-6.28区间，加上12.56
	}
	return value; // 如果不在这些区间，返回原值
}

void speed_ctrl(hcan_t* hcan, uint16_t motor_id, float vel)
{
	uint16_t id;
	uint8_t* vbuf;
	uint8_t data[4];

	id = motor_id + SPEED_MODE;
	vbuf = (uint8_t*)&vel;

	data[0] = *vbuf;
	data[1] = *(vbuf + 1);
	data[2] = *(vbuf + 2);
	data[3] = *(vbuf + 3);

	fdcanx_send_data(hcan, id, data, 4);
}

void pos_speed_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel)
{
	uint16_t id;
	uint8_t* pbuf, * vbuf;
	uint8_t data[8];

	id = motor_id + POS_MODE;
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

int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

void mit_ctrl(hcan_t* hcan,uint16_t motor_id, float pos, float vel, float kp, float kd, float tor) 
{
    uint8_t data[8];
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    uint16_t id = motor_id + MIT_MODE;
    
    // 使用宏定义替代原来的motor->tmp参数
    pos_tmp = float_to_uint(pos, -MOTOR_PMAX, MOTOR_PMAX, 16);
    vel_tmp = float_to_uint(vel, -MOTOR_VMAX, MOTOR_VMAX, 12);
    tor_tmp = float_to_uint(tor, -MOTOR_TMAX, MOTOR_TMAX, 12);
    kp_tmp  = float_to_uint(kp,  KP_MIN, KP_MAX, 12);
    kd_tmp  = float_to_uint(kd,  KD_MIN, KD_MAX, 12);
    
    data[0] = (pos_tmp >> 8);
    data[1] = pos_tmp;
    data[2] = (vel_tmp >> 4);
    data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
    data[4] = kp_tmp;
    data[5] = (kd_tmp >> 4);
    data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
    data[7] = tor_tmp;
    
    fdcanx_send_data(hcan, id, data, 8);
}