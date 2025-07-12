#ifndef __CAN_BSP_H__
#define __CAN_BSP_H__
#include "main.h"
#include "fdcan.h"

#define hcan_t FDCAN_HandleTypeDef

// 电机系统配置宏定义
#define MAX_MOTOR_COUNT 10
#define MOTOR_ID_COUNT 2  // 实际使用的电机数量
extern float motor_pos[MAX_MOTOR_COUNT];

void can_bsp_init(void);
void can_filter_init(void);
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint16_t *rec_id, uint8_t *buf);
#endif /* __CAN_BSP_H_ */

