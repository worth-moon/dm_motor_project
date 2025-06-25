#ifndef __CAN_BSP_H__
#define __CAN_BSP_H__
#include "main.h"
#include "fdcan.h"

#define hcan_t FDCAN_HandleTypeDef

extern float xita;
void can_bsp_init(void);
void can_filter_init(void);
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint16_t *rec_id, uint8_t *buf);
void fdcan1_rx_callback(void);
void speed_ctrl(hcan_t* hcan, uint16_t motor_id, float vel);
void pos_speed_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel);
void mit_ctrl(hcan_t* hcan,uint16_t motor_id, float pos, float vel, float kp, float kd, float tor);
#endif /* __CAN_BSP_H_ */

