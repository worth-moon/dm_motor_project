#ifndef __OPENMV_H
#define __OPENMV_H

#include "main.h"
#include "stdbool.h"
#include "can_bsp.h"
#include "dmotor.h"
#include <stdio.h>   // ?? snprintf, printf ?
#include <string.h>  // ?? memcmp, memcpy, strlen ?
#include <stdlib.h>  // ?? malloc, free ?

uint8_t openmv_data_process_flag(uint8_t* rx_data, uint8_t len, uint8_t target);
uint8_t openmv_data_process_float(uint8_t* rx_data, uint8_t len, uint8_t target_len, float* target_data);

#endif
