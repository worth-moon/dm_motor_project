#ifndef __RC_H
#define __RC_H

#include "main.h"
#include "stdbool.h"
#include "can_bsp.h"
#include "dmotor.h"

void robot_arm(float X, float Y);
void Gravity_Compensation(void);

#endif
