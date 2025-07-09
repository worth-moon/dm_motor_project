#include "mc.h"
#include "fdcan.h"
#include "delay.h"
#include "tim.h"
#include "rc.h"
//使能全部电机
void mc_init(void)
{
    for(int i = 0; i < 4; i++)
    {
        all_motors_enable(&hfdcan1);
        delay_ms(10);
    }
    HAL_TIM_Base_Start_IT(&htim3);//开启心跳，这个要在电机初始化之后
}

//心跳函数，10ms一次
void mc_run(void)
{
	Gravity_Compensation();
    //pos_speed_ctrl(&hfdcan1,3,0.0f,1.0f);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3)
	{
		mc_run();
	}
}