/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "memorymap.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_bsp.h"
#include "delay.h"
#include "string.h"

#include "lcd.h"
#include "pic.h"

#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"

#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern float pos, jd_pos;
volatile float distance, distance2;
uint8_t state;
uint64_t error_count, success_count, error_count2;

uint16_t adc_val[1]; 
//uint8_t tx_buffer[100] = "distance:99.99,99.99\r\n";
uint8_t tx_buffer[100];

//以下为卡尔曼滤波器参数
//float P = 1;
float P_;  //对应公式中的p'
float X = 0;
float X_;  //X'
float K = 0;
float Q = 0.01;//噪声
float R = 0.2;  //R如果很大，更相信预测值，那么传感器反应就会迟钝，反之相反

//调试电机方向的
float test_motor1,test_motor2,test_motor3,test_motor4,test_motor5,test_motor6,test_motor7 = 3.0f;

int count = 0;
volatile float X_IN = 0,Y_IN = 0;
float A1 = 23.0, A2 = 12.0, A3 = 12.0, A4 = 12.0, A5 = 12.0, A6 = 9.5; // 假设新增的两个关节也是12.0的长度
float P = 5; // 假定为机械臂底部，抓不到的一个圆形直径
float J1 = 0, J2 = 0, J3 = 0, J4 = 0, J5 = 0, J6 = 0; // 待求,单位是弧度
//float X, Y, Z; // 末端坐标
float Z;
float high, len;
float test_fuzhi = -5.5;

float tolerance = 0.1; // 容差范围，用于比较浮点数
float bu_chang = 0.1; // 步长，用于计算角度
float cur_high, cur_len;

uint64_t forward_count = 0, backward_count = 0;
float dsp_sqrt_in, dsp_sqrt_out;
float c2, c23, c234, c2345, c23456, s2, s23, s234, s2345, s23456;
uint64_t total_iterations = 0; // 新增变量记录总循环次数
uint64_t youxiao_count = 0; // 记录有效次数
uint64_t error_314_count = 0; // 记录错误次数

extern float jxb_motor_pos[7];

uint8_t uart_count;
uint8_t rx_buffer[50];
uint8_t test_flag;
volatile float tar_buffer[20];
volatile float t_output[6],xita_jxb[6];
volatile float alpha[6];
float debug_fuhao = 1,debug_fuhao2 = -1;
float jxb_709[3][7] = { {0,0,0,0,1.57,0,0},{0.128f,0.212f,1.2f,-1.7f,0.47f,-1.37f,3.0f},{0.0059,0.08,0.05,-1.61,1.50,-1.55,3.0} };
float jxb_716[3][7] = { {0,0,0,0,1.57,0,0},{0,0,0,0,0,0,0},{-0.03,2.13,1.96,-1.72,-1.7,1.33,0.43} };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
extern uint8_t CDC_Transmit_HS(uint8_t* Buf, uint16_t Len);
void robot_arm(float X, float Y);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_UART7_Init();
  /* USER CODE BEGIN 2 */
	delay_init(480);
	can_bsp_init();
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,1);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,1);
	HAL_Delay(1000);
	
	uint8_t data[8];
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFC;

    uint8_t disable_data[8] = {0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD};
	
    fdcanx_send_data(&hfdcan1,0x00,data,8);//使能
		HAL_Delay(10);
    fdcanx_send_data(&hfdcan1,0x02,data,8);
    HAL_Delay(10);
//		fdcanx_send_data(&hfdcan1,0x04,data,8);
//    HAL_Delay(10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //修正角度
    xita_jxb[0] = xita[0] + 3.14f/2;
    xita_jxb[1] = - xita[1];
    xita_jxb[2] = xita[2] + 3.14f/2;
    //计算alpha
    alpha[0] = xita_jxb[0];
    alpha[1] = xita_jxb[0] + xita_jxb[1];
    alpha[2] = xita_jxb[0] + xita_jxb[1] + xita_jxb[2];
		
    // 参数定义（新增末端电机参数）
//float m_link = 0.135f, g_count = 9.8f, lc = 0.06f, l_count = 0.12f, m_motor = 0.305f;
float m_link = 0.149f, g_count = 10.0f, lc = 0.06f, l_count = 0.12f, m_motor = 0.345f;
		//t_output[0] = 1.8382 * cosf(alpha[0]);
// 关节1重力补偿力矩：承载整个系统重量
t_output[0] = g_count * (m_link * lc * cosf(alpha[0]) + 
                        m_motor * l_count * cosf(alpha[0]) + 
                        m_link * (l_count * cosf(alpha[0]) + lc * cosf(alpha[1])) +
                        m_motor * (l_count * cosf(alpha[0]) + l_count * cosf(alpha[1])));

// 关节2重力补偿力矩：承载连杆2 + 末端电机重量  
t_output[1] = (g_count * (m_link * lc * cosf(alpha[1]) + 
                        m_motor * l_count * cosf(alpha[1]))) * (-1.0f);

  // 参数定义（保持您的风格）
//  float m_link = 0.120f, g_count = 9.8f, lc = 0.06f, l_count = 0.12f, m_motor = 0.280f;

//  // 关节1重力补偿力矩：τ? = g × [m?×lc×cos(θ?) + m_motor×l?×cos(θ?) + m?×(l?×cos(θ?) + lc?×cos(θ?+θ?))]
//  t_output[0] = g_count * (m_link * lc * cosf(alpha[0]) + 
//                        m_motor * l_count * cosf(alpha[0]) + 
//                        m_link * (l_count * cosf(alpha[0]) + lc * cosf(alpha[1])));

//  // 关节2重力补偿力矩：τ? = g × m? × lc? × cos(θ?+θ?)
//  t_output[1] = (-1.0f)*(g_count * m_link * lc * cosf(alpha[1]) + g_count * m_motor * l_count * cosf(alpha[1])) ;
//    
    //施加力矩到关节电机
    mit_ctrl(&hfdcan1,0x00,0,0,0,0,t_output[0]);
		HAL_Delay(1);
    mit_ctrl(&hfdcan1,0x02,0,0,0,0,t_output[1]);
		HAL_Delay(1);
//		mit_ctrl(&hfdcan1,0x04,0,0,0,0,0);
//		HAL_Delay(1);

      /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/*
* 传入四个参数，rx_data是接收数据的数组，len是数组长度，target_len是目标数据长度，target_data是目标数据数组
* 检测到target_data的数据长度有target_len，则返回1，反之返回0.
* 目标长度最大为20
*/
uint8_t openmv_data_process_float(uint8_t* rx_data, uint8_t len, uint8_t target_len, float* target_data)
{
    // 检查目标长度是否在1到20之间
    if (target_len < 1 || target_len > 20)
    {
        return 0;
    }

    // 查找 '#' 和 ';' 的位置
    char* start = strchr((char*)rx_data, '#');
    char* end = strchr((char*)rx_data, ';');

    // 检查是否找到 '#' 和 ';'
    if (start == NULL || end == NULL || end <= start)
    {
        return 0;
    }

    // 确保 ';' 在 '#' 之后
    if (end <= start)
    {
        return 0;
    }

    // 构建格式字符串
    char format[100] = "#A%f";
    for (uint8_t i = 1; i < target_len; i++) {
        char temp[10];
        snprintf(temp, sizeof(temp), ",%c%%f", 'A' + i);
        strcat(format, temp);
    }
    strcat(format, ";");

    // 提取浮点数
    int scanned = sscanf(start, format, &target_data[0], &target_data[1], &target_data[2], &target_data[3], &target_data[4],
        &target_data[5], &target_data[6], &target_data[7], &target_data[8], &target_data[9],
        &target_data[10], &target_data[11], &target_data[12], &target_data[13], &target_data[14],
        &target_data[15], &target_data[16], &target_data[17], &target_data[18], &target_data[19]);

    // 检查是否成功提取所有浮点数
    if (scanned != target_len) {
        return 0;
    }

    return 1;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    const static uint8_t uart_rx_len = 50;
    const static uint8_t target_flag = 'B';
    const static uint8_t target_len = 5;

    uart_count++;
    if (uart_count > uart_rx_len)
    {
        //test_flag = openmv_data_process_flag(rx_buffer, strlen((const char*)rx_buffer), target_flag);
        test_flag = openmv_data_process_float(rx_buffer, strlen((const char*)rx_buffer), target_len, tar_buffer);
		X_IN = tar_buffer[0];
		Y_IN = tar_buffer[1];
        uart_count = 0;
        memset(rx_buffer, 0, strlen((const char*)rx_buffer));
    }
    HAL_UART_Receive_IT(&huart7, rx_buffer + uart_count, 1);
    //标志位置一后，需要执行的任务
    if (test_flag == 1)
    {

        test_flag = 0;//单次执行需要该语句
    }
}
/*
1. 传入XY坐标
2. 数学库验证
3. 搭建调试系统
4. 能用的前提下，测试计算速度
*/

//这里的坐标原点，应该不是在1号电机的脚下，而是沿Y轴方向再走P的样子
//高度问题,确定是静态误差 -3cm,大抵是测量误差
//X轴负方向对称的问题
//定义为桌面机械臂，仅抓取向前一个扇面和向下的物体
void robot_arm(float X, float Y)
{
    //X = 10; Y = 20; Z = 5;
    Z = 15;//
    J1 = atan((P + Y) / X);
    if (J1 < 0)
    {
        J1 = fabs(J1) + 3.14f;
    }

    high = Z + 3;
    len = sqrt(X * X + (P + Y) * (P + Y));

    //printf("high:%f, len:%f\n", high, len); // 阶段值
    test_fuzhi = fabs(-10.56);
    forward_count = HAL_GetTick();
    for (J2 = 0; J2 < 3.14; J2 += bu_chang)
    {
        for (J3 = 0; J3 < 3.14; J3 += bu_chang)
        {
            for (J4 = 0; J4 < 3.14; J4 += bu_chang)
            {
                for (J5 = 0; J5 < 3.14; J5 += bu_chang)
                {
                    for (J6 = 0; J6 < 3.14; J6 += bu_chang)
                    {
                        total_iterations++; // 记录总循环次数

                        // 检查 J2 + J3 + J4 + J5 + J6 是否超过 3.14
                        if ((J2 + J3 + J4 + J5 + J6) > 3.14)
                        {
                            error_314_count++; // 记录错误次数
                            continue; // 跳过当前组合，不计入 count
                        }
                        youxiao_count++;

//                        c2 = arm_cos_f32(J2);
//                        c23 = arm_cos_f32(J2 + J3);
//                        c234 = arm_cos_f32(J2 + J3 + J4);
//                        c2345 = arm_cos_f32(J2 + J3 + J4 + J5);
//                        c23456 = arm_cos_f32(J2 + J3 + J4 + J5 + J6);

//                        dsp_sqrt_in = 1 - c2 * c2;
//                        arm_sqrt_f32(dsp_sqrt_in, &s2);
//                        dsp_sqrt_in = 1 - c23 * c23;
//                        arm_sqrt_f32(dsp_sqrt_in, &s23);
//                        dsp_sqrt_in = 1 - c234 * c234;
//                        arm_sqrt_f32(dsp_sqrt_in, &s234);
//                        dsp_sqrt_in = 1 - c2345 * c2345;
//                        arm_sqrt_f32(dsp_sqrt_in, &s2345);
//                        dsp_sqrt_in = 1 - c23456 * c23456;
//                        arm_sqrt_f32(dsp_sqrt_in, &s23456);

                        cur_high = A1 + A2 * c2 + A3 * c23 + A4 * c234 + A5 * c2345 + A6 * c23456;
                        cur_len = A2 * s2 + A3 * s23 + A4 * s234 + A5 * s2345 + A6 * s23456;

                        if (fabs(cur_high - high) < tolerance && fabs(len - cur_len) < tolerance)
                        {
                            count++;
														
							test_motor1 = J1;
							test_motor2 = J2;
                            test_motor3 = J3;
							test_motor4 = J4;
							test_motor5 = J5;
							test_motor6 = J6;
                            test_motor7 = 3.14;
                            //printf("第%d个满足条件的角度: J1:%f, J2:%f, J3:%f, J4:%f, J5:%f, J6:%f\n", count, J1, J2, J3, J4, J5, J6);
                            //sprintf(tx_buffer, "第%d个满足条件的角度: J1:%f, J2:%f, J3:%f, J4:%f, J5:%f, J6:%f\n", count, J1, J2, J3, J4, J5, J6);
                            //CDC_Transmit_HS(tx_buffer, strlen((const char*)tx_buffer));
                        }
                    }
                }
            }
        }
    }
    backward_count = HAL_GetTick();
    //printf("满足条件的角度组合个数: %d\n", count);
    //printf("超过 3.14 的组合次数: %d\n", error_3.14_count);
    //printf("总循环次数: %llu\n", total_iterations);
}


/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
