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
#include "dmotor.h"
#include "mc.h"
#include "rc.h"
#include "vofa.h"
#include "openmv.h"
#include "pid.h"

#include <stdio.h>
#include <string.h>         // ����memset����ͷ�ļ�
#include <ctype.h>          // ����isalpha����ͷ�ļ�
#include <stdlib.h>         // ����strtof����ͷ�ļ�
#include <stdio.h>          // ����printf����ͷ�ļ�
#include "ws2812.h"
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
volatile float X_IN = 0,Y_IN = 0;
uint8_t uart_count;
uint8_t rx_buffer[50];
uint8_t test_flag;
volatile float tar_buffer[20];
volatile uint8_t shijiao_rx_data[5];
volatile uint8_t shijiao_display_data;


uint8_t rx_cmd[50];
volatile uint8_t task_cmd;
float first_point_one[2];
	// ...existing code...
#define SECOND_POINT_NUM 8
#define POS_ERR_TH 0.002f // ����������ֵ������ʵ���������?
float second_points[SECOND_POINT_NUM][2];
uint8_t second_point_index = 0;
uint8_t second_point_count = 0; // ��������¼�ѱ�ǵ���?
uint8_t enable_flag=1;

volatile float four_cur_pos[2],four_tar_pos[2],four_add_pos[2];

Pid_Controller_t pid_x,pid_y;


typedef enum {
    FIND_HEADER,
    RECEIVE_DATA,
    DATA_READY
} Receive_State;
#define rx_buffer_num 50
#define rx_task_buffer_num 50
Receive_State UartRxState = FIND_HEADER;

float values[6];

uint8_t rx_index = 0;
uint8_t rx_buffer[rx_buffer_num];
uint8_t parse_buffer[rx_buffer_num];

uint8_t rx_task_index = 0;
uint8_t rx_task_buffer[rx_task_buffer_num] = {0};


uint8_t r = 0;
uint8_t g = 0;
uint8_t b = 0;
// ...existing code...
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
extern uint8_t CDC_Transmit_HS(uint8_t* Buf, uint16_t Len);
void robot_arm(float X, float Y);
void Uart_Data_Process(void);
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
  MX_SPI1_Init();
  MX_UART7_Init();
  MX_USART10_UART_Init();
  MX_TIM12_Init();
  MX_SPI6_Init();
  /* USER CODE BEGIN 2 */
	delay_init(480);//���ӳ���Ῠ��?
	can_bsp_init();//can�˲����Ϳ�������
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,1);//XT30+CAN �ɿؿ���1
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,1);//XT30+CAN �ɿؿ���2
	delay_ms(1000);
	
  mc_init(); //���ϵͳ��ʼ�����������е��


  //rx_cmd = 0;
  Pid_Init(&pid_x,0.00075,0,0.000,0,0.033f,3.14f);
  Pid_Init(&pid_y,0.00075,0,0.000,0,0.033f,3.14f);
  HAL_UART_Receive_IT(&huart7, rx_cmd, 1);
  HAL_UART_Receive_IT(&huart10, rx_buffer + rx_index, 1);
  
  HAL_TIM_Base_Start(&htim12);
  HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);
	TIM12->CCR2 = 00; 
	WS2812_Ctrl(128,128,0);
  //HAL_UART_Transmit(&huart10,(uint8_t *)"HELLO!",6,1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {  
    if (task_cmd)
    {
        switch (task_cmd)
        {
            case 0xf1: // ���ʧ��?
            {
                enable_flag = 0;
                motor_disable(&hfdcan1, 0);
                motor_disable(&hfdcan1, 1);
                break;
            }
    
            case 0x11: // ��һ�ʱ��?
            {
                first_point_one[0] = motor_pos[0];
                first_point_one[1] = motor_pos[1];
                // question_one_mark_point(rx_task_buffer);
                break;
            }
    
            case 0x12: // ��һ��ִ��
            {
                enable_flag = 1;
                motor_enable(&hfdcan1, 0);
                motor_enable(&hfdcan1, 1);
                pos_speed_ctrl(&hfdcan1, 0, first_point_one[0], 3);
                pos_speed_ctrl(&hfdcan1, 1, first_point_one[1], 3);
                // question_one_perform(rx_task_buffer);
                break;
            }
    
            case 0x21: // �ڶ��ʱ��?
            {
                // ��ǵ�ǰ�㵽����?
                second_points[second_point_index][0] = motor_pos[0];
                second_points[second_point_index][1] = motor_pos[1];
                second_point_index++;
                if (second_point_index >= SECOND_POINT_NUM)
                    second_point_index = 0; // �����ͷ���?
                if (second_point_count < SECOND_POINT_NUM)
                    second_point_count++; // ֻ��δ��ʱ����
                break;
            }
    
            case 0x22: // �ڶ���ִ��
            {
                enable_flag = 1;
                motor_enable(&hfdcan1, 0);
                motor_enable(&hfdcan1, 1);
    
                // ִֻ���ѱ�ǵĵ�?
                for (uint8_t i = 0; i < second_point_count; i++)
                {
                    pos_speed_ctrl(&hfdcan1, 0, second_points[i][0], 3);
                    pos_speed_ctrl(&hfdcan1, 1, second_points[i][1], 3);
    
                    // �ȴ�����Ŀ���?
                    while (1)
                    {
                        float err0 = fabs(motor_pos[0] - second_points[i][0]);
                        float err1 = fabs(motor_pos[1] - second_points[i][1]);
                        if (err0 < POS_ERR_TH && err1 < POS_ERR_TH)
                            break;
                        delay_ms(10); // ÿ��10ms���һ��?
                    }
                }
                break;
            }

            case 0x41:
            {
              
              break;
            }
    
            // ...existing code...
    
            default:
            {
                break;
            }
        }
        task_cmd = 0;
    }
    Uart_Data_Process();

    // ...existing code...
    // ...existing code...
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
void Uart_Data_Process(void)
{
    // ���ݽ���
    if (UartRxState == DATA_READY)
    {
        uint8_t parse_len = rx_index + 1;
        memcpy(parse_buffer, rx_buffer, parse_len);

        char* ptr = (char*)parse_buffer;
        char* end;
        int i = 0;

        // ������ͷ�� '#'
        char* temp_ptr = strchr(ptr, '#');
        if (temp_ptr != NULL)
        {
            ptr = temp_ptr + 1;
        }
        else
        {
            // δ�ҵ� '#' ����µĴ���
            rx_index = 0;
            memset(rx_buffer, 0, sizeof(rx_buffer));
            memset(parse_buffer, 0, sizeof(parse_buffer));
            HAL_UART_Receive_IT(&huart7, rx_buffer + rx_index, 1);  // ȷ���ٴ����ý����ж�

        }

        if (parse_buffer[parse_len - 1] != ';')
        {
            return;
        }

        // ����������
        while (*ptr && i < 2)
        {
            while (isalpha(*ptr)) ptr++;  // ������ĸ

            values[i] = strtof(ptr, &end);
            if (ptr == end) break;

            ptr = end;
            i++;

            // ��������
            while (*ptr == ',') ptr++;
        }

        // ��������������
        // for (int j = 0; j < i; j++)
        // {
        //     //printf("Value %c: %f\n", 'A' + j, values[j]);
        // }

        // ����״̬
        rx_index = 0;
        memset(rx_buffer, 0, sizeof(rx_buffer));

        UartRxState = FIND_HEADER;
		HAL_UART_Receive_IT(&huart7, rx_buffer, 1);


        four_add_pos[0] = Pid_Cal(&pid_x,values[0]);
        four_add_pos[1] = Pid_Cal(&pid_y,values[1]);

        four_cur_pos[0] = motor_pos[0];
        four_cur_pos[1] = motor_pos[1];

        four_tar_pos[0] = -four_add_pos[0] + four_cur_pos[0];
        four_tar_pos[1] = -four_add_pos[1] + four_cur_pos[1];

        pos_speed_ctrl(&hfdcan1, 0, four_tar_pos[0], 3);
        pos_speed_ctrl(&hfdcan1, 1, four_tar_pos[1], 3);

    }

  

}


void UART7_IRQHandler(void)
{
  /* USER CODE BEGIN UART7_IRQn 0 */
  /* USER CODE END UART7_IRQn 0 */
	HAL_UART_IRQHandler(&huart7);
  /* USER CODE BEGIN UART7_IRQn 1 */
	task_cmd = rx_cmd[0];
	HAL_UART_Receive_IT(&huart7, rx_cmd, 1);
  /* USER CODE END UART7_IRQn 1 */
}


void USART10_IRQHandler(void)
{
  /* USER CODE BEGIN USART10_IRQn 0 */
  
  /* USER CODE END USART10_IRQn 0 */
  HAL_UART_IRQHandler(&huart10);
  /* USER CODE BEGIN USART10_IRQn 1 */
    switch (UartRxState)
  {
      case FIND_HEADER:
          if (rx_buffer[rx_index] == '#')
          {
              UartRxState = RECEIVE_DATA;
              rx_index += 1;

          }
          break;
      case RECEIVE_DATA:
          if (rx_buffer[rx_index] == ';')
          {
              UartRxState = DATA_READY;

          }
          else
          {
              rx_index++;
              if (rx_index > sizeof(rx_buffer))
              {
                  rx_index = 0;
                  return;
              }
          }
          break;
      case DATA_READY:
          break;
      
  }
  HAL_UART_Receive_IT(&huart10, rx_buffer + rx_index, 1);
  /* USER CODE END USART10_IRQn 1 */
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
  if(huart == &huart10)
  {

  }
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
