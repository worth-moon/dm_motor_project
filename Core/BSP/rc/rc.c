#include "rc.h"
#include "fast_sin.h"
#include "debug_printf.h"
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
//坐标系参考倾时肖声，Z为指定高度，除底部x轴电机外，其他电机的零点均是机械臂在竖直状态下的位置

// 全局变量定义 - 4轴配置 (1个x轴 + 4个y轴电机)
// 关节角度变量
float J1 = 0.0f, J2 = 0.0f, J3 = 0.0f, J4 = 0.0f, J5 = 0.0f;
float Z = 0.0f;

// 机器人参数
float bu_chang = 0.1f;             // 角度搜索步长 (约5.7度)
float tolerance = 0.1f;           // 位置误差容忍度
float P = 5.0f;                    // 基座偏移参数 - 假定为机械臂底部，抓不到的一个圆形直径

// DH参数 - 连杆长度
float A1 = 23.0f, A2 = 12.0f, A3 = 12.0f, A4 = 12.0f, A5 = 12.0f, A6 = 9.5f; 
// 假设新增的两个关节也是12.0的长度

// 计算中间变量
float high = 0.0f, len = 0.0f;
float cur_high = 0.0f, cur_len = 0.0f;

// 三角函数值
float s2 = 0.0f, c2 = 0.0f;
float s23 = 0.0f, c23 = 0.0f;
float s234 = 0.0f, c234 = 0.0f;
float s2345 = 0.0f, c2345 = 0.0f;

// 统计计数器
uint64_t total_iterations = 0;     // 总循环次数
uint32_t error_314_count = 0;      // 超过3.14约束的次数
uint32_t youxiao_count = 0;        // 有效计算次数
uint32_t count = 0;                // 满足条件的解个数

// 时间计数
uint32_t forward_count = 0;        // 开始时间
uint32_t backward_count = 0;       // 结束时间

// 结果存储（4个y轴电机）
float test_motor1 = 0.0f;          // 测试电机1角度 (x轴)
float test_motor2 = 0.0f;          // 测试电机2角度 (y轴)
float test_motor3 = 0.0f;          // 测试电机3角度 (y轴)
float test_motor4 = 0.0f;          // 测试电机4角度 (y轴)
float test_motor5 = 0.0f;          // 测试电机5角度 (y轴)

// 方法1：使用goto语句（推荐，简洁直接）
void robot_arm(float X, float Y) 
{ 
    my_printf("start to arm counter!\r\n");
    
    Z = 15;
    J1 = atanf((P + Y) / X);
    if (J1 < 0)
    {
        J1 = fabsf(J1) + 3.14f;
    }
    
    high = Z + 3;
    len = sqrtf(X * X + (P + Y) * (P + Y));
    
    forward_count = HAL_GetTick();
    
    for (J2 = -1.57; J2 < 1.57; J2 += bu_chang) // -pi/4~pi/4
    {
        for (J3 = 0; J3 < 3.14; J3 += bu_chang)
        {
            for (J4 = 0; J4 < 3.14; J4 += bu_chang)
            {
                for (J5 = 0; J5 < 3.14; J5 += bu_chang)
                {
                    total_iterations++; // 记录总循环次数
                    
                    // 检查 J2 + J3 + J4 + J5 是否超过 3.14 (修改为4个电机)
                    if ((J2 + J3 + J4 + J5) > 3.14)
                    {
                        error_314_count++; // 记录错误次数
                        continue; // 跳过当前组合，不计入 count
                    }
                    youxiao_count++;
                    
                    // 使用fast_sin_cos函数计算三角函数值（4个y轴电机）
                    fast_sin_cos(J2, &s2, &c2);
                    fast_sin_cos(J2 + J3, &s23, &c23);
                    fast_sin_cos(J2 + J3 + J4, &s234, &c234);
                    fast_sin_cos(J2 + J3 + J4 + J5, &s2345, &c2345);
                    
                    // 正运动学计算（去掉A6项）
                    cur_high = A1 + A2 * c2 + A3 * c23 + A4 * c234 + A5 * c2345;
                    cur_len = A2 * s2 + A3 * s23 + A4 * s234 + A5 * s2345;
                    
                    if (fabsf(cur_high - high) < tolerance && fabsf(len - cur_len) < tolerance)
                    {
                        count++;
                        test_motor1 = J1;   // x轴电机
                        test_motor2 = J2;   // y轴电机1
                        test_motor3 = J3;   // y轴电机2
                        test_motor4 = J4;   // y轴电机3
                        test_motor5 = J5;   // y轴电机4
                        my_printf("Solution found - J1:%.3f, J2:%.3f, J3:%.3f, J4:%.3f, J5:%.3f\n", 
                                 J1, J2, J3, J4, J5);
                        
                        // 找到解后立即退出所有循环
                        goto solution_found;
                    }
                }
            }
        }
    }
    
solution_found:
    backward_count = HAL_GetTick();
    uint32_t execution_time = backward_count - forward_count;
    
    if (count > 0) {
        my_printf("Solution found successfully!\n");
    } else {
        my_printf("No solution found within tolerance.\n");
    }
    
    my_printf("Total counter: %llu\n", total_iterations);
    my_printf("total time: %llu ms\n", execution_time);
    
    total_iterations = 0;
    count = 0;
}

volatile float t_output[6],xita_jxb[6];
volatile float alpha[6];
extern float jxb_motor_pos[7];
float debug_fuhao = 1,debug_fuhao2 = -1;
extern float motor_pos[MAX_MOTOR_COUNT];
float m_link = 0.149f, g_count = 9.8f, lc = 0.06f, l_count = 0.12f, m_motor = 0.345f; 
void Gravity_Compensation(void)
{
    //------------------- 1. 修正角度 -------------------
    // 将原始角度 xita 修正为机械臂实际关节角度 xita_jxb
    xita_jxb[0] = motor_pos[0] + 3.14f/2;   // 关节1角度加90度
    xita_jxb[1] = - motor_pos[1];           // 关节2角度取反
    xita_jxb[2] = motor_pos[2];             // 关节3角度不变
    
    //------------------- 2. 计算各关节的绝对角度 alpha -------------------
    // alpha[i] 表示第i个关节的绝对角度（从基座到该关节的总旋转角度）
    alpha[0] = xita_jxb[0];
    alpha[1] = xita_jxb[0] + xita_jxb[1];
    alpha[2] = xita_jxb[0] + xita_jxb[1] + xita_jxb[2];
    
    //------------------- 3. 定义机械参数 -------------------
//    float m_link = 0.149f, g_count = 10.0f, lc = 0.06f, l_count = 0.12f, m_motor = 0.345f; 
    // m_link: 单根连杆质量
    // m_motor: 单个电机质量
    // lc: 连杆质心到关节距离
    // l_count: 连杆长度
    // g_count: 重力加速度
    
    //------------------- 4. 计算各关节所需力矩 -------------------
    
    // 关节1：承载整个系统的重力分量
    t_output[0] = g_count * (
      m_link * lc * cosf(alpha[0]) + 
      m_motor * l_count * cosf(alpha[0]) + 
      m_link * (l_count * cosf(alpha[0]) + lc * cosf(alpha[1])) +
      m_motor * (l_count * cosf(alpha[0]) + l_count * cosf(alpha[1])) +
      m_link * (l_count * cosf(alpha[0]) + l_count * cosf(alpha[1]) + lc * cosf(alpha[2]))
    );
    
    // 关节2：承载连杆2、电机3、连杆3的重力分量
    t_output[1] = g_count * (
      m_link * lc * cosf(alpha[1]) +
      m_motor * l_count * cosf(alpha[1]) +
      m_link * (l_count * cosf(alpha[1]) + lc * cosf(alpha[2]))
    ) * (-1.0f); // 注意方向为负
    
    // 关节3：仅承载连杆3的重力分量
    t_output[2] = g_count * m_link * lc * cosf(alpha[2]);
    
    //------------------- 5. 发送力矩指令到各关节电机 -------------------
    
    // 依次给3个关节电机发送力矩控制指令
    mit_ctrl(&hfdcan1,0,0,0,0,0,t_output[0]);
    mit_ctrl(&hfdcan1,1,0,0,0,0,t_output[1]);
    mit_ctrl(&hfdcan1,2,0,0,0,0,t_output[2]);
    // HAL_Delay(1);    
}