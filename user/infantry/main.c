#define __HANDLE_GLOBALS

#include "config.h"
#include "macro.h"
#include "handle.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tasks.h"

int main(void) {
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
    //初始化Delay和I2C
    My_delay_init();
    MyI2C_Init();

    //遥控器USART+DMA转运, 
    //USART在中断里面处理解码，解码数据在结构体usart1_data_decoded中
    MY_USART_Init();
    MyDMA_Init((uint32_t)&(USART1->DR),(uint32_t)usart1_raw_data,18);
	
    //初始化夹爪舵机结构体，PWM初始化
    //BSP_PWM_Init内TIM_OCMode_PWM2 修改为 TIM_OCMode_PWM1
    BSP_PWM_Set_Port(&PWM_Holding_Jaw_Servo,PWM_PD12);
    BSP_PWM_Init(&PWM_Holding_Jaw_Servo,1800,1000,TIM_OCPolarity_High);

    //初始化底盘3508电机结构体
    MyMotor_Init(&Motor_3508_LF,3,IsPositive_False,19,&Motor_3508_LF_PID,&Motor_3508_LF_Position_PID,0);
    MyMotor_Init(&Motor_3508_RF,2,IsPositive_True,19,&Motor_3508_RF_PID,&Motor_3508_RF_Position_PID,0);
    MyMotor_Init(&Motor_3508_LB,4,IsPositive_False,19,&Motor_3508_LB_PID,&Motor_3508_LB_Position_PID,0);
    MyMotor_Init(&Motor_3508_RB,1,IsPositive_True,19,&Motor_3508_RB_PID,&Motor_3508_RB_Position_PID,0);

    //初始化底盘6020电机结构体
    MyMotor_Init(&Motor_6020_LF,4,IsPositive_True,1,&Motor_6020_LF_PID,&Motor_6020_LF_Position_PID,0);
    MyMotor_Init(&Motor_6020_RF,1,IsPositive_True,1,&Motor_6020_RF_PID,&Motor_6020_RF_Position_PID,0);
    MyMotor_Init(&Motor_6020_LB,3,IsPositive_True,1,&Motor_6020_LB_PID,&Motor_6020_LB_Position_PID,0);
    MyMotor_Init(&Motor_6020_RB,2,IsPositive_True,1,&Motor_6020_RB_PID,&Motor_6020_RB_Position_PID,0);

    //初始化底盘MOTOR3508的pid
    PID_Init(Motor_3508_LF.Motor_PID,7.5,0.1,0.5,10000,3000);
    PID_Init(Motor_3508_RF.Motor_PID,7.5,0.1,0.5,10000,3000);
    PID_Init(Motor_3508_LB.Motor_PID,7.5,0.1,0.5,10000,3000);
    PID_Init(Motor_3508_RB.Motor_PID,7.5,0.1,0.5,10000,3000);

    //初始化底盘MOTOR6020的pid
    PID_Init(Motor_6020_LF.Motor_PID,15,0.1,0.5,10000,3000);
    PID_Init(Motor_6020_RF.Motor_PID,15,0.1,0.5,10000,3000);
    PID_Init(Motor_6020_LB.Motor_PID,15,0.1,0.5,10000,3000);
    PID_Init(Motor_6020_RB.Motor_PID,15,0.1,0.5,10000,3000);
    PID_Init(Motor_6020_LF.Motor_Position_PID,3,0.02,0.5,10000,3000);
    PID_Init(Motor_6020_RF.Motor_Position_PID,3,0.02,0.5,10000,3000);
    PID_Init(Motor_6020_LB.Motor_Position_PID,3,0.02,0.5,10000,3000);
    PID_Init(Motor_6020_RB.Motor_Position_PID,3,0.02,0.5,10000,3000);

    //底盘3508电机组结构体合集
    MyMotor_3508_Collection.LF=&Motor_3508_LF;
    MyMotor_3508_Collection.RF=&Motor_3508_RF;
    MyMotor_3508_Collection.LB=&Motor_3508_LB;
    MyMotor_3508_Collection.RB=&Motor_3508_RB;
    MyMotor_3508_Collection.tx_std_id=0x200;

    //底盘6020电机组结构体合集
    MyMotor_6020_Collection.LF=&Motor_6020_LF;
    MyMotor_6020_Collection.RF=&Motor_6020_RF;
    MyMotor_6020_Collection.LB=&Motor_6020_LB;
    MyMotor_6020_Collection.RB=&Motor_6020_RB;
    MyMotor_6020_Collection.tx_std_id=0x1FE;

    //完整底盘结构体
    Chassis_Structure.CANx=CAN1;
    Chassis_Structure.Chassis_3508_Type_Structure=&MyMotor_3508_Collection;
    Chassis_Structure.Chassis_6020_Type_Structure=&MyMotor_6020_Collection;

    //初始化CAN通信
    BSP_CAN_Init();

    //初始化TIM2，生成一个1KHZ的中断，主要业务写在这个中断中，要保证所有对象初始化完毕，故应最后开启
    //BSP_TIM2_Init经过修改，改了分频器，GPIOA初始化被注释
    BSP_TIM2_Init();

    //初始化TIM3，生成一个0.5KHZ的中断，保持电机和底盘角度写在这个中断中
    //若写在TIM2中会导致CAN发送不稳，导致电机过冲
    //BSP_TIM3_Init();
    while (1)
    {
       
    }
    
}
