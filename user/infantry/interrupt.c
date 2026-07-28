/**
 * @brief  中断服务函数根据地
 */

#include "handle.h"
// EXTI9_5 陀螺仪中断
void EXTI9_5_IRQHandler(void) {
    uint8_t suc;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (EXTI_GetITStatus(EXTI_Line8) != RESET) {
        EXTI_ClearFlag(EXTI_Line8);
        EXTI_ClearITPendingBit(EXTI_Line8);
        xSemaphoreGiveFromISR(ImuDataReady, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// DBus空闲中断(USART1)
void USART1_IRQHandler(void) {
    uint8_t UARTtemp;

    UARTtemp = USART1->DR;
    UARTtemp = USART1->SR;

    DMA_Cmd(DMA2_Stream2, DISABLE);

    // disabe DMA
    DMA_Disable(USART1_Rx);

    // 数据量正确
    if (DMA_Get_Stream(USART1_Rx)->NDTR == DBUS_BACK_LENGTH) {
        DBus_Update(&remoteData, &keyboardData, &mouseData, remoteBuffer); //解码
    }

    // enable DMA
    DMA_Enable(USART1_Rx, DBUS_LENGTH + DBUS_BACK_LENGTH);
}

/**
 * @brief USART3 串口中断
 * @note  视觉系统读取
 */
void USART3_IRQHandler(void) {
    Bridge_Receive_USART(&BridgeData, USART_BRIDGE, 3);
}

/**
 * @brief USART6 串口中断
 * @note  裁判系统读取
 */
void USART6_IRQHandler(void) {
        Bridge_Receive_USART(&BridgeData, USART_BRIDGE, 6);
}

/**
 * @brief UART7 串口中断
 */
void UART7_IRQHandler(void) {
    Bridge_Receive_USART(&BridgeData, USART_BRIDGE, 7);
}

/**
 * @brief UART8 串口中断
 */
void UART8_IRQHandler(void) {
   Bridge_Receive_USART(&BridgeData, USART_BRIDGE, 8);
}

// CAN1数据接收中断服务函数
void CAN1_RX0_IRQHandler(void) {
    // Bridge_Receive_CAN(&BridgeData, CAN1_BRIDGE);
    CanRxMsg rx_message;

    // 检查是否收到数据
    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET) {
        // 读取接收到的数据
        CAN_Receive(CAN1, CAN_FIFO0, &rx_message); 
        // 根据 ID 进行处理
        switch (rx_message.StdId) {
            case 0x201:
                // 处理电机 1 数据
                Motor_decode_data(&Motor_3508_LF,rx_message.Data);
                //Update_3508_Continuous_Angle(&Motor_3508_LF);
                break;
            case 0x202:
                // 处理电机 2 数据
                Motor_decode_data(&Motor_3508_RF,rx_message.Data);
                //Update_3508_Continuous_Angle(&Motor_3508_RF);
                break;
            case 0x203:
                //电机3
                Motor_decode_data(&Motor_3508_LB,rx_message.Data);
                //Update_3508_Continuous_Angle(&Motor_3508_LB);
                break;
            case 0x204:
                //电机4
                Motor_decode_data(&Motor_3508_RB,rx_message.Data);
                //Update_3508_Continuous_Angle(&Motor_3508_RB);
                break;
            case 0x205:
                // Motor_decode_data(&Motor_3508_Gantry_Crane_X1,rx_message.Data);
                // Update_3508_Continuous_Angle(&Motor_3508_Gantry_Crane_X1);
                break;
            case 0x206:
                // Motor_decode_data(&Motor_3508_Gantry_Crane_Y1,rx_message.Data);
                // Update_3508_Continuous_Angle(&Motor_3508_Gantry_Crane_Y1);
                break;
            case 0x207:
                // Motor_decode_data(&Motor_3508_Gantry_Crane_Y2,rx_message.Data);
                // Update_3508_Continuous_Angle(&Motor_3508_Gantry_Crane_Y2);
                break;
            case 0x208:
                break;
            case 0x209:
                break;
            case 0x20A:
                break;
            case 0x20B:
                break;
            default:
                break;
        }
        
        // 清除中断标志位
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    }
}

// CAN2数据接收中断服务函数
void CAN2_RX0_IRQHandler(void) {
    Bridge_Receive_CAN(&BridgeData, CAN2_BRIDGE);
}

// TIM2 高频计数器
extern volatile uint32_t ulHighFrequencyTimerTicks;

void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        //ulHighFrequencyTimerTicks++;

        //底盘测试
        //后期要加上死区
        //SendChassis_ByRPM(&MyMotor_3508_Collection,CAN1,0x200,usart1_data_decoded.r1y/50,0,0,0);
        //Chassis_Calculate(&MyMotor_3508_Collection,CAN1,0x200,(float)usart1_data_decoded.r1y/1000.0f,(float)usart1_data_decoded.r1x/1000.0f,(float)usart1_data_decoded.r2x/330.0f,Chassis_L,Chassis_W);
        
        //龙门架测试
        //SendCrane_ByRPM(&Motor_3508_Gantry_Crane_Collection,CAN1,0x1FF,usart1_data_decoded.r1y/50,0,0);
        
        //舵机控制测试
        //SetServoByController(usart1_data_decoded.s1);
        
        //遥控器函数(运动控制)
        MyController_Move(&usart1_data_decoded);

        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    }
}

void TIM3_IRQHandler(void) {
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
        
        //遥控器函数(保持角度)
        MyController_Stay(&usart1_data_decoded);

        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        TIM_ClearFlag(TIM3, TIM_FLAG_Update);
    }
}

/**
 * @brief  This function handles NMI exception.
 * @param  None
 * @return None
 */

void NMI_Handler(void) {
    while (1) {
    }
}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @return None
 */
void hardfault_c(uint32_t *sp);
	
__ASM void HardFault_Handler(void)
{
		IMPORT hardfault_c;
	
    TST     LR, #4
    ITE     EQ
    MRSEQ   R0, MSP
    MRSNE   R0, PSP
    B       hardfault_c
}

void hardfault_c(uint32_t *sp)
{
    volatile uint32_t pc  = sp[6];
    volatile uint32_t lr  = sp[5];
    volatile uint32_t cfsr  = SCB->CFSR;
    volatile uint32_t hfsr  = SCB->HFSR;
    volatile uint32_t mmfar = SCB->MMFAR;
    volatile uint32_t bfar  = SCB->BFAR;

    __BKPT(0);
    while (1) {}
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @return None
 */
void MemManage_Handler(void) {
     while (1) {
    }
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @return None
 */
void BusFault_Handler(void) {
   while (1) {
    }
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @return None
 */
void UsageFault_Handler(void) {
    while (1) {
    }
}

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @return None
 */
void DebugMon_Handler(void) {
    while (1) {
    }
}

// /**
//  * @brief  This function handles SVCall exception.
//  * @param  None
//  * @return None
//  */
// void SVC_Handler(void) {
//     //while(1){}
// }

// /**
//  * @brief  This function handles PendSVC exception.
//  * @param  None
//  * @return None
//  */
// void PendSV_Handler(void) {
//    //while(1){}
// }

// /**
//  * @brief  This function handles SysTick Handler.
//  * @param  None
//  * @return None
//  */
// void SysTick_Handler(void) {
//      //while(1){}
// }
