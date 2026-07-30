/*
 * @Author: Zixuan Wang tanng@163.com
 * @Date: 2026-05-22 00:23:36
 * @LastEditors: Zixuan Wang tanng@163.com
 * @LastEditTime: 2026-07-30 00:09:40
 * @FilePath: \mdkd:\Desktop\电控\raicom_SteeringWheel\driver\inc\Driver_MyChassis.h
 * @Description: 
 * 
 * Copyright (c) 2026 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef __MY_CHASSIS_H
#define __MY_CHASSIS_H
#include "stm32f4xx.h"

#define Wheel_radius 0.076f
#define Chassis_L 0.365f
#define Chassis_W 0.385f

typedef struct{
    MyMotor_Type *LF;
    MyMotor_Type *RF;
    MyMotor_Type *LB;
    MyMotor_Type *RB;
    uint32_t tx_std_id;     //CAN报头
}Chassis_3508_Type_Collection;

typedef struct{
    MyMotor_Type *LF;
    MyMotor_Type *RF;
    MyMotor_Type *LB;
    MyMotor_Type *RB;
    uint32_t tx_std_id;     //CAN报头
}Chassis_6020_Type_Collection;

typedef struct{
    CAN_TypeDef *CANx;
    Chassis_3508_Type_Collection *Chassis_3508_Type_Structure;
    Chassis_6020_Type_Collection *Chassis_6020_Type_Structure;
}Chassis_Type;


void SendChassis_ByRPM(Chassis_3508_Type_Collection *motor_collect,CAN_TypeDef *CANx, int16_t id, int16_t i_201, int16_t i_202, int16_t i_203, int16_t i_204);
void Chassis_Calculate(Chassis_Type *chassis,float Vx,float Vy,float AngularVelocity);
void Chassis_Send(Chassis_Type *chassis);
#endif
