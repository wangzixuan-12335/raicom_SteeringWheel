#include "stm32f4xx.h"
#include "handle.h"

/**
 * @description: 线速度转换角速度
 * @param {float} v 线速度
 * @param {float} r 轮子半径
 * @return {*}  角速度(rpm)
 */
float Velocity2RPM(float v,float r){
    float rpm=(v*60.0f)/(2*PI*r);
    return (float)rpm;
}

/**
 * @brief 将 [-PI, PI] 弧度转换为 0-8191 机械角
 * @param rad_angle 弧度 (范围 -PI 到 PI)
 * @return uint16_t 机械角 (范围 0 到 8191)
 */
uint16_t Rad_To_MechanicalAngle(float rad_angle) {
    // 1. 将弧度归一化到 [0, 2*PI] 范围
    float angle_0_to_2pi = rad_angle;
    while (angle_0_to_2pi < 0.0f) {
        angle_0_to_2pi += 2.0f * M_PI;
    }
    while (angle_0_to_2pi >= 2.0f * M_PI) {
        angle_0_to_2pi -= 2.0f * M_PI;
    }

    // 2. 映射到 0 - 8191
    uint16_t mechanical_angle = (uint16_t)(angle_0_to_2pi * (8192.0f / (2.0f * M_PI)));
    
    // 取模防止浮点误差导致溢出到 8192
    return mechanical_angle % 8192;
}

uint16_t EncoderToRealMechanicalAngle(uint16_t encoder_angle, uint16_t zero_offset)
{
    int32_t raw = (int32_t)encoder_angle - (int32_t)zero_offset;

    // 归一化到 0~8191
    raw %= 8192;
    if (raw < 0) {
        raw += 8192;
    }

    return (uint16_t)raw;
}

/**
 * @description: 
 * @param {Chassis_3508_Type_Collection} *motor //电机集合结构体
 * @param {CAN_TypeDef} *CANx 
 * @param {int16_t} id          //CAN报头
 * @param {int16_t} i_201       //左前 (Left Front) RPM
 * @param {int16_t} i_202       //右前 (Right Front) RPM
 * @param {int16_t} i_203       //左后 (Left Back) RPM
 * @param {int16_t} i_204       //右后 (Right Back) RPM
 * @return {*}
 */
void SendChassis_ByRPM(Chassis_3508_Type_Collection *motor_collect,CAN_TypeDef *CANx, int16_t id, int16_t i_201, int16_t i_202, int16_t i_203, int16_t i_204){
    //计算考虑正反转和减速比
    float output_LF = PID_Calculate(motor_collect->LF->Motor_PID, i_201*motor_collect->LF->ReductionRatio*motor_collect->LF->IsPositive, (float)motor_collect->LF->Rotor_Speed);
    float output_RF = PID_Calculate(motor_collect->RF->Motor_PID, i_202*motor_collect->RF->ReductionRatio*motor_collect->RF->IsPositive, (float)motor_collect->RF->Rotor_Speed);
    float output_LB = PID_Calculate(motor_collect->LB->Motor_PID, i_203*motor_collect->LB->ReductionRatio*motor_collect->LB->IsPositive, (float)motor_collect->LB->Rotor_Speed);
    float output_RB = PID_Calculate(motor_collect->RB->Motor_PID, i_204*motor_collect->RB->ReductionRatio*motor_collect->RB->IsPositive, (float)motor_collect->RB->Rotor_Speed);
    Can_Send(CANx,id,(int16_t)output_LF,(int16_t)output_RF,(int16_t)output_LB,(int16_t)output_RB);
}

/**
 * @description: 底盘正解算
 * @param {Chassis_Type} *chassis 电机集合结构体                            
 * @param {float} Vx    Vx速度(m/s)
 * @param {float} Vy    Vy速度(m/s)
 * @param {float} AngularVelocity  角速度rad/s 
 * @return {*}
 */
void Chassis_Calculate(Chassis_Type *chassis,float Vx,float Vy,float AngularVelocity){
    float L=Chassis_L;
    float W=Chassis_W;
    if(chassis==NULL){
        return;
    }
    float a = L / 2.0f; // 半轴距
    float b = W / 2.0f; // 半轮距

    /* 
     * 轮次对应笛卡尔象限：
     * [0]: 第一象限 (右上 / 前右,  x =  b, y =  a)
     * [1]: 第二象限 (左上 / 前左,  x = -b, y =  a)
     * [2]: 第三象限 (左下 / 后左,  x = -b, y = -a)
     * [3]: 第四象限 (右下 / 后右,  x =  b, y = -a)
     */
    const float x_pos[4] = {  b, -b, -b,  b };
    const float y_pos[4] = {  a,  a, -a, -a };

    for (int i = 0; i < 4; i++) {
        // 1. 速度分解：Vix = Vx - w * y_i, Viy = Vy + w * x_i
        float vix = Vx - AngularVelocity * y_pos[i];
        float viy = Vy + AngularVelocity * x_pos[i];

        float v_speed = sqrtf(vix * vix + viy * viy);    

        // 2. 计算基础角与转速
        float raw_angle = atan2f(viy, vix);
        float raw_rpm = Velocity2RPM(v_speed,Wheel_radius);
        uint16_t mec_angle = Rad_To_MechanicalAngle(raw_angle);
        int32_t real_angle;
        switch (i)
        {
        case 0:
            //右前轮
            real_angle = EncoderToRealMechanicalAngle(mec_angle,chassis->Chassis_6020_Type_Structure->RF->Zero_offset);

            chassis->Chassis_3508_Type_Structure->RF->Target_Speed=(int16_t)raw_rpm;
            chassis->Chassis_6020_Type_Structure->RF->target_pos=(int64_t)real_angle;
            break;
        case 1:
            //左前轮
            real_angle = EncoderToRealMechanicalAngle(mec_angle,chassis->Chassis_6020_Type_Structure->LF->Zero_offset);

            chassis->Chassis_3508_Type_Structure->LF->Target_Speed=(int16_t)raw_rpm;
            chassis->Chassis_6020_Type_Structure->LF->target_pos=(int64_t)real_angle;
            break;
        case 2:
            //左后轮
            real_angle = EncoderToRealMechanicalAngle(mec_angle,chassis->Chassis_6020_Type_Structure->LB->Zero_offset);

            chassis->Chassis_3508_Type_Structure->LB->Target_Speed=(int16_t)raw_rpm;
            chassis->Chassis_6020_Type_Structure->LB->target_pos=(int64_t)real_angle;
            break;
        case 3:
            //右后轮
            real_angle = EncoderToRealMechanicalAngle(mec_angle,chassis->Chassis_6020_Type_Structure->RB->Zero_offset);

            chassis->Chassis_3508_Type_Structure->RB->Target_Speed=(int16_t)raw_rpm;
            chassis->Chassis_6020_Type_Structure->RB->target_pos=(int64_t)real_angle;
            break;
        default:
            break;
        }
    }
}

void Chassis_Send(Chassis_Type *chassis){
    int16_t output_3508[4];
    int16_t output_6020[4];
    int32_t angle_error;
    MyMotor_Type* Motor_3508[4]={
        chassis->Chassis_3508_Type_Structure->LF,
        chassis->Chassis_3508_Type_Structure->RF,
        chassis->Chassis_3508_Type_Structure->LB,
        chassis->Chassis_3508_Type_Structure->RB};
    MyMotor_Type* Motor_6020[4]={
        chassis->Chassis_6020_Type_Structure->LF,
        chassis->Chassis_6020_Type_Structure->RF,
        chassis->Chassis_6020_Type_Structure->LB,
        chassis->Chassis_6020_Type_Structure->RB};
    for (uint8_t i = 0; i < 4; i++)
    {
        output_3508[Motor_3508[i]->motor_id-1]=PID_Calculate(Motor_3508[i]->Motor_PID, Motor_3508[i]->Target_Speed*Motor_3508[i]->ReductionRatio*Motor_3508[i]->IsPositive, (float)Motor_3508[i]->Rotor_Speed);
    }
    for (uint8_t i = 0; i < 4; i++)
    {
        angle_error=(int32_t)Motor_6020[i]->target_pos-(int32_t)Motor_6020[i]->Mechanical_Angle;
        if (angle_error > 4096) {
            angle_error -= 8192;
        } else if (angle_error < -4096) {
            angle_error += 8192;
        }
    }
    
    
    Can_Send(chassis->CANx,chassis->Chassis_3508_Type_Structure->tx_std_id,output_3508[0],output_3508[1],output_3508[2],output_3508[3]);
}