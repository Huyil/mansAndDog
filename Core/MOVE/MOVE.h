//
// Created by mzx on 2021/8/1.
//

#ifndef CPROJECT_MOVE_H
#define CPROJECT_MOVE_H

#include "main.h"
#include "string.h"
#include "gpio.h"
#include "usart.h"
#include "stdio.h"
#define start_flag 0x12
#define stop_flag 0x13
#define reset_flag 0x14
struct Move_cfg{
    float P;
    float I;
    float D;
    int32_t target;//目标
    int32_t motor_speed;//电机速度
    int32_t mg995_angle;//电机速度
    uint32_t period;//周期
    uint8_t power_set;//总开关
    uint8_t mg995_set;
    uint8_t motor_set;
    uint8_t err_log;
};
extern float PID_Kp,PID_Kd,PID_Ki;
struct Move_cfg fire_Set(uint8_t data_buff[],struct Move_cfg cfg);
void Ble_send(struct Move_cfg cfg);
void fire_send(struct Move_cfg cfg,uint8_t ch);
struct Move_cfg motor(struct Move_cfg cfg);//电机方向
struct Move_cfg Move_Set(uint8_t data_buff[],struct Move_cfg);
int motor_PID(int Expect_Speed,int Actual_Speed);

#endif //CPROJECT_MOVE_H
