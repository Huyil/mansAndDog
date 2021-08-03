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
    int32_t target;//Ŀ��
    int32_t motor_speed;//����ٶ�
    int32_t mg995_angle;//����ٶ�
    uint32_t period;//����
    uint8_t power_set;//�ܿ���
    uint8_t mg995_set;
    uint8_t motor_set;
    uint8_t err_log;
};
extern float PID_Kp,PID_Kd,PID_Ki;
struct Move_cfg fire_Set(uint8_t data_buff[],struct Move_cfg cfg);
void Ble_send(struct Move_cfg cfg);
void fire_send(struct Move_cfg cfg,uint8_t ch);
struct Move_cfg motor(struct Move_cfg cfg);//�������
struct Move_cfg Move_Set(uint8_t data_buff[],struct Move_cfg);
int motor_PID(int Expect_Speed,int Actual_Speed);

#endif //CPROJECT_MOVE_H
