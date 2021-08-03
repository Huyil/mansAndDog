//code for GBK
// Created by Huyil on 2021/8/1.
//���ڽ����ж� 0xA5  0x01  0x20  0x40  0x61  0x5A
//����λ��6λ  ��ͷ  ѡ�豸 ����� �����  У��  ��β
//У��λΪ�м���λ����λ֮�͵ĺ��λ����01+20+40=61;
//���+0x80Ϊ��ת

#include "MOVE.h"

struct Move_cfg fire_Set(uint8_t data_buff[],struct Move_cfg cfg){
    uint8_t ch = 0x01;
    uint32_t size = 22;
    if(data_buff[0] == 0x53) {
        memcpy(data_buff + 8, &size, 4);
        for (int i = 5; i < size - 5; i++) {
            switch (data_buff[i]) {
                case 0x10:
                    memcpy(&cfg.P, data_buff + 1, 4);
                    memcpy(&cfg.I, data_buff + 5, 4);
                    memcpy(&cfg.D, data_buff + 9, 4);
                    i += 12;
                    break;
                case 0x11:
                    memcpy(&cfg.target, data_buff + 1, 8);
                    i += 8;
                    break;
                case 0x12:
                    HAL_GPIO_WritePin(CHA_GPIO_Port, CHA_Pin, GPIO_PIN_RESET);//��ת
                    HAL_GPIO_WritePin(CHB_GPIO_Port, CHB_Pin, GPIO_PIN_RESET);
                    cfg.err_log = 0;
                    motor(cfg);
                case 0x13:
                    cfg.power_set = 1;
                    cfg.err_log = 1;
                    break;
                case 0x14:
                    cfg.power_set = 0;
                    cfg.err_log = 2;
                    break;
                case 0x15:
                    memcpy(&cfg.period, data_buff + 1, 8);
                    i += 8;
                    break;
                default:
                    cfg.err_log = 101;
            }
        }
        cfg.err_log = 0;
    }
    memset(data_buff,0,data_buff[4]);
    return cfg;
}
void fire_send(struct Move_cfg cfg,uint8_t ch){
    uint8_t tx_buff[30]={0x53,0x5A,0x48,0x59,ch,0,0,0,0};
    uint32_t buff_lenght = 9;
    //��Դ״̬����
    if(cfg.power_set)
        tx_buff[buff_lenght++] = 0x04;
    else
        tx_buff[buff_lenght++] = 0x05;

    //���ڷ���
    tx_buff[buff_lenght++] = 0x06;
    memcpy(tx_buff+buff_lenght,&cfg.period,8);
    buff_lenght += 8;

    //ʵ��ֵ����
    tx_buff[buff_lenght++] = 0x02;
    memcpy(tx_buff+buff_lenght,&cfg.motor_speed,8);
    buff_lenght += 8;
    //У��
    buff_lenght++;//11
    memcpy(tx_buff+5,&buff_lenght,4);

    uint8_t jiaoyan = 0x4E;
    for (int i = 4; i < buff_lenght; i++) {
        jiaoyan += tx_buff[i];
    }
    tx_buff[buff_lenght-1] = jiaoyan;
    //����
    HAL_UART_Transmit(&huart1,tx_buff,buff_lenght,10);
}

struct Move_cfg Move_Set(uint8_t data_buff[5],struct Move_cfg cfg)
{
    cfg.err_log = 1;
    if(data_buff[0] == 0xA5)
    {
        switch (data_buff[1])
        {
            case 01://ֻ����ɻ�
                cfg.motor_set = data_buff[3];
                cfg = motor(cfg);
                break;
            case 02://ֻ����ɻ�
                cfg.mg995_set = data_buff[2];
                //cfg.err_log = 2;
                cfg.motor_set = 0;
                motor(cfg);
                break;
            case 03://���ɻ�
                cfg.mg995_set = data_buff[2];
                cfg.motor_set = data_buff[3];
                cfg = motor(cfg);
                break;
            default://�����ɻ�
                //cfg.err_log = 2;
                cfg.motor_set = 0;
                motor(cfg);
                //cfg.mg995_set = 0;
                //cfg.motor_set = 0;
        }
        cfg.power_set = data_buff[4];
        memcpy(&PID_Kp,&data_buff[5],4);
        memcpy(&PID_Ki,&data_buff[9],4);
        memcpy(&PID_Kd,&data_buff[13],4);
        memcpy(&cfg.target,&data_buff[17],4);
        memset(data_buff,0, strlen((char*)data_buff));
        cfg.err_log = 0;
    }
    return cfg;
}
void Ble_send(struct Move_cfg cfg){
    uint8_t tx_buff[11]={0xA5,0,0,0,0,0,0,0,0,0,0x5A};
    memcpy(tx_buff+1,&cfg.motor_speed,4);
    memcpy(tx_buff+5,&cfg.mg995_set,4);
    for (int i = 1; i < 10; i++) {
        tx_buff[10] +=tx_buff[i];
    }
    HAL_UART_Transmit(&huart1,tx_buff,11,10);
}
struct Move_cfg motor(struct Move_cfg cfg)//�������
{
    if(cfg.motor_set>0x80)//����
    {
        HAL_GPIO_WritePin(CHA_GPIO_Port, CHA_Pin, GPIO_PIN_RESET);//��ת
        HAL_GPIO_WritePin(CHB_GPIO_Port, CHB_Pin, GPIO_PIN_SET);
        cfg.motor_set = ~cfg.motor_set+1;//�ֻ����ո�����ʽ
        //cfg.motor_set = cfg.motor_set-0x80;//���Խ��ո�����ʽ
    }
    else if(cfg.motor_set>0)//����
    {
        HAL_GPIO_WritePin(CHA_GPIO_Port, CHA_Pin, GPIO_PIN_SET);//��ת
        HAL_GPIO_WritePin(CHB_GPIO_Port, CHB_Pin, GPIO_PIN_RESET);
    }
    else if(cfg.err_log!=2){
        HAL_GPIO_WritePin(CHA_GPIO_Port, CHA_Pin, GPIO_PIN_SET);//��ת����
        HAL_GPIO_WritePin(CHB_GPIO_Port, CHB_Pin, GPIO_PIN_SET);
    }else{
        HAL_GPIO_WritePin(CHA_GPIO_Port, CHA_Pin, GPIO_PIN_RESET);//��ת
        HAL_GPIO_WritePin(CHB_GPIO_Port, CHB_Pin, GPIO_PIN_RESET);
    }

}


int motor_PID(int Expect_Speed,int Actual_Speed)
{
    int PWM_out = 0,Err = 0;//�����PWMֵ�����
    float Err_Low_pass;
    static float Err_Low_pass_last = 0;
    static float Err_Integral = 0;//����
    Err = Expect_Speed - Actual_Speed;//�������
    Err_Low_pass = Err * 0.7 + Err_Low_pass_last *0.3;//����ͨ�˲�
    Err_Low_pass_last = Err_Low_pass;//������ε����
    Err_Integral += Err_Low_pass;//������
    if(Err_Integral > 1000)
        Err_Integral = 1000;//�����޷�
    PWM_out = PID_Kp * Err_Low_pass + PID_Kd * (Err_Low_pass - Err_Low_pass_last) + PID_Ki *  Err_Integral;//PID�㷨
    if(PWM_out > 100)
        PWM_out = 100;//�������PWM�޷�
    return PWM_out;
}
