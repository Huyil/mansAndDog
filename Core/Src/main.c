/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
#include "../MOVE/MOVE.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE_MAX 255 //串口最大接收长度

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t receive_buff[BUFFER_SIZE_MAX];
uint8_t mode = 0;//接收模式(手机1或者野火0)
uint8_t data_buff[6];
uint8_t fire_buff[32];
float PID_Kp = 0,PID_Kd = 0,PID_Ki = 0;
struct  Move_cfg move_cfg;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//uint32_t adc_ch1[3];
//uint16_t adc_buff[3];

uint8_t MotorVal;  //电机速度
uint8_t Mg995Val;  //舵机位置
int MotorSpeed;  // 电机当前速度数值，从编码器中获取
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void Servo_init()
//{
//
//}

void mg995_Control(uint16_t val)
{

    uint16_t temp;
    temp = 501 +val*20;//500~2500 ~= 2~84
    if(val>=0)
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, temp);
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  MX_DMA_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
    uint8_t But_flag = 0;

    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); // 使能串口空闲中断
    HAL_UART_Receive_DMA(&huart1, (uint8_t *) receive_buff, BUFFER_SIZE_MAX);//开始DMA传输 每次255字节数据

    //HAL_ADC_Start_DMA(&hadc1,adc_ch1,3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);        // 开启PWM 1         使用APB2(168M)
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);        // 开启PWM 2         使用APB2(168M)
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);   //  开启编码器A        使用APB1(84M)
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);   // 开启编码器B         使用APB1(84M)
    HAL_TIM_Base_Start_IT(&htim2);                  // 使能定时器2中断      使用APB1(84M)
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);        //使能定时器3中断       使用APB1(84M)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 50);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
//    HAL_ADC_Start(&hadc1);
//    HAL_ADC_PollForConversion(&hadc1,50);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    MotorVal = 50;
    mg995_Control(50);
    move_cfg.err_log = 255;
    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if(mode)/*mode == 1     手机端*/
    {
        move_cfg = Move_Set(data_buff, move_cfg);
        if(move_cfg.power_set)
            if (!move_cfg.err_log) //如果正常
            {
                if (move_cfg.motor_set != MotorVal)
                    MotorVal = move_cfg.motor_set;
                if (move_cfg.mg995_set != Mg995Val) {
                    Mg995Val = move_cfg.mg995_set;
                    mg995_Control(Mg995Val);
                }
            }
        Ble_send(move_cfg);
    }//mode==0,野火端
    else{
        move_cfg = fire_Set(fire_buff,move_cfg);
        if(!move_cfg.err_log)//如果正常
        {
            if (move_cfg.motor_set != MotorVal)
                MotorVal = move_cfg.motor_set;
            if (move_cfg.mg995_set != Mg995Val) {
                Mg995Val = move_cfg.mg995_set;
                mg995_Control(Mg995Val);
            }
        }
        fire_send(move_cfg,1);
    }

    //else if(1);//停止调试
//    else switch (move_cfg.err_log) {
//            case 1: printf("无数据\\n");break;
//            case 2: printf("未选择设备\n");break;
//        }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void USAR_UART_IDLECallback(UART_HandleTypeDef *huart)
{
    HAL_UART_DMAStop(&huart1);//停止本次DMA传输
    uint8_t data_length = BUFFER_SIZE_MAX - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);   //计算接收到的数据长度
    uint8_t Ble_len = 23;
    for (int i = 0; i < data_length; i++)
    {
        //手机蓝牙调试器接收模式
        if((data_length >= Ble_len-1) && (receive_buff[i] == 0x5A)){
            if( (receive_buff[i+1-Ble_len]) == 0xA5){
                uint8_t jiaoyan = 0;//校验
                for (int j = i-2; j > i+1-Ble_len ; j--) {
                    jiaoyan += receive_buff[j];
                }
                if(receive_buff[i-1] == jiaoyan)
                {
                    for (int j = 0; j < Ble_len; j++){
                        data_buff[j] = receive_buff[j];
                    }
                    mode = 1;
                }
            }
        }
        //野火上位机接收模式
        if(receive_buff[i]==0x53 && receive_buff[i+1]==0x5A)
            //if(receive_buff[i+2]==0x48 && receive_buff[i+3]==0x59)
            {
                uint8_t jiaoyan = 0x4E;//校验
                ////接收大小////
                uint32_t size = data_length;
                memcpy(&size,&receive_buff[5],4);
                ////接收大小////
                for (int j = 4; j < size-1; j++) {
                    jiaoyan += receive_buff[j];
                }
                if(jiaoyan == receive_buff[size-1]){
                    for (int j = i; j < size+i; j++) {
                        fire_buff[j] = receive_buff[j];
                    }
                }
                mode = 0;
            }
    }
    HAL_UART_Transmit(&huart1,fire_buff,data_length,16);
    memset(receive_buff,0,BUFFER_SIZE_MAX);
    HAL_UART_Receive_DMA(&huart1, (uint8_t *)receive_buff, 255);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    static unsigned char i = 0;
    if (htim == (&htim2))
    {
        MotorSpeed = (int16_t) __HAL_TIM_GET_COUNTER(&htim4);
        //MotorSpeed = __HAL_TIM_GET_FLAG(&htim4,1);
        // TIM4计数器获得电机脉冲，该电机在10ms采样的脉冲/18则为实际
        __HAL_TIM_SET_COUNTER(&htim4, 0);  // 计数器清零

        //2.将占空比导入至电机控制函数

        if (MotorVal > 99)
            MotorVal = 100;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, MotorVal);
        MotorVal = MotorVal; // 占空比（按最高100%计算，串口打印）
        i++;
        if (i > 100)
        {
            HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
            printf("\nEncoder = %d\tmoto = %d%c \tmode = %d\n", MotorSpeed, MotorVal, '%',mode);
            for (int j = 0; j < 11; j++) {
                printf("%x ",fire_buff[j]);
            }
            printf("\n");
            printf("P = %f\n",move_cfg.P);
            printf("I = %f\n",move_cfg.I);
            printf("D = %f\n",move_cfg.D);
            printf("target = %ld\n",move_cfg.target              );  /*目标*/
            printf("motor_speed = %ld\n",move_cfg.motor_speed    );  /*电机速度*/
            printf("mg995_angle = %ld\n",move_cfg.mg995_angle    );  /*电机速度*/
            printf("period = %ld\n",move_cfg.period              );  /*周期*/
            printf("power_set = %d\n",move_cfg.power_set        );  /*总开关*/
            printf("mg995_set = %d\n",move_cfg.mg995_set);
            printf("motor_set = %d\n",move_cfg.motor_set);
            printf("err_log = %d\n",move_cfg.err_log);
            i = 0;
        }
    }
}

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
//{
//    if(hadc == (&hadc1))
//    {
//        HAL_ADC_Stop_DMA(&hadc1);
//        adc_buff[0] = adc_ch1[0];
//        adc_buff[1] = adc_ch1[1];
//        adc_buff[2] = adc_ch1[2];
//        HAL_ADC_Start_DMA(&hadc1,adc_ch1,3);
//    }
//}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
