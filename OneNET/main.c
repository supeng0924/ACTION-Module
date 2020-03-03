/**
  ******************************************************************************
  * @file    main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "misc.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "update.h"
#include "debug.h"
#include "string.h"
#include <stdlib.h>
#include "elmo.h"
#include "user.h"
#include "update.h"
#include "MotionCard.h"
#include "stm32f4xx.h"
#include "sim800c.h"
#include "threeWheelMove.h"
#include "stm32f4xx_it.h"
//正常模式
#define NORMAL_MODE 1
//采集模式
#define SAMPLING_MODE 2
#define MOTOR_USE 1
//中断数据存放队列  500防止来不及显示清空就溢出
extern uint8_t Receive_Buffer[500]; 

static int  mode = 1;

_Bool GyroscopeFlag = 0;


void Init(void)
{
	int i=0;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	

	//CAN1初始化，用于电机控制
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8, GPIO_Pin_9); 
	
	
	//陀螺仪接收串口
	GyroscopeUsartInit(115200);
	

	//蓝牙串口
//	WirelessBluetoothUsartInit(115200);
//	WirelessBluetoothUsartInit(9600);
	
	//温湿度传感器串口
	Usart6Init(9600);
	//SIM800串口
	uart_initPA2(115200);
	//调试串口
	uart_initpb10(115200);

	//蜂鸣器初始化
	MyBeefInit();	
	
	//行程开关
	MySwitchInit();
	
	
	//定时器初始化 10ms
	TIM_Init(TIM2, 999, 839, 3, 3);
	
	//初始化内存池
	BufferZizeInit(500);
	
	BeefON();

	TIM_Delayms(TIM5, 1000);
	
	BeefOFF();
	
	
	while(GyroscopeFlag == 0)
	{
		//如果等待陀螺仪初始化期间发生按键触发情况，进行采样模式
		if(ReadSwitch() == 1)
		{
				BeefON();
				mode = SAMPLING_MODE;
		}
		else
		{
				BeefOFF();
		}
	}
	
	
	//电机初始化
if(mode == NORMAL_MODE)
{
		elmo_Enable(1);
		elmo_Enable(2);
	  elmo_Enable(3);
		
		Vel_cfg(1,13000,13000);
		Vel_cfg(2,13000,13000);
		Vel_cfg(3,13000,13000);

}
	
	//数据清空为
	for(i=0;i<SIM800_BUFFER_SIZE;i++)
	{
		Receive_Buffer[i]=NO_DATA;
	}
	//清空TC标志位，否则发送的第一个数据有问题
	USART_ClearFlag(SEND_TO_SIM800, USART_FLAG_TC);
	USART_ClearFlag(RECEIVE_SIM800, USART_FLAG_TC);
BeefON();
}






extern _Bool USARTSEND;
extern long Time_count_10ms;
uint8_t number_char[10];
	static int pos_x_last=0;

int main(void)
{
	Pose_t pos_end,pos_end2;
	uint32_t numCount = 0;
	int i=0;
	v_three dipan;
	static uint8_t state=0;
	static int count_sim=0;
	int pos_x_temp=0;
	static int state_cir=0;
	//初始化函数
	Init();
	
	while(1)
	{
		if(ReadSwitch() == 1)
		{
			numCount++;
			if(numCount > 1000)
			{
				break;
			}
		}
		else
		{
			numCount = 0;
		}
	}
	BeefOFF();
	while(1)
	{
		if(ReadSwitch() == 0)
		{
			numCount++;
			if(numCount > 2000)
			{
				break;
			}
		}
		else
		{
			numCount = 0;
		}
	}
	
	while(1)
	{
		USART_OUT(RECEIVE_SIM800,(uint8_t *)"x=%d\ty=%d\r\n",(int)GetPosx(),(int)GetPosy());	
		switch(state_cir)
		{
			case 0:
				dipan=SpeedControlWheel(300.0,45.0);state_cir=1;
				VelControlTriWheel(dipan.v1,dipan.v2,dipan.v3);	
				break;
			case 1:
				if(GetPosy()>600.0)
				{
					dipan=SpeedControlWheel(300.0,-45.0);state_cir=2;
					VelControlTriWheel(dipan.v1,dipan.v2,dipan.v3);	
				}	
				break;
			case 2:
				if(GetPosy()<0.0)
				{
					dipan=SpeedControlWheel(300.0,180.0);state_cir=3;
					VelControlTriWheel(dipan.v1,dipan.v2,dipan.v3);	

				}
				break;
			case 3:
				if(GetPosx()<0.0)
				{
					state_cir=0;
				}
				break;
			default :
				break;
			
		}
		//sim800c上传温度数据
		sim800c_main();
	}


}
