#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "lcd.h"
#include "usart.h"	 
#include "adc.h"
#include "timer.h"
#include "ccd.h"

//#define DEBUG_MODE
extern u8 TIME4flag_5ms;
static u8 temp=0,counttime=0;
static uint16_t huizong=0;
u8 couddd=0;
u8 kuandu=0;
int main(void)
{	
    //延时函数初始化		
	delay_init();	  
	//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	//串口初始化为115200  初始化  USART1------PA9  PA10
	uart_init(115200);	 	
	
	//TIME3 CCD曝光时间控制  时钟是72M
	TIM3_Int_Init(20000,35);  

	//TIME4 CCD发送数据控制  每间隔5ms发送一次
	TIM4_Int_Init(5000-1,71);

	//debug 模式下打开按键初始化
#ifdef DEBUG_MODE	
	KEY_Init();//按键初始化
#endif
	//CCD初始化  SI CLK AO引脚初始化	
	ccd_init();
	while(1)
	{
		//CCD 主函数初始化
		ccd();		
//数据发送
#ifdef SEND_IMAGE
		
#else		
		if(TIME4flag_5ms ==1)//每隔5ms 发一次数发送曝光时间  中间值，白条
		{
			TIME4flag_5ms=0;				
			USART_SendData(USART1,'a');
			USART_SendData(USART1,getmiddle());
			USART_SendData(USART1,getexposuretime());
			USART_SendData(USART1,getwhiteStripes());		
		}	
#endif		
	}	
}

