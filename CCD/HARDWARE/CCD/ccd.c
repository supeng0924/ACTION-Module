#include "ccd.h"
#include "adc.h"
#include "math.h"
#include "usart.h"
#include "stdlib.h"
#include "delay.h"
#include "timer.h"

//声明CCDStruct变量
CCDStruct _ccd;
extern u8 ExpTimeFlag;


/*
** CCD初始化函数
** CCD引脚初始化  CCDStruct结构体一些变量初始化
*/
void ccd_init(void)
{
	// CCD引脚初始化
	LandzoCCD_init();               
	StartIntegration();
    	//成员初始化
	_ccd.position=64;
	_ccd.time_arr=8000-1;
	_ccd.expore_time=0;
	_ccd.exp_time_flag=0;
	_ccd.expore_ref_low=40;
	_ccd.expore_ref_high=100;
	_ccd.expore_kp=20;
}

/*
** CCD引脚初始化
** 引脚对应为
** SI---PA2
** CLK--PA3
** AO---PA1
*/
void LandzoCCD_init(void)
{	
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	
    //SI CLK 引脚配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_2;	 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
	GPIO_Init(GPIOA, &GPIO_InitStructure);					 	
	GPIO_ResetBits(GPIOA,GPIO_Pin_3);			     
	GPIO_ResetBits(GPIOA,GPIO_Pin_2);			     
    //AO 引脚配置
	Adc_Init();		
}

/*
** 像素值曝光流程初始化
** 只是按照时钟图给定脉冲，并未采集像素值
*/
void StartIntegration(void) {
    unsigned char i;
    SI_SetVal();            /* SI  = 1 */
    SamplingDelay();	
    CLK_SetVal();           /* CLK = 1 */
    SamplingDelay();
    SI_ClrVal();            /* SI  = 0 */
    SamplingDelay();
    CLK_ClrVal();           /* CLK = 0 */

    for(i=0; i<127; i++) {
        SamplingDelay();
        SamplingDelay();
        CLK_SetVal();       /* CLK = 1 */
        SamplingDelay();
        SamplingDelay();
        CLK_ClrVal();       /* CLK = 0 */
    }
    SamplingDelay();
    SamplingDelay();
    CLK_SetVal();           /* CLK = 1 */
    SamplingDelay();
    SamplingDelay();
    CLK_ClrVal();           /* CLK = 0 */
}

/*
** CCD主函数
*/
u8 totalcount=0;
void ccd(void)
{	 
#ifdef SEND_IMAGE
	//如果发送图像的话  每5个周期发送一次
	static int count_send=0;
#endif
	//曝光时间到达标志判断
	if( ExpTimeFlag == 1 )  //标志判断成功
	{
		//标志判断清空
		ExpTimeFlag = 0; 
		//采集像素值
        ImageCapture(_ccd.Pixel);
		//寻找边界
		FindBoundary();		

        
		
		//发送结果
#ifdef SEND_IMAGE
		count_send++;
		if(count_send==10)
		{
			count_send=0;
			SendImageData(_ccd.Pixel);			
		}
#else
		//确定下一个周期的 定时器arr值
		CalculateIntegrationTime();
#endif

	 }
}

/*
** AD采集128个像素点的像素值
*/
void ImageCapture(uint8_t * ImageData) {
uint8_t i;
extern uint8_t AtemP;

    SI_SetVal();            /* SI  = 1 */
    SamplingDelay();
    CLK_SetVal();           /* CLK = 1 */
    SamplingDelay();
    SI_ClrVal();            /* SI  = 0 */
    SamplingDelay();

    for(i = 0; i < 200; i++) {                    //更改250，让CCD的图像看上去比较平滑，
      SamplingDelay();  //200ns                  //把该值改大或者改小达到自己满意的结果。
    }
    CLK_SetVal();
    *ImageData = Get_Adc(ADC_Channel_1)>>4;     //4096/16=256  12位    灰度值是8位 
    ImageData++;
                
     CLK_ClrVal();delay_us(10); //delay_us(20);
    
    for(i=0; i<127; i++) 
	{						
		SamplingDelay();
        SamplingDelay();
        CLK_SetVal();       /* CLK = 1 */
        SamplingDelay();
        SamplingDelay();	
       *ImageData =  Get_Adc(ADC_Channel_1)>>4;	 
       ImageData ++ ; 
		CLK_ClrVal();          
    }             
    SamplingDelay();
    SamplingDelay();
    CLK_SetVal();           /* CLK = 1 */
    SamplingDelay();
    SamplingDelay();
    CLK_ClrVal();   
}
/*
** 寻找边界
*/
int contrastvalue=0;
int fengcha=0;
void FindBoundary(void)
{

	int i=0;
	int bianjiecha=0;
	uint8_t xuhao;
	uint8_t zuidazhi=0;
    uint8_t zuobianjie=0;
	uint8_t youbianjie=0;
	u8 temp_pos=0;
	u8 temp_wid=0;
	//128个像素点的平均值
	_ccd.PixelAverageValue=PixelAverage(128,_ccd.Pixel);
	
	//寻找128个像素点的最大值
	zuidazhi=_ccd.Pixel[0];
	xuhao=0;
	for(i=1; i<128; i++)      
	{
		if(_ccd.Pixel[i]>zuidazhi)
		{
			zuidazhi=_ccd.Pixel[i];
			xuhao=i;
		}
	}
	
    //计算出对比值
	contrastvalue=(_ccd.PixelAverageValue+zuidazhi)>>1;
	//与对比值进行对比
	for(i=0;i<128;i++)
	{
		if(_ccd.Pixel[i]>contrastvalue)
		{_ccd.Pixel_thresh[i]=1;}		
		else
		{_ccd.Pixel_thresh[i]=0;}
	}
	
	//计算峰差
   	if(xuhao>90)
	{fengcha=_ccd.Pixel[xuhao]-_ccd.Pixel[(xuhao-12)];}
	else
	{fengcha=_ccd.Pixel[xuhao]-_ccd.Pixel[(xuhao+12)];}
	 for(i=3;i<126;i++)             //  去噪声
	 {
		   if(_ccd.Pixel_thresh[i]==0)
		   {              //  去噪声为0的时候
			 if((_ccd.Pixel_thresh[i-1]==1)&&(_ccd.Pixel_thresh[i+1]==1))
			 {_ccd.Pixel_thresh[i]=1;}
		   }
		   if(_ccd.Pixel_thresh[i]==1)
		   {                 //  去噪声为1的时候
		     if((_ccd.Pixel_thresh[i-1]==0)&&(_ccd.Pixel_thresh[i+1]==0))
			 {_ccd.Pixel_thresh[i]=0;}
		   }
	  } 
	  	  

		 zuobianjie=left_size;
		 youbianjie=right_size;
		 for(i=xuhao;i>left_size;i--)
		 {
			 if(_ccd.Pixel_thresh[i]==0)
			 {zuobianjie=i;break;}
		 }
		 for(i=xuhao;i<right_size;i++)
		 {
			 if(_ccd.Pixel_thresh[i]==0)
			 {youbianjie=i;break;}
		 }
		 bianjiecha=youbianjie-zuobianjie;


		 if(fengcha>15)
		 {
			_ccd.record_view=1;
			 
			 temp_pos=(uint8_t)((zuobianjie+youbianjie)/2);
			 
			 if(abs(temp_pos-_ccd.position)>1)
				 _ccd.position=temp_pos;
			 if(abs(_ccd.WhiteWidth-bianjiecha)>3)
				 _ccd.WhiteWidth=bianjiecha;				 				
		 }
		 else
		 {
			_ccd.record_view=0;
		 }	
	 _ccd.expore_act_value=zuidazhi;
}

/*
** 确定下一个周期的 定时器arr值
*/
void CalculateIntegrationTime(void) 
{
	int exposure=0;
	int expore_temp=0;
	int value_te;

	//参考值低于最低限制
	if(_ccd.expore_act_value<_ccd.expore_ref_low)
	{
		exposure=_ccd.expore_ref_low-_ccd.expore_act_value;
		value_te=_ccd.time_arr+exposure*_ccd.expore_kp;	
		if(value_te>65000)
		{
			_ccd.time_arr=65000;
		}
		else 
		{			
			_ccd.time_arr=value_te;
		}
		TIM3->ARR=_ccd.time_arr;
	}
	//参考值高于最高限制
	else if(_ccd.expore_act_value>_ccd.expore_ref_high)
	{
		exposure=_ccd.expore_act_value-_ccd.expore_ref_high;
		value_te=_ccd.time_arr-exposure*_ccd.expore_kp;	
		if(value_te<2000) 
		{
			_ccd.time_arr=2000;		
		}
		else
		{
			_ccd.time_arr=value_te;	  
		}
		TIM3->ARR=_ccd.time_arr;
	}
	//将arr寄存器的值近似到实际曝光时间
	expore_temp=_ccd.time_arr/2000;
	if(expore_temp>255)
		expore_temp=255;
	else if(expore_temp<2)
		expore_temp=2;
	
	_ccd.expore_time=expore_temp;			
}

/*
** 计算均值
*/
uint8_t PixelAverage(uint8_t len, uint8_t *data) {
  unsigned char i;
  unsigned int sum = 0;
  for(i = 0; i<len; i++) {
    sum = sum + *data++;
  }
  return ((unsigned char)(sum/len));
}
/*
** 采样之间的延时
*/
void SamplingDelay(void){
   volatile uint8_t i ;
   for(i=0;i<1;i++) {
    __NOP();
    __NOP();}
   
}

/*
** 返回白条宽度
*/
u8 getwhiteStripes(void)
{
//	if(abs(bianjiecha-countbigold)<3)
//	{bianjiecha=countbigold;}
//	else
//	{countbigold=bianjiecha;}
	return _ccd.WhiteWidth;
}

/*
** 返回中间值
*/
u8 getmiddle(void)
{
	return _ccd.position;
}

/*
** 返回曝光时间 ms 
*/
u8 getexposuretime(void)
{
	if(_ccd.expore_time<5)
	{
		if(_ccd.expore_time<2)
			_ccd.expore_time=2;
		TIM4->ARR=(_ccd.expore_time*1000);
	}
	else
	{TIM4->ARR=5000;}
	return _ccd.expore_time;
}
