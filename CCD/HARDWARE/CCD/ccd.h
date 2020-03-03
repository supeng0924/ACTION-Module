#ifndef _CCD_H_
#define _CCD_H_

#include "sys.h"

typedef struct CCDStruct{
	u8 Pixel[128];         //像素值数组
	u8 Pixel_thresh[128];  //像素二值化数组
	u8 position;           //白条位置
	u8 expore_time;        //曝光时间  单位：ms
	u16 time_arr;          //定时器的 arr寄存器 的值
	u8 exp_time_flag;      //曝光时间到达标志
	u8 PixelAverageValue;  //128个像素的平均值 
	u8 record_view;        //是否看见白条的标志
	u8 expore_act_value;   //曝光实际值
	u8 expore_ref_low;     //曝光期望下限
	u8 expore_ref_high;    //曝光期望上限
	u8 expore_kp;          //曝光调节p参数
	u8 WhiteWidth;
}CCDStruct;

//#define SEND_IMAGE

#define SI_SetVal()   GPIO_SetBits(GPIOA,GPIO_Pin_2);//PTE4_OUT = 1;
#define SI_ClrVal()   GPIO_ResetBits(GPIOA,GPIO_Pin_2);//PTE4_OUT = 0;
#define CLK_ClrVal()  GPIO_ResetBits(GPIOA,GPIO_Pin_3);//PTE5_OUT = 0;
#define CLK_SetVal()  GPIO_SetBits(GPIOA,GPIO_Pin_3);//PTE5_OUT = 1;

#define left_size  3
#define right_size 124

void StartIntegration(void);   
void ImageCapture(unsigned char * ImageData);

void SamplingDelay(void);
void LandzoCCD_init(void);
void CalculateIntegrationTime(void) ;
uint8_t PixelAverage(uint8_t len, uint8_t *data) ;



void ccd(void);
void ccd_init(void);

void FindBoundary(void);



u8 getwhiteStripes(void);
u8 getmiddle(void);
u8 getexposuretime(void);
#endif 

