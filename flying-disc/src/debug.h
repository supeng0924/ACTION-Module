#ifndef __DEBUG_H
#define __DEBUG_H

#include <Windows.h>
#include <iostream>

#define OVAL_HEIGHT 150
#define OVAL_WIDTH 28
#define OVAL_HEIGHT_HALT (OVAL_HEIGHT>>1)
#define OVAL_WIDTH_HALF (OVAL_WIDTH>>1)

#define OVAL_ANGLE 95
#define CIRCLE_i 20

#define RED 0 
#define BLUE 1
#define COLOR BLUE   
#define EXPEND_SIZE 20

//#define EACH_FRAME_TIME


//串口发数相关的宏定义  
//#define USB_USART      //串口发数
//#define SELF_HAVE_FRISBEE //发送是否有自己的飞盘
//#define INIT_SIGNAL      //初始化信号

//#define WAIT_START_EXPORE  //等待开始曝光
//#define WAIT_ARRIVE_WORK  //等待开始曝光
#define DEBUG_MODE
#define REGION_DIV_OLD_METHOD
#define USE_VIDEO_RECORD
#define DEFEND_TWO_REGION
//#define NO_WHITE_BALANCE //没有白平衡阶段，单纯的通过记录的参数


/**
* @berif 时间戳，测试一段代码的运行时间，单位是ms
*/
class TimeStamp {
public:
	/**
	* @berif 测试开始的地方
	*/
	inline void start() { GetLocalTime(&starttime); }

	/**
	* @berif 测试结束的地方，并打印从start()开始运行的时间差
	*/
	inline int runtime()
	{
		GetLocalTime(&endtime);

		int diff = endtime.wMilliseconds - starttime.wMilliseconds;
		diff += (endtime.wSecond - starttime.wSecond) * 1000;
		diff += (endtime.wMinute - starttime.wMinute) * 60000;

		return diff;
	}
private:
	SYSTEMTIME starttime, endtime;
};


#endif 