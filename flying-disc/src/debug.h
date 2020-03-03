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


//���ڷ�����صĺ궨��  
//#define USB_USART      //���ڷ���
//#define SELF_HAVE_FRISBEE //�����Ƿ����Լ��ķ���
//#define INIT_SIGNAL      //��ʼ���ź�

//#define WAIT_START_EXPORE  //�ȴ���ʼ�ع�
//#define WAIT_ARRIVE_WORK  //�ȴ���ʼ�ع�
#define DEBUG_MODE
#define REGION_DIV_OLD_METHOD
#define USE_VIDEO_RECORD
#define DEFEND_TWO_REGION
//#define NO_WHITE_BALANCE //û�а�ƽ��׶Σ�������ͨ����¼�Ĳ���


/**
* @berif ʱ���������һ�δ��������ʱ�䣬��λ��ms
*/
class TimeStamp {
public:
	/**
	* @berif ���Կ�ʼ�ĵط�
	*/
	inline void start() { GetLocalTime(&starttime); }

	/**
	* @berif ���Խ����ĵط�������ӡ��start()��ʼ���е�ʱ���
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