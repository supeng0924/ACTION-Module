#include <Windows.h>
#include <iostream> 
#include <fstream>
#include <process.h>
#include "debug.h"
#include "recognition.h"
#include "CameraApi.h"
#include "SerialPort.h"
#include <fstream>
#include "camerastatus.h"
#include <opencv2\opencv.hpp>

CSerialPort mySerialPort;
//摄像头状态宏定义：
#define WAIT_INIT             0  //等待初始化
#define START_INIT            1  //开始初始化
#define SELFCHECK_POS         2  //自检位置检测
#define GET_IMAGE             3  //1 2 4 5 着陆台V值匹配图像获取
#define WAIT_ARRIVE           4  //等待车到达射击点
#define PLAT1245_V_THRESHOLD  5  //1 2 4 5 着陆台
#define IDENTIFY_12457        7  //识别1245+7号防守匹配阙值确定
#define ONLY_DEFENSIVE        6  //仅7号台防守
#define ONLY_ORIGIN           8  //只显示原图像
#define TEST_FIELD            9  //着陆台参数手动调节
#define TEST_REF_PARAMETER   10  //着陆台手动调节参考参数效果
#define IDENTIFY_1245        11  //只是识别1245
#define MANU_DEFEND          12  //手动控制枪
#define TEST_BALL            13  //识别球测试
#define DEBUG_STATE          14  //调试相关
#define SHICHANG             15  //视场地
#define TEXTMATCH            16  //测试匹配
#define TEXCameraBao         17  //测试曝光时间返回=========================未测量===================================
#define DIVIDE_REGION        18  //分区测试   
#define RECOGNISE_Disc       19  //识别飞盘形状
#define WhiteBalance         20  //图像白平衡  如果没有初始化 是否需要自动白平衡 。。。。
#define PaperWhiteBalance    21  //根据白纸进行白平衡
#define PaperWBInit          22  //根据白纸进行白平衡
#define Picture2Video        23  //进行图片转换为视频
#define HistogramPrac        24  //直方图练习
#define HistogramPracInit    25  //直方图初始化
#define RGBjiaozheng         26  //RGB矫正

//摄像头状态
int cam_status = SHICHANG;

//己方场地颜色
char color_reg = 'r';
int picture_rec = 0;
static int self_pan_signal = 0;
WBAdjust light_wb;
VideoRecord record_video;
POINT_AND_POINT position_check;
int gain_max = 400;
int gain_analog_max = 100;
static int one_col = 0, two_col = 0, two_row = 0;
static int row_l = 0, row_m = 0, row_r = 0;
GainARGB camera_gain;


//试场调整参数***********************************开始******************************************
//摄像头增益参数
int gain_r = 140;
int gain_g = 175;
int gain_b = 208;
int gain_analog_total = 4;
int Expore_min_time = 30000;
int Expore_max_time = 7000;

int plat7_height = 223;
int plat7_width = 64;
float plat7_angle = 90.4;
int plat7_angle_10x = 904;
int plat7_v_val = 140;
int plat7_s_val = 55;
cv::Point center_ref_poi = cv::Point(280, 51);//**********************

//飞盘参数
int red_h_big = 160;
int red_h_small = 5;
int red_s = 140;
int red_v = 60;
int blue_h_big = 130;
int blue_h_small = 100;
int blue_s = 80;
int blue_v = 50;
int hsv_max = 255;
int black_s = 90;
int black_v = 90;

//*********************************参数调节区结束*****************************************




UINT            m_threadID;		        //图像抓取线程的ID
HANDLE          m_hDispThread;	        //图像抓取线程的句柄
BOOL            m_bExit = FALSE;		//用来通知图像抓取线程结束
CameraHandle    m_hCamera;		        //相机句柄，多个相机同时使用时，可以用数组代替	
BYTE*           m_pFrameBuffer;         //用于将原始图像数据转换为RGB的缓冲区
char		    g_CameraName[64];

#define NEAR_PLAT_SYN_BYTE 0x80   //近台区号起始字节
#define PLAT_SYN_BYTE 0x8B        //近台区号起始字节
using namespace cv;
int plot_y = 0, plot_x = 0, signal_shot = 0;
std::ofstream out_stream, out_hsv_stream, out_every_data;

//鼠标回调函数
void on_MouseHandle_camera(int event, int x, int y, int flags, void *param)
{
	if (param == nullptr)
		return;

	act::Recognition reco(*(act::Recognition*)param);
	switch (event)
	{
	case CV_EVENT_LBUTTONDOWN:
		auto hsv_pix = reco.getHSVImage().ptr<cv::Vec3b>(y, x);
		auto org_pix = reco.getOriginalImage().ptr<cv::Vec3b>(y, x);
		std::cout << "(" << x << ", " << y << ") ";
		std::cout << "(" << "H " << (int)(*hsv_pix)[0] << ",S " << (int)(*hsv_pix)[1] << ",V " << (int)(*hsv_pix)[2] << ") ";
		std::cout << "(" << "B " << (int)(*org_pix)[0] << ",G " << (int)(*org_pix)[1] << ",R " << (int)(*org_pix)[2] << ")" << std::endl;
		plot_y = y;
		plot_x = x; signal_shot = 1;

		out_hsv_stream << (int)(*hsv_pix)[0] << "\t" << (int)(*hsv_pix)[1] << "\t" << (int)(*hsv_pix)[2] << std::endl;
		break;
	}
}


//发送
#define MAX_FIRBEE 8
cv::Point Frisbee_posi[MAX_FIRBEE];
cv::Point Frisbee_push_point;
int count_frisbee_state = 0;
int region_have[7] = { 0 };
int region_leiji_send[7] = { 0 };
int region_send_signal[7] = { 0 };
int Frisbee_many = 0;
//发送近台对方盘所在区号
void SendSignal(unsigned char send_data_comb)
{
#ifdef DEFEND_TWO_REGION
	static unsigned char last_have = 0;
	static unsigned char have_one = 0;
	static unsigned char have_two = 0;
	static unsigned char one_num = 0;
	static unsigned char two_num = 0;

	if (!send_data_comb)
	{
		have_one = 0;
		have_two = 0;
		one_num = 0;
		two_num = 0;
		last_have = 0;
		std::cout << "no you" << std::endl;
	}
	else
	{
		if (!have_one)
		{
			for (unsigned char i = 0; i < 6; i++)
			{
				if (send_data_comb&(1 << i))
				{
					have_one = 1 << i;
					one_num = i + 1;
					send_data_comb &= ~(1 << i);
					break;
				}
			}
		}
		else
		{
			if (send_data_comb&have_one)
			{
				send_data_comb &= ~(have_one);
			}
			else if (have_two)
			{
				if (have_two&send_data_comb)
				{
					have_one = have_two;
					one_num = two_num;
					send_data_comb &= ~have_two;
				}
				else
				{
					for (unsigned char i = 0; i < 6; i++)
					{
						if (send_data_comb&(1 << i))
						{
							have_one = 1 << i;
							one_num = i + 1;
							send_data_comb &= ~(1 << i);
							break;
						}
					}
				}
			}
			else
			{
				for (unsigned char i = 0; i < 6; i++)
				{
					if (send_data_comb&(1 << i))
					{
						have_one = 1 << i;
						one_num = i + 1;
						send_data_comb &= ~(1 << i);
						break;
					}
				}
			}
		}

		have_two = 0;
		two_num = 0;
		for (unsigned char i = 0; i < 6; i++)
		{
			if (send_data_comb&(1 << i))
			{
				have_two = 1 << i;
				two_num = i + 1;
				send_data_comb &= ~(1 << i);
				break;
			}
		}
		last_have = 0;
		if (have_two)
		{
			last_have = two_num << 4;
		}
		if (have_one)
		{
			last_have |= (one_num);
		}
		std::cout << "data: " << " " << (int)send_data_comb << "  " << (int)last_have << "  " << (int)one_num << "  " << (int)two_num << std::endl;

	}
#endif
#ifdef DEFEND_MANY_REGION
	static unsigned char last_have = 0;
	static unsigned char have_one = 0;
	static unsigned char have_two = 0;
	static unsigned char one_num = 0;
	static unsigned char two_num = 0;

	if (!send_data_comb)
	{
		have_one = 0;
		have_two = 0;
		one_num = 0;
		two_num = 0;
		last_have = 0;
		std::cout << "no you" << std::endl;
	}
	else
	{
		if (Frisbee_many < 3)
		{
			if (!have_one)
			{
				for (unsigned char i = 0; i < 6; i++)
				{
					if (send_data_comb&(1 << i))
					{
						have_one = 1 << i;
						one_num = i + 1;
						send_data_comb &= ~(1 << i);
						break;
					}
				}
			}
			else
			{
				if (send_data_comb&have_one)
				{
					send_data_comb &= ~(have_one);
				}
				else if (have_two)
				{
					if (have_two&send_data_comb)
					{
						have_one = have_two;
						one_num = two_num;
						send_data_comb &= ~have_two;
					}
					else
					{
						for (unsigned char i = 0; i < 6; i++)
						{
							if (send_data_comb&(1 << i))
							{
								have_one = 1 << i;
								one_num = i + 1;
								send_data_comb &= ~(1 << i);
								break;
							}
						}
					}
				}
				else
				{
					for (unsigned char i = 0; i < 6; i++)
					{
						if (send_data_comb&(1 << i))
						{
							have_one = 1 << i;
							one_num = i + 1;
							send_data_comb &= ~(1 << i);
							break;
						}
					}
				}
			}
			have_two = 0;
			two_num = 0;
			for (unsigned char i = 0; i < 6; i++)
			{
				if (send_data_comb&(1 << i))
				{
					have_two = 1 << i;
					two_num = i + 1;
					send_data_comb &= ~(1 << i);
					break;
				}
			}
			last_have = 0;
			if (have_two)
			{
				last_have = two_num << 3;
			}
			if (have_one)
			{
				last_have |= (one_num);
			}
		}
		else
		{
			last_have = send_data_comb;
			last_have |= 0xC0;
			one_num = 0;
			two_num = 0;
			have_one = 0;
			have_two = 0;
		}
	}
	std::cout << "data: " << Frisbee_many << " " << (int)send_data_comb << "  " << (int)last_have << "  " << (int)one_num << "  " << (int)two_num << std::endl;
#endif

#ifdef OLD_REGION
	static unsigned char last_have = 0;
	if (!send_data_comb)
		last_have = 0;
	else
	{
		if (send_data_comb&last_have)
		{
			;
		}
		else
		{
			for (unsigned char i = 0; i < 6; i++)
			{
				if (send_data_comb&(1 << i))
				{
					last_have = 1 << i;
					break;
				}
			}
		}
	}
	if (last_have)
	{
		std::cout << "data: " << (int)last_have << " " << std::endl;
	}
#endif



#ifdef USB_USART
	unsigned char send_data_syn = NEAR_PLAT_SYN_BYTE;
	while (mySerialPort.WriteData(&send_data_syn, 1) == false)
	{
		cv::waitKey(100);
	}
	while (mySerialPort.WriteData(&send_data_syn, 1) == false)
	{
		cv::waitKey(100);
	}
	while (mySerialPort.WriteData(&last_have, 1) == false)
	{
		cv::waitKey(10);
	}
#endif
}


void SendDataRegion(unsigned char send_data_comb)
{
#ifdef USB_USART
	unsigned char send_data_syn = NEAR_PLAT_SYN_BYTE;
	while (mySerialPort.WriteData(&send_data_syn, 1) == false)
	{
		cv::waitKey(100);
	}
	while (mySerialPort.WriteData(&send_data_syn, 1) == false)
	{
		cv::waitKey(100);
	}
	while (mySerialPort.WriteData(&send_data_comb, 1) == false)
	{
		cv::waitKey(10);
	}
#endif
}
void NotSendSignal(unsigned char send_data_comb)
{
	static unsigned char last_have = 0;
	if (!send_data_comb)
		last_have = 0;
	else
	{
		if (send_data_comb&last_have)
		{
			;
		}
		else
		{
			for (unsigned char i = 0; i < 6; i++)
			{
				if (send_data_comb&(1 << i))
				{
					last_have = 1 << i;
					break;
				}
			}
		}
	}

	if (last_have)
	{
		std::cout << "data: " << (int)last_have << " " << std::endl;
	}
}

//曝光参数 起始，结束，对应7号着陆台参数
ExporeTime Expore_par;
#define WINDOW_NAME_TIAO "滑动条"
#define RGBGAIN "RGB增益"
#define Gain_Exp "增益与曝光"
#define PLAT_SEVEN "着陆台7"
#define CALIBRATION "校准坐标"
#define HONGPAN  "红盘"
#define LANPAN  "蓝盘"

int max_expore_time = 100000;
int H_max_que = 255;
int V_max_que = 255;
int max_height_value = 400;
int max_width_value = 120;
int max_angle_value = 1100;
int max_v_val = 200;
int max_s_val = 250;

//着陆台参数
OVAL_parameter platform_7;

CameraHandle    hCamera_cal;
int expore_time = 50000;
void on_Trackbar(int, void*)
{
	double stime = 0;
	double *pfExposureTime = &stime;
	if (!CameraGetExposureTime(hCamera_cal, pfExposureTime))
		std::cout << "bao  " << stime << std::endl;
	if (!CameraSetExposureTime(hCamera_cal, (double)expore_time))
	{
		std::cout << "now  " << stime << std::endl;
	}
}

int gain_signal = 0;
void on_gain_rgb(int, void*)
{
	gain_signal = 1;
	if (!CameraSetGain(hCamera_cal, gain_r, gain_g, gain_b))
	{
		std::cout << "rgb gain ok" << std::endl;;
	}
	else
	{
		std::cout << "rgb gain error" << std::endl;;
	}
	if (!CameraSetAnalogGain(hCamera_cal, gain_analog_total))
	{
		std::cout << "analog gain ok" << std::endl;;
	}
	else
	{
		std::cout << "analog gain error" << std::endl;;
	}
	std::cout << " r " << gain_r << " g " << gain_g << " b " << gain_b << "analog " << gain_analog_total << std::endl;
}

void Trackbar_p7(int, void*)
{
	platform_7.angle = (float)(platform_7.angle_10x / 10);
}

//设置曝光时间参数
void set_expore(double expore_time)
{
	if (!CameraSetExposureTime(hCamera_cal, (double)expore_time))
	{
		std::cout << "set  " << expore_time << "  ";
	}
	else
	{
		std::cout << "set  failer" << std::endl;
	}
}

extern int car_stop;
extern int car_stop_move;

//校准窗口起点和终点
Calibration cali_window;
int max_st_end_xy = 1024;


//曝光参数初始化
void Expore_Parameter_Init(void)
{
	Expore_par.expore_time_start = Expore_max_time;
	Expore_par.expore_time_end = Expore_min_time;
	Expore_par.exporeTime = Expore_par.expore_time_start;
	Expore_par.maxValue_angle = 0;
	Expore_par.maxValue_exporeTime = 0;
	Expore_par.maxValue_value = 0;
	Expore_par.expore_time_div = 2000;

	camera_gain.b = gain_b;
	camera_gain.g = gain_g;
	camera_gain.r = gain_r;
	camera_gain.analog = gain_analog_total;


	light_wb.first = cv::Point(457, 92);
	light_wb.last = cv::Point(500, 103);
	light_wb.complete_signal = 0;
	light_wb.b_prop_coeff = 1.0;
	light_wb.g_prop_coeff = 1.0;
	light_wb.r_prop_coeff = 1.0;
}

//录像初始化
void Video_Record_Init(void)
{
	//初始化视频编写器，参数根据实际视频文件修改
	record_video.writer = 0;
	record_video.isColor = 1;
	record_video.fps = 10;
	record_video.frameW = 504;
	record_video.frameH = 147;
	record_video.writer = cvCreateVideoWriter("out.avi", CV_FOURCC('D', 'I', 'V', 'X'), record_video.fps, cvSize(record_video.frameW, record_video.frameH), record_video.isColor);
	printf("tvideo height : %d video width : %d  fps : %d \n", record_video.frameH, record_video.frameW, record_video.fps);
}
//增益参数初始化
void Gain_Analog_RGB_Init(void)
{
	if (!CameraSetGain(hCamera_cal, gain_r, gain_g, gain_b))
	{
		std::cout << "set rgb ok" << std::endl;;
	}

	if (!CameraSetAnalogGain(hCamera_cal, gain_analog_total))
	{
		std::cout << "set analog ok" << std::endl;;
	}
}

//着陆台参数初始化
void Plat_Parameter_Init(void)
{
	platform_7.plat_num = 7;
	platform_7.angle = plat7_angle;
	platform_7.angle_10x = plat7_angle_10x;
	platform_7.height = plat7_height;
	platform_7.width = plat7_width;
	platform_7.RectSt = cv::Point(333, 108);
	platform_7.RectEnd = cv::Point(622, 229);
	platform_7.v_val = plat7_v_val;
	platform_7.s_val = plat7_s_val;

	platform_7.center_ref = center_ref_poi;
	platform_7.center_dert = cv::Size(20, 20);
	platform_7.angle_ref = platform_7.angle;
	platform_7.angle_dert = 15;
}

TimeStamp A_Frame_Time;
int get_picture = 0;
cv::Mat getImage_deal;
extern int task_ball_plat_over;

/**
* @berif 图像抓取线程，主动调用SDK接口函数获取图像
*/
UINT WINAPI uiDisplayThread(LPVOID lpParam)
{
	tSdkFrameHead 	sFrameInfo;
	CameraHandle    hCamera = (CameraHandle)lpParam;
	BYTE*			pbyBuffer;
	CameraSdkStatus status;

	//初始化次数记录
	static int time_5_sig = 0;
	Expore_Parameter_Init();
	Plat_Parameter_Init();

#ifdef USE_VIDEO_RECORD
	Video_Record_Init();
#endif // USE_VIDEO_RECORD


	light_wb.first = cv::Point(457, 92);
	light_wb.last = cv::Point(500, 119);
	position_check.first= cv::Point(210, 98);
	position_check.last= cv::Point(281, 119);

	//校准参数
	cali_window.start_x = 161;
	cali_window.start_y = 42;
	cali_window.end_x = 224;
	cali_window.end_y = 70;

	//创建窗体
	cv::namedWindow(CALIBRATION);
	cv::namedWindow(Gain_Exp);
	cv::namedWindow(PLAT_SEVEN);
	cv::namedWindow(HONGPAN);
	cv::namedWindow(LANPAN);
	cv::namedWindow("original");
	cv::namedWindow("mainWin", CV_WINDOW_AUTOSIZE);


	//******************************创建滑动条**********************************************
	//校准窗口
	{
		char TrackbarNamestx[50];
		sprintf(TrackbarNamestx, "起点x %d", max_st_end_xy);
		cv::createTrackbar(TrackbarNamestx, CALIBRATION, &light_wb.first.x, max_st_end_xy);

		char TrackbarNamesty[50];
		sprintf(TrackbarNamesty, "起点y %d", max_st_end_xy);
		cv::createTrackbar(TrackbarNamesty, CALIBRATION, &light_wb.first.y, max_st_end_xy);

		char TrackbarNameendx[50];
		sprintf(TrackbarNameendx, "终点x %d", max_st_end_xy);
		cv::createTrackbar(TrackbarNameendx, CALIBRATION, &light_wb.last.x, max_st_end_xy);

		char TrackbarNameendy[50];
		sprintf(TrackbarNameendy, "终点y %d", max_st_end_xy);
		cv::createTrackbar(TrackbarNameendy, CALIBRATION, &light_wb.last.y, max_st_end_xy);
	}
	//曝光和增益窗口
	{
		char TrackbarNameendAnalog[50];
		sprintf(TrackbarNameendAnalog, "analog %d", gain_analog_max);
		cv::createTrackbar(TrackbarNameendAnalog, Gain_Exp, &gain_analog_total, gain_analog_max, on_gain_rgb);

		char TrackbarName[50];
		sprintf(TrackbarName, "透明值 %d", max_expore_time);
		cv::createTrackbar(TrackbarName, Gain_Exp, &expore_time, max_expore_time, on_Trackbar);

		char TrackbarNamestR[50];
		sprintf(TrackbarNamestR, "r %d", gain_max);
		cv::createTrackbar(TrackbarNamestR, Gain_Exp, &gain_r, gain_max, on_gain_rgb);

		char TrackbarNamestG[50];
		sprintf(TrackbarNamestG, "g %d", gain_max);
		cv::createTrackbar(TrackbarNamestG, Gain_Exp, &gain_g, gain_max, on_gain_rgb);

		char TrackbarNameendB[50];
		sprintf(TrackbarNameendB, "b %d", gain_max);
		cv::createTrackbar(TrackbarNameendB, Gain_Exp, &gain_b, gain_max, on_gain_rgb);
	}
	//飞盘参数
	{
		char TrackbarNameRedBig[50];
		sprintf(TrackbarNameRedBig, "红盘H> %d", hsv_max);
		cv::createTrackbar(TrackbarNameRedBig, HONGPAN, &red_h_big, hsv_max);
		char TrackbarRedsmall[50];
		sprintf(TrackbarRedsmall, "红盘H< %d", hsv_max);
		cv::createTrackbar(TrackbarRedsmall, HONGPAN, &red_h_small, hsv_max);
		char TrackbarRedS[50];
		sprintf(TrackbarRedS, "红盘S %d", hsv_max);
		cv::createTrackbar(TrackbarRedS, HONGPAN, &red_s, hsv_max);
		char TrackbarRedV[50];
		sprintf(TrackbarRedV, "红盘V %d", hsv_max);
		cv::createTrackbar(TrackbarRedV, HONGPAN, &red_v, hsv_max);
	}
	{

		char bluebig[50];
		sprintf(bluebig, "蓝盘H> %d", hsv_max);
		cv::createTrackbar(bluebig, LANPAN, &blue_h_big, hsv_max);
		char bluesmall[50];
		sprintf(bluesmall, "蓝盘H< %d", hsv_max);
		cv::createTrackbar(bluesmall, LANPAN, &blue_h_small, hsv_max);
		char blues[50];
		sprintf(blues, "蓝盘S %d", hsv_max);
		cv::createTrackbar(blues, LANPAN, &blue_s, hsv_max);
		char bluev[50];
		sprintf(bluev, "蓝盘V %d", hsv_max);
		cv::createTrackbar(bluev, LANPAN, &blue_v, hsv_max);

		char blacks[50];
		sprintf(blacks, "黑边S< %d", hsv_max);
		cv::createTrackbar(blacks, LANPAN, &black_s, hsv_max);

		char blackv[50];
		sprintf(blackv, "黑边V< %d", hsv_max);
		cv::createTrackbar(blackv, LANPAN, &black_v, hsv_max);

	}
	//着陆台7
	{
		char Plat7_ang[50];
		sprintf(Plat7_ang, "角度值 %d", max_angle_value);
		cv::createTrackbar(Plat7_ang, PLAT_SEVEN, &platform_7.angle_10x, max_angle_value, Trackbar_p7);

		char Plat7_hei[50];
		sprintf(Plat7_hei, "高度 %d", max_height_value);
		cv::createTrackbar(Plat7_hei, PLAT_SEVEN, &platform_7.height, max_height_value, Trackbar_p7);

		char Plat7_wid[50];
		sprintf(Plat7_wid, "宽度 %d", max_width_value);
		cv::createTrackbar(Plat7_wid, PLAT_SEVEN, &platform_7.width, max_width_value, Trackbar_p7);

		char Plat7_v_val[50];
		sprintf(Plat7_v_val, "V %d", max_v_val);
		cv::createTrackbar(Plat7_v_val, PLAT_SEVEN, &platform_7.v_val, max_v_val, Trackbar_p7);

		char Plat7_s_val[50];
		sprintf(Plat7_s_val, "S %d", max_s_val);
		cv::createTrackbar(Plat7_s_val, PLAT_SEVEN, &platform_7.s_val, max_s_val, Trackbar_p7);
	}

	//*********************************滑动条结束**********************************************
	act::Recognition fr;
	cv::setMouseCallback("original", on_MouseHandle_camera, &fr);

	//存放圆心坐标
	out_stream.open("circle_center.txt", std::ios::app);
	out_stream << "init start: " << std::endl;

	//写入hsv值
	out_hsv_stream.open("hsv.txt", std::ios::app);
	out_every_data.open("out_every_data.txt", std::ios::app);
	out_every_data << "start record:  " << std::endl;
	cv::Mat original_image;
	static int gain_set_time2 = 0;

	while (!m_bExit)
	{
		if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS)
		{
			//将获得的原始数据转换成RGB格式的数据，同时经过ISP模块，对图像进行降噪，边沿提升，颜色校正等处理。
			status = CameraImageProcess(hCamera, pbyBuffer, m_pFrameBuffer, &sFrameInfo);//连续模式

			if (status == CAMERA_STATUS_SUCCESS)
			{
				//-------------------------------3/4ms
				hCamera_cal = hCamera;
				cv::Mat original_image_all = cv::Mat({ sFrameInfo.iWidth, sFrameInfo.iHeight }, CV_8UC3, m_pFrameBuffer);
				//cv::imshow("original_image_all", original_image_all);
				//cv::resize(original_image_all, original_image, cv::Size(original_image_all.cols / 2, original_image_all.rows / 2), (0, 0), (0, 0), 3);			
				cv::resize(original_image_all, original_image, cv::Size(original_image_all.cols / 8 * 3, original_image_all.rows / 8 * 3), (0, 0), (0, 0), 3);
				cv::imshow("original", original_image);
				//白平衡图像
				if (light_wb.complete_signal)
				{
					WhiteBalancePicture(original_image, light_wb);
					cv::imshow("original_wb", original_image);
					fr.update(original_image.size(), CV_8UC3, original_image.data);
				}
				else
				{
					fr.update(original_image.size(), CV_8UC3, original_image.data);
				}
				if (cam_status == WAIT_INIT)
				{
					if (gain_set_time2 < 2)
					{
						Gain_Analog_RGB_Init();
						set_expore(Expore_max_time);
						gain_set_time2++;
					}
#ifdef DEBUG_MODE
					std::cout << "debug mode " << std::endl;
					//cam_status = RGBjiaozheng;
#else
					if (GetColorReco())
					{
						std::cout << "play field color  :" << color_reg << std::endl;
						color_reg = GetColorReco();
						cam_status = RGBjiaozheng;
					}
#endif // DEBUG_MODE

#ifdef NO_WHITE_BALANCE
					std::cout << "no white balance mode " << std::endl;
					cam_status = WAIT_ARRIVE;

#endif // NO_WHITE_BALANCE					
					cv::Point left_up, right_up, left_down, right_down;
					left_up = light_wb.first;
					right_up = cv::Point(light_wb.last.x, light_wb.first.y);
					left_down = cv::Point(light_wb.first.x, light_wb.last.y);
					right_down = light_wb.last;
					cv::line(original_image, left_up, right_up, cv::Scalar(255, 0, 255));
					cv::line(original_image, right_up, right_down, cv::Scalar(255, 0, 255));
					cv::line(original_image, right_down, left_down, cv::Scalar(255, 0, 255));
					cv::line(original_image, left_down, left_up, cv::Scalar(255, 0, 255));


					left_up = position_check.first;
					right_up = cv::Point(position_check.last.x, position_check.first.y);
					left_down = cv::Point(position_check.first.x, position_check.last.y);
					right_down = position_check.last;
					cv::line(original_image, left_up, right_up, cv::Scalar(255, 0, 255));
					cv::line(original_image, right_up, right_down, cv::Scalar(255, 0, 255));
					cv::line(original_image, right_down, left_down, cv::Scalar(255, 0, 255));
					cv::line(original_image, left_down, left_up, cv::Scalar(255, 0, 255));



					cv::imshow("original", original_image);
#ifdef INIT_SIGNAL
					unsigned char send_data_syn = 0x77;
					while (mySerialPort.WriteData(&send_data_syn, 1) == false)
					{
						cv::waitKey(100);
					}
					//static int only_once = 0;
					//if (!only_once)
					//{
					//	only_once = 1;
					//	for (int ii = 0; ii < 100000; ii++)
					//	{
					//		unsigned char send_data_syn = 0x77;
					//		while (mySerialPort.WriteData(&send_data_syn, 1) == false)
					//		{
					//			cv::waitKey(100);
					//		}
					//	}
					//}
#endif
				}
				else if (cam_status == ONLY_DEFENSIVE)
				{
					if (GetSelf_che_signal())
					{
						cam_status = SELFCHECK_POS;
					}
					int cout_endl_signal = 0;
					cv::Mat frame_plat7;
					cv::Mat outer_edges_image, median_image7;

					//为实现匹配算法  提前提取图像中边界
					frame_plat7 = original_image;
					auto hsv_thr = thresholdHSV(fr.getHSVImage(), platform_7.v_val, platform_7.s_val);
					cv::medianBlur(hsv_thr, median_image7, 5);
					cv::imshow("median_image7", median_image7);
					cv::Canny(median_image7, outer_edges_image, 3, 9, 3);
					cv::imshow("outer_edges_image", outer_edges_image);


					//匹配着陆台
					platform_7.center = fr.MatchPartNearPlat(outer_edges_image, frame_plat7, platform_7.angle, platform_7.height, platform_7.width);
					std::cout << "cen: " << platform_7.center << "  ";

					//进行位置限定
					if (abs(platform_7.center.x - platform_7.center_ref.x) < 50 && abs(platform_7.center.y - platform_7.center_ref.y) < 20)
					{
						;
					}
					else
					{
						platform_7.center.x = platform_7.center_ref.x;
						platform_7.center.y = platform_7.center_ref.y;
						std::cout << "recognition error" << std::endl;
					}
					cv::Mat frisbee_bin_img;
					cv::Mat self_frisbee;
					auto roi_image = original_image;

					//黑边二值化
					auto frisbee_black = fr.thresholdByHSVpanzi([](cv::Vec3b* p)->bool { return (*p)[1] < black_s && (*p)[2] < black_v; });
					cv::medianBlur(frisbee_black, frisbee_black, 5);

					cv::imshow("frisbee_black", frisbee_black);

					//蓝盘 红盘  寻找
					if (color_reg == 'r')
					{
						frisbee_bin_img = fr.thresholdByHSVpanzi([](cv::Vec3b* p)->bool { return (((*p)[0] < blue_h_big) && ((*p)[0] > blue_h_small) && (*p)[1] > blue_s && (*p)[2] > blue_v); });//蓝色
						self_frisbee = fr.thresholdByHSVpanzi([](cv::Vec3b* p)->bool { return ((((*p)[0] > red_h_big || (*p)[0] < red_h_small) && (*p)[1] > red_s && (*p)[2] > red_v)); });//红色
					}
					else
					{
						self_frisbee = fr.thresholdByHSVpanzi([](cv::Vec3b* p)->bool { return (((*p)[0] < blue_h_big) && ((*p)[0] > blue_h_small) && (*p)[1] > blue_s && (*p)[2] > blue_v); });//蓝色
						frisbee_bin_img = fr.thresholdByHSVpanzi([](cv::Vec3b* p)->bool { return ((((*p)[0] > red_h_big || (*p)[0] < red_h_small) && (*p)[1] > red_s && (*p)[2] > red_v)); });//红色
					}


					//防守  识别敌人的飞盘 形态学开运算
					cv::Mat median_image_roi;
					cv::medianBlur(frisbee_bin_img, median_image_roi, 5);
					cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
					cv::morphologyEx(median_image_roi, median_image_roi, cv::MORPH_OPEN, element);
					cv::imshow("敌人飞盘", median_image_roi);

					std::vector<cv::Point> point_rect;
					//提取边界  但是对内部空洞未能实现滤除  返回值是框的四个顶点
					point_rect = fr.find_contours_te(median_image_roi, frisbee_black, platform_7);


					//飞盘中心点确定 同时根据进行绘制
					std::vector<cv::Point> point_pos;
					if (point_rect.size())
					{
						int i_rect = point_rect.size() >> 2;
						for (int i = 0; i < i_rect; i++)
						{
							int cir_i = i * 4;
							cv::line(roi_image, point_rect[cir_i], point_rect[cir_i + 1], cv::Scalar(0, 255, 255));
							cv::line(roi_image, point_rect[cir_i + 1], point_rect[cir_i + 2], cv::Scalar(0, 255, 255));
							cv::line(roi_image, point_rect[cir_i + 2], point_rect[cir_i + 3], cv::Scalar(0, 255, 255));
							cv::line(roi_image, point_rect[cir_i + 3], point_rect[cir_i], cv::Scalar(0, 255, 255));

							int x = (point_rect[cir_i].x + point_rect[cir_i + 2].x) >> 1;
							int y = (point_rect[cir_i].y + point_rect[cir_i + 2].y) >> 1;
							cv::Point send = cv::Point(x, y);
							//cv::circle(roi_image, send, 3, cv::Scalar(0, 220, 0));
							point_pos.push_back(send);
						}
					}


					int num = 0;
					uchar send_data_comb = 0;

					//着陆台左上角点确定
					cv::Point image_st = cv::Point((platform_7.center.x - platform_7.height / 2), (platform_7.center.y - platform_7.width / 2));
					cv::circle(roi_image, image_st, 3, cv::Scalar(200, 0, 0));

					//分区界限划分
					int  row_2_3;
					int row_left = 0, row_middle = 0, row_right = 0;
					int col_1_3, col_2_3, region_num = 0;
					col_1_3 = platform_7.center.x - platform_7.height / 6;
					col_2_3 = platform_7.center.x + platform_7.height / 6;
					row_2_3 = platform_7.center.y + platform_7.width *0.1;

					row_left = image_st.y + platform_7.width *0.57;
					row_middle = image_st.y + platform_7.width *0.63;
					row_right = image_st.y + platform_7.width *0.57;

					if (abs(one_col - col_1_3) < 10)
					{
						col_1_3 = one_col;
					}
					else
					{
						one_col = col_1_3;
					}
					if (abs(two_col - col_2_3) < 10)
					{
						col_2_3 = two_col;
					}
					else
					{
						two_col = col_2_3;
					}

					if (abs(row_l - row_left) < 10)
					{
						row_left = row_l;
					}
					else
					{
						row_l = row_left;
					}

					if (abs(row_m - row_middle) < 10)
					{
						row_middle = row_m;
					}
					else
					{
						row_m = row_middle;
					}

					if (abs(row_r - row_right) < 10)
					{
						row_right = row_r;
					}
					else
					{
						row_r = row_right;
					}
					if (abs(two_row - row_2_3) < 10)
					{
						row_2_3 = two_row;
					}
					else
					{
						two_row = row_2_3;
					}

					one_col = col_1_3;
					two_col = col_2_3;
					two_row = row_2_3;
					row_r = row_right;
					row_m = row_middle;
					row_l = row_left;
					LandBoundary landboundary;
					landboundary.one_col = col_1_3;
					landboundary.two_col = col_2_3;
					landboundary.two_row = row_2_3;
					landboundary.row_r = row_right;
					landboundary.row_m = row_middle;
					landboundary.row_l = row_left;


					std::cout << " lie " << col_1_3 << " " << col_2_3 << " " << row_l << "  " << row_m << "  " << row_r << "  ";

					//飞盘中心点的位置加了一个滤波 
					//连续两个周期位置 x差20 y差10 以内认为是同一个点
					static int disc_signal = 0;
					if (point_pos.size())
					{
						std::vector<cv::Point> firbe_pos_temp;
						for (int i = 0; i < point_pos.size(); i++)
						{
							std::cout << "or: " << point_pos[i] << "  ";
							int i_i = Frisbee_many;
							while (i_i)
							{
								i_i--;
								int temp = abs(Frisbee_posi[i_i].x - point_pos[i].x) + 2 * abs(Frisbee_posi[i_i].y - point_pos[i].y);

								if (temp<20)
								{
									point_pos[i] = Frisbee_posi[i_i];
									std::cout << "same ";
									break;
								}
							}
							firbe_pos_temp.push_back(point_pos[i]);
						}
						for (int i = 0; i < point_pos.size(); i++)
						{
							Frisbee_posi[i] = point_pos[i];
						}
						Frisbee_many = point_pos.size();
						disc_signal = 1;
					}
					else
					{
						disc_signal = 0;
					}
					unsigned char getresult;

					//追踪飞盘算法
					//getresult = Track(point_pos, disc_signal, landboundary);

					//不追踪飞盘  看见就就控制电机动作
					getresult = NotTrack(point_pos, disc_signal, landboundary);

					//SendDataRegion(getresult);
					SendSignal(getresult);

					//std::cout << "result track  " << (int)getresult << std::endl;

					//识别效果进行简略描述
					if (getresult)
					{
						cv::ellipse(roi_image, cv::Point(roi_image.cols / 8, roi_image.rows / 2), Size(50, 30), 0, 0, 360, Scalar(255, 129, 0));
						cv::line(roi_image, cv::Point(roi_image.cols / 8 - 50, roi_image.rows / 2), cv::Point(roi_image.cols / 8 + 50, roi_image.rows / 2), cv::Scalar(255, 0, 255));
						cv::line(roi_image, cv::Point(roi_image.cols / 8 - 16, roi_image.rows / 2 - 30), cv::Point(roi_image.cols / 8 - 16, roi_image.rows / 2 + 30), cv::Scalar(255, 0, 255));
						cv::line(roi_image, cv::Point(roi_image.cols / 8 + 16, roi_image.rows / 2 - 30), cv::Point(roi_image.cols / 8 + 16, roi_image.rows / 2 + 30), cv::Scalar(255, 0, 255));
						for (int i = 0; i < 6; i++)
						{
							if (getresult & 1 << i)
							{
								cv::Point cen;
								int x, y;
								cen.x = roi_image.cols / 8 - 50 + 33 / 2 + i / 2 * 33;
								if (i % 2)
								{
									cen.y = roi_image.rows / 2 + 15;
								}
								else
								{
									cen.y = roi_image.rows / 2 - 15;
								}
								cv::circle(roi_image, cen, 5, Scalar(200, 200, 0));
							}
						}
					}


					//确定是否有自己的飞盘
					cv::Mat median_image_self;
					cv::medianBlur(self_frisbee, median_image_self, 5);
					cv::imshow("自己飞盘", median_image_self);

					std::vector<cv::Point> point_rect_self;
					//提取边界  但是对内部空洞未能实现滤除
					point_rect_self = fr.find_contours_te(median_image_self, frisbee_black, platform_7);
					if (point_rect_self.size())
					{
						int i_rect = point_rect_self.size() >> 2;
						for (int i = 0; i < i_rect; i++)
						{
							int cir_i = i * 4;
							cv::line(roi_image, point_rect_self[cir_i], point_rect_self[cir_i + 1], cv::Scalar(255, 0, 255));
							cv::line(roi_image, point_rect_self[cir_i + 1], point_rect_self[cir_i + 2], cv::Scalar(255, 0, 255));
							cv::line(roi_image, point_rect_self[cir_i + 2], point_rect_self[cir_i + 3], cv::Scalar(255, 0, 255));
							cv::line(roi_image, point_rect_self[cir_i + 3], point_rect_self[cir_i], cv::Scalar(255, 0, 255));
						}
#ifdef  SELF_HAVE_FRISBEE
						unsigned char send_self_have = 0x81;
						while (mySerialPort.WriteData(&send_self_have, 1) == false)
						{
							cv::waitKey(100);
						}
						while (mySerialPort.WriteData(&send_self_have, 1) == false)
						{
							cv::waitKey(100);
						}
						send_self_have = 0x91;
						while (mySerialPort.WriteData(&send_self_have, 1) == false)
						{
							cv::waitKey(100);
						}
						std::cout << "have self" << std::endl;
#endif //  SELF_HAVE_FRISBEE
					}
					else
					{
						;
#ifdef  SELF_HAVE_FRISBEE
						unsigned char send_self_have = 0x81;
						while (mySerialPort.WriteData(&send_self_have, 1) == false)
						{
							cv::waitKey(100);
						}
						while (mySerialPort.WriteData(&send_self_have, 1) == false)
						{
							cv::waitKey(100);
						}
						send_self_have = 0x90;
						while (mySerialPort.WriteData(&send_self_have, 1) == false)
						{
							cv::waitKey(100);
						}
						std::cout << "non self" << std::endl;
#endif //  SELF_HAVE_FRISBEE
					}

					cv::imshow("roi_image_fix", roi_image);

					//摄像头图像保存，存成一个视频
#ifdef USE_VIDEO_RECORD

					IplImage img2 = IplImage(roi_image);
					IplImage* img = &img2;
					if (!img)
					{
						printf("Could not load image file...n");
						exit(0);
					}
					cvShowImage("mainWin", img);
					if (cvWriteFrame(record_video.writer, img))
					{
						std::cout << "success" << std::endl;
					}
					else
					{
						std::cout << "fail" << std::endl;
					}
#endif // USE_VIDEO_RECORD
				}
				else if (cam_status == SELFCHECK_POS)
				{
					cv::line(original_image, cv::Point(cali_window.start_x, cali_window.start_y), cv::Point(cali_window.end_x, cali_window.start_y), cv::Scalar(255, 0, 255));
					cv::line(original_image, cv::Point(cali_window.end_x, cali_window.start_y), cv::Point(cali_window.end_x, cali_window.end_y), cv::Scalar(255, 0, 255));
					cv::line(original_image, cv::Point(cali_window.end_x, cali_window.end_y), cv::Point(cali_window.start_x, cali_window.end_y), cv::Scalar(255, 0, 255));
					cv::line(original_image, cv::Point(cali_window.start_x, cali_window.end_y), cv::Point(cali_window.start_x, cali_window.start_y), cv::Scalar(255, 0, 255));
					cv::imshow("original", original_image);
				}
				else if (cam_status == WhiteBalance)
				{
					std::vector<cv::Mat> chs(3);
					cv::split(original_image, chs);
					cv::Scalar mean_val = cv::mean(original_image);
					int b_mean = mean_val.val[0];
					int g_mean = mean_val.val[1];
					int r_mean = mean_val.val[2];
					std::cout << b_mean << "   " << g_mean << "   " << r_mean << "  change  ";

					int mean_bgr = b_mean + g_mean + r_mean;
					mean_bgr = mean_bgr / 3;

					int ratio_b = mean_bgr * 100 / b_mean;
					int ratio_g = mean_bgr * 100 / g_mean;
					int ratio_r = mean_bgr * 100 / r_mean;

					for (int i = 0; i < original_image.rows; ++i)
					{
						uchar *data = original_image.ptr<uchar>(i);
						for (int j = 0; j < original_image.cols; ++j)
						{
							int j_col = j * 3;
							int val_temp_b = ratio_b*data[j_col] / 100;
							int val_temp_g = ratio_g*data[j_col + 1] / 100;
							int val_temp_r = ratio_r*data[j_col + 2] / 100;
							val_temp_b = val_temp_b > 255 ? 255 : val_temp_b;
							val_temp_g = val_temp_g > 255 ? 255 : val_temp_g;
							val_temp_r = val_temp_r > 255 ? 255 : val_temp_r;

							data[j_col] = val_temp_b;
							data[j_col + 1] = val_temp_g;
							data[j_col + 2] = val_temp_r;
						}
					}

					cv::imshow("original_change", original_image);
					mean_val = cv::mean(original_image);
					b_mean = mean_val.val[0];
					g_mean = mean_val.val[1];
					r_mean = mean_val.val[2];
					std::cout << b_mean << "   " << g_mean << "   " << r_mean << "   " << std::endl;
				}
				else if (cam_status == Picture2Video)
				{
					char image_name[13];
					static int i = 0;
					sprintf(image_name, "%s%d%s", "image", ++i, ".jpg");
					IplImage img2 = IplImage(original_image);
					IplImage* img = &img2;

					if (!img)
					{
						printf("Could not load image file...n");
						exit(0);
					}
					cvShowImage("mainWin", img);
					cvWaitKey(20);
					if (cvWriteFrame(record_video.writer, img))
					{
						std::cout << "success" << std::endl;
					}
					else
					{
						std::cout << "fail" << std::endl;
					}

				}
				else if (cam_status == PaperWBInit)
				{
					static int first_change = 0;
					if (!first_change)
					{
						Gain_Analog_RGB_Init();
						set_expore(Expore_max_time);
						std::cout << "init ok only once " << std::endl;
						first_change = 1;
					}
					else
					{
						set_expore(Expore_par.exporeTime - 2000);
						int data_r = 0, data_g = 0, data_b = 0;
						int count_many = 0;
						for (int i = light_wb.first.y; i < light_wb.last.y; i++)
						{
							uchar *data = original_image.ptr<uchar>(i);
							for (int j = light_wb.first.x; j < light_wb.last.x; j++)
							{
								int temp = j * 3;
								data_b += data[temp];
								data_g += data[temp + 1];
								data_r += data[temp + 2];
								count_many++;
							}
						}
						data_b = data_b / count_many;
						data_g = data_g / count_many;
						data_r = data_r / count_many;
						if (data_b < 230 || data_g < 230 || data_r < 230)
						{
							set_expore(Expore_par.exporeTime);
							cam_status = PaperWhiteBalance;
							light_wb.complete_signal = 1;
							light_wb.b_prop_coeff = 220.0 / (1 + data_b);
							light_wb.g_prop_coeff = 220.0 / (1 + data_g);
							light_wb.r_prop_coeff = 220.0 / (1 + data_r);
							std::cout << "expore time  " << Expore_par.exporeTime << std::endl;
							std::cout << "prop coefficient " << light_wb.b_prop_coeff << "  " << light_wb.g_prop_coeff << "  " << light_wb.r_prop_coeff << std::endl;
						}
						Expore_par.exporeTime = Expore_par.exporeTime - 2000;
					}
					TimeStamp time_wati;
					time_wati.start();
					double stime = 0;
					double *pfExposureTime = &stime;
					//等待曝光的结束
					if (!CameraGetExposureTime(hCamera_cal, pfExposureTime))
						std::cout << "  check  " << stime << "  ";
					while (time_wati.runtime() < 300)
					{
						;
					}

					if (Expore_par.exporeTime < Expore_par.expore_time_end)
					{
						set_expore(Expore_par.exporeTime);
						cam_status = PaperWhiteBalance;
					}
					std::cout << std::endl;
				}
				else if (cam_status == PaperWhiteBalance)
				{
					cv::line(original_image, cv::Point(light_wb.first.x, light_wb.first.y), cv::Point(light_wb.last.x, light_wb.first.y), cv::Scalar(255, 0, 255));
					cv::line(original_image, cv::Point(light_wb.last.x, light_wb.first.y), cv::Point(light_wb.last.x, light_wb.last.y), cv::Scalar(255, 0, 255));
					cv::line(original_image, cv::Point(light_wb.last.x, light_wb.last.y), cv::Point(light_wb.first.x, light_wb.last.y), cv::Scalar(255, 0, 255));
					cv::line(original_image, cv::Point(light_wb.first.x, light_wb.last.y), cv::Point(light_wb.first.x, light_wb.first.y), cv::Scalar(255, 0, 255));
					cv::imshow("original_wb", original_image);
				}
				else if (cam_status == HistogramPracInit)
				{
					static int first_change = 0;
					if (!first_change)
					{
						Gain_Analog_RGB_Init();
						set_expore(Expore_max_time);
						std::cout << "init ok only once " << std::endl;
						first_change = 1;
					}
					else
					{
						int data_r = 0, data_g = 0, data_b = 0;
						int count_many = 0;
						for (int i = light_wb.first.y; i < light_wb.last.y; i++)
						{
							uchar *data = original_image.ptr<uchar>(i);
							for (int j = light_wb.first.x; j < light_wb.last.x; j++)
							{
								int temp = j * 3;
								data_b += data[temp];
								data_g += data[temp + 1];
								data_r += data[temp + 2];
								count_many++;
							}
						}
						data_b = data_b / count_many;
						data_g = data_g / count_many;
						data_r = data_r / count_many;

						int max_white_bgr = data_b + data_g + data_r;
						max_white_bgr = max_white_bgr / 3;


						cam_status = HistogramPrac;
						//cam_status = ONLY_DEFENSIVE;

						light_wb.complete_signal = 1;
						light_wb.b_prop_coeff = (float)max_white_bgr / (1 + data_b);
						light_wb.g_prop_coeff = (float)max_white_bgr / (1 + data_g);
						light_wb.r_prop_coeff = (float)max_white_bgr / (1 + data_r);
						std::cout << "prop coefficient " << light_wb.b_prop_coeff << "  " << light_wb.g_prop_coeff << "  " << light_wb.r_prop_coeff << std::endl;


					}
				}
				else if (cam_status == RGBjiaozheng)
				{
					static int first_change = 0;
					static int wb_com_signal = 0;
					static int light_signal = 0;

					if (!first_change)
					{
						Gain_Analog_RGB_Init();
						set_expore(Expore_max_time);
						std::cout << "init ok only once " << std::endl;
						first_change = 1;
					}
					else
					{
						int data_r = 0, data_g = 0, data_b = 0;
						int count_many = 0;
						int total_bgr = 0;
						if (!wb_com_signal)
						{
							for (int i = light_wb.first.y; i < light_wb.last.y; i++)
							{
								uchar *data = original_image.ptr<uchar>(i);
								for (int j = light_wb.first.x; j < light_wb.last.x; j++)
								{
									int temp = j * 3;
									int temp_total = data[temp] + data[temp + 1] + data[temp + 2];
									if (temp_total > total_bgr)
									{
										total_bgr = temp_total;
										data_b = data[temp];
										data_g = data[temp + 1];
										data_r = data[temp + 2];
									}
								}
							}

							//调整曝光时间参数 最亮的点b g r 三个值平均值在150-240 
							//调整好   light_signal=1;
							if (!light_signal)
							{
								//int max_bgr_val = data_b > data_g ? data_b : data_g;
								//max_bgr_val = max_bgr_val > data_r ? max_bgr_val : data_r;

								int max_bgr_val = data_b + data_g + data_r;
								max_bgr_val = max_bgr_val / 3;
								if (max_bgr_val < 190)
								{
									Expore_par.exporeTime = Expore_par.exporeTime + 2000;
									set_expore(Expore_par.exporeTime);
									if (Expore_par.exporeTime > 100000)
									{
										std::cout << "too dark" << std::endl;
									}
								}
								else if (max_bgr_val > 240)
								{
									Expore_par.exporeTime = Expore_par.exporeTime - 2000;
									set_expore(Expore_par.exporeTime);
									if (Expore_par.exporeTime < 5000)
									{
										std::cout << "too light" << std::endl;
									}
								}
								else
								{
									std::cout << "light ok" << std::endl;
									light_signal = 1;
								}




								if (Expore_par.exporeTime > 100000|| Expore_par.exporeTime < 5000)
								{
									Gain_Analog_RGB_Init();
									set_expore(Expore_max_time);
									std::cout << "use default parameter " << std::endl;
									cam_status = WAIT_ARRIVE;
								}








								//实际曝光时间获取
								double stime = 0;
								double *pfExposureTime = &stime;
								//等待曝光的结束
								if (!CameraGetExposureTime(hCamera_cal, pfExposureTime))
									std::cout << "  check  " << stime << "  ";
							}
							else
							{
								int min_bgr_val = data_b < data_g ? data_b : data_g;
								min_bgr_val = min_bgr_val < data_r ? min_bgr_val : data_r;
								int max_bgr_val = data_b > data_g ? data_b : data_g;
								max_bgr_val = max_bgr_val > data_r ? max_bgr_val : data_r;

								//调整白平衡   b g r三者最大最小 差值小于10 
								//分阶段调节  差值大，调节力度大  差值小，调节力度小
								if ((max_bgr_val - min_bgr_val) > 40)
								{
									if (max_bgr_val == data_b)
									{
										camera_gain.b -= 5;
									}
									else if (max_bgr_val == data_g)
									{
										camera_gain.g -= 5;
									}
									else
									{
										camera_gain.r -= 5;
									}

									if (!CameraSetGain(hCamera_cal, camera_gain.r, camera_gain.g, camera_gain.b))
									{
										std::cout << "rgb val b: " << data_b << "  g: " << data_g << "  r:" << data_r;
										std::cout << "   rgb gain ok b: " << camera_gain.b << "  g: " << camera_gain.g << "  r:" << camera_gain.r << std::endl;
									}
								}
								else if ((max_bgr_val - min_bgr_val) > 10)
								{
									if (min_bgr_val == data_b)
									{
										camera_gain.b += 5;
									}
									else if (min_bgr_val == data_g)
									{
										camera_gain.g += 5;
									}
									else
									{
										camera_gain.r += 5;
									}
									if (!CameraSetGain(hCamera_cal, camera_gain.r, camera_gain.g, camera_gain.b))
									{
										std::cout << "rgb val b: " << data_b << "  g: " << data_g << "  r:" << data_r;
										std::cout << "   rgb gain ok b: " << camera_gain.b << "  g: " << camera_gain.g << "  r:" << camera_gain.r << std::endl;
									}
								}
								else
								{
									wb_com_signal = 1;
									std::cout << "finaly gain ok" << std::endl;
									std::cout << "finaly gain ok" << std::endl;
									std::cout << "finaly gain ok" << std::endl;
									cam_status = WAIT_ARRIVE;
								}
							}
						}
					}

					TimeStamp time_wati;
					time_wati.start();

					while (time_wati.runtime() < 300)
					{
						;
					}
				}
				else if (cam_status == HistogramPrac)
				{
					if (gain_set_time2 < 2)
					{
						Gain_Analog_RGB_Init();
						set_expore(Expore_max_time);
						gain_set_time2++;
					}
					//直方图 英文 histogram
					//直方图均衡化 equalizehist
					cv::Mat gray_origin;
					cv::cvtColor(original_image, gray_origin, CV_BGR2GRAY);
					cv::imshow("gray_ori", gray_origin);
					cv::equalizeHist(gray_origin, gray_origin);
					cv::imshow("gray_equalize", gray_origin);

					//cv::vector<cv::Mat> chs(3);
					cv::Mat chs[3];
					cv::split(original_image, chs);
					cv::imshow("blue", chs[0]);
					int Histsize = 256;
					float range[] = { 0,256 };
					const float *histrange = { range };
					cv::Mat b_hist, g_hist, r_hist;
					cv::calcHist(&chs[0], 1, 0, cv::Mat(), b_hist, 1, &Histsize, &histrange, true, false);
					cv::calcHist(&chs[1], 1, 0, cv::Mat(), g_hist, 1, &Histsize, &histrange, true, false);
					cv::calcHist(&chs[2], 1, 0, cv::Mat(), r_hist, 1, &Histsize, &histrange, true, false);

					int hist_height = 400;
					int hist_width = 512;
					int bin_width = hist_width / Histsize;
					cv::Mat histImage(hist_height, hist_width, CV_8UC3, Scalar(0, 0, 0));
					cv::Mat histImage_b(hist_height, hist_width, CV_8UC3, Scalar(0, 0, 0));
					cv::Mat histImage_g(hist_height, hist_width, CV_8UC3, Scalar(0, 0, 0));
					cv::Mat histImage_r(hist_height, hist_width, CV_8UC3, Scalar(0, 0, 0));

					cv::normalize(b_hist, b_hist, 0, hist_height, NORM_MINMAX, -1, cv::Mat());
					cv::normalize(g_hist, g_hist, 0, hist_height, NORM_MINMAX, -1, cv::Mat());
					cv::normalize(r_hist, r_hist, 0, hist_height, NORM_MINMAX, -1, cv::Mat());

					for (int i = 1; i < Histsize; i++)
					{
						cv::line(histImage, cv::Point((i - 1)*bin_width, hist_height - cvRound(b_hist.at<float>(i - 1))),
							cv::Point((i)*bin_width, hist_height - cvRound(b_hist.at<float>(i))), Scalar(255, 0, 0), 2);
						cv::line(histImage, cv::Point((i - 1)*bin_width, hist_height - cvRound(g_hist.at<float>(i - 1))),
							cv::Point((i)*bin_width, hist_height - cvRound(g_hist.at<float>(i))), Scalar(0, 255, 0), 2);
						cv::line(histImage, cv::Point((i - 1)*bin_width, hist_height - cvRound(r_hist.at<float>(i - 1))),
							cv::Point((i)*bin_width, hist_height - cvRound(r_hist.at<float>(i))), Scalar(0, 0, 255), 2);

						cv::line(histImage_b, cv::Point((i - 1)*bin_width, hist_height - cvRound(b_hist.at<float>(i - 1))),
							cv::Point((i)*bin_width, hist_height - cvRound(b_hist.at<float>(i))), Scalar(255, 0, 0), 2);
						cv::line(histImage_g, cv::Point((i - 1)*bin_width, hist_height - cvRound(g_hist.at<float>(i - 1))),
							cv::Point((i)*bin_width, hist_height - cvRound(g_hist.at<float>(i))), Scalar(0, 255, 0), 2);
						cv::line(histImage_r, cv::Point((i - 1)*bin_width, hist_height - cvRound(r_hist.at<float>(i - 1))),
							cv::Point((i)*bin_width, hist_height - cvRound(r_hist.at<float>(i))), Scalar(0, 0, 255), 2);

					}
					cv::imshow("histImage", histImage);
					cv::imshow("histImage_b", histImage_b);
					cv::imshow("histImage_g", histImage_g);
					cv::imshow("histImage_r", histImage_r);


				}
				else if (cam_status == START_INIT)
				{
					cv::Mat frame_plat7, plat7_hsv;
					cv::Mat outer_edges_image7, median_image7;

					frame_plat7 = original_image;
					cv::cvtColor(frame_plat7, plat7_hsv, cv::COLOR_BGR2HSV);

					TimeStamp time_wati;
					time_wati.start();
					auto hsv_thr = thresholdHSV(plat7_hsv, platform_7.v_val, platform_7.s_val);
					cv::medianBlur(hsv_thr, median_image7, 5);
					cv::imshow("median_image7", median_image7);
					cv::Canny(median_image7, outer_edges_image7, 3, 9, 3);
					cv::imshow("outer_edges_image7", outer_edges_image7);

					double stime = 0;
					double *pfExposureTime = &stime;
					//等待曝光的结束
					if (!CameraGetExposureTime(hCamera_cal, pfExposureTime))
						std::cout << "  check  " << stime << "  ";

					std::cout << std::endl;

					set_expore(Expore_par.exporeTime - 2000);

					time_5_sig++;
					MatchValAng val_ang;
					val_ang = fr.MatchPartNearInit(outer_edges_image7, frame_plat7, platform_7);
					if (time_5_sig > 2 && val_ang.maxValue > Expore_par.maxValue_value&&JudgeInthresholdRange(platform_7, val_ang.angle))
					{
						Expore_par.maxValue_exporeTime = Expore_par.exporeTime;
						Expore_par.maxValue_value = val_ang.maxValue;
						Expore_par.maxValue_angle = val_ang.angle;
					}
					Expore_par.exporeTime -= 2000;
					std::cout << val_ang.maxValue << " " << std::endl;

					while (time_wati.runtime() < 200)
					{
						;
					}

					if (Expore_par.exporeTime < Expore_par.expore_time_end)
					{
						cam_status = WAIT_ARRIVE;
						set_expore((Expore_par.maxValue_exporeTime));
						std::cout << Expore_par.maxValue_exporeTime << "set  t " << Expore_par.maxValue_value << "  ang " << Expore_par.maxValue_angle << std::endl;
						platform_7.angle = Expore_par.maxValue_angle;
#ifdef INIT_SIGNAL
						for (int ii = 0; ii < 100000; ii++)
						{
							unsigned char send_data_syn = 0x77;
							while (mySerialPort.WriteData(&send_data_syn, 1) == false)
							{
								cv::waitKey(100);
							}
						}
#endif
					}
				}
				else if (cam_status == TEXTMATCH)
				{
					cv::Mat frame_plat7, plat7_hsv;
					cv::Mat outer_edges_image7, median_image7;

					frame_plat7 = original_image;
					cv::cvtColor(frame_plat7, plat7_hsv, cv::COLOR_BGR2HSV);


					auto hsv_thr = thresholdHSV(plat7_hsv, platform_7.v_val, platform_7.s_val);
					cv::medianBlur(hsv_thr, median_image7, 5);
					cv::imshow("median_image7", median_image7);
					cv::Canny(median_image7, outer_edges_image7, 3, 9, 3);
					cv::imshow("outer_edges_image7", outer_edges_image7);

					double stime = 0;
					double *pfExposureTime = &stime;
					//等待曝光的结束
					if (!CameraGetExposureTime(hCamera_cal, pfExposureTime))
						std::cout << "  check  " << stime << "  ";

					time_5_sig++;
					MatchValAng val_ang;
					val_ang = fr.MatchPartNearInit(outer_edges_image7, frame_plat7, platform_7);

					std::cout << val_ang.maxValue << " " << std::endl;
				}
				else if (cam_status == SHICHANG)
				{
					//首先将参考增益进行设定
					static int first_change = 0;
					if (!first_change)
					{
						Gain_Analog_RGB_Init();
						set_expore(Expore_max_time);
						std::cout << "init ok only once " << std::endl;
						first_change = 1;
					}
					cv::imshow("original", original_image);



					cv::Mat frame_plat7;
					cv::Mat outer_edges_image, median_image7;

					frame_plat7 = original_image;

					auto hsv_thr = thresholdHSV(fr.getHSVImage(), platform_7.v_val, platform_7.s_val);

					cv::medianBlur(hsv_thr, median_image7, 5);
					cv::imshow("median_image7", median_image7);


					cv::Canny(median_image7, outer_edges_image, 3, 9, 3);
					cv::imshow("outer_edges_image", outer_edges_image);

					platform_7.center = fr.MatchPartNearPlat(outer_edges_image, frame_plat7, platform_7.angle, platform_7.height, platform_7.width);

					static int count_center_send = 0;
					count_center_send++;
					if (count_center_send == 10)
					{
						count_center_send = 0;
						std::cout << "center:  " << platform_7.center << std::endl;
					}

					cv::Mat frisbee_bin_img;
					cv::Mat self_frisbee;
					auto roi_image = original_image;

					auto frisbee_black = fr.thresholdByHSVpanzi([](cv::Vec3b* p)->bool { return (*p)[1] < black_s && (*p)[2] < black_v; });
					cv::medianBlur(frisbee_black, frisbee_black, 5);

					cv::imshow("frisbee_black", frisbee_black);

					if (color_reg == 'r')
					{
						frisbee_bin_img = fr.thresholdByHSVpanzi([](cv::Vec3b* p)->bool { return (((*p)[0] < blue_h_big) && ((*p)[0] > blue_h_small) && (*p)[1] > blue_s && (*p)[2] > blue_v); });//蓝色
						self_frisbee = fr.thresholdByHSVpanzi([](cv::Vec3b* p)->bool { return ((((*p)[0] > red_h_big || (*p)[0] < red_h_small) && (*p)[1] > red_s && (*p)[2] > red_v)); });//红色
					}
					else
					{
						self_frisbee = fr.thresholdByHSVpanzi([](cv::Vec3b* p)->bool { return (((*p)[0] < blue_h_big) && ((*p)[0] > blue_h_small) && (*p)[1] > blue_s && (*p)[2] > blue_v); });//蓝色
						frisbee_bin_img = fr.thresholdByHSVpanzi([](cv::Vec3b* p)->bool { return ((((*p)[0] > red_h_big || (*p)[0] < red_h_small) && (*p)[1] > red_s && (*p)[2] > red_v)); });//红色
					}
					//防守  识别敌人的飞盘
					cv::Mat median_image_roi;
					cv::medianBlur(frisbee_bin_img, median_image_roi, 5);
					cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
					cv::morphologyEx(median_image_roi, median_image_roi, cv::MORPH_OPEN, element);


					cv::imshow("敌人飞盘", median_image_roi);

					std::vector<cv::Point> point_rect;
					//提取边界  但是对内部空洞未能实现滤除
					point_rect = fr.find_contours_te(median_image_roi, frisbee_black, platform_7);

					std::vector<cv::Point> point_pos;
					if (point_rect.size())
					{
						int i_rect = point_rect.size() >> 2;
						for (int i = 0; i < i_rect; i++)
						{
							int cir_i = i * 4;
							//cv::line(roi_image, point_rect[cir_i], point_rect[cir_i + 1], cv::Scalar(0, 255, 255));
							//cv::line(roi_image, point_rect[cir_i + 1], point_rect[cir_i + 2], cv::Scalar(0, 255, 255));
							//cv::line(roi_image, point_rect[cir_i + 2], point_rect[cir_i + 3], cv::Scalar(0, 255, 255));
							//cv::line(roi_image, point_rect[cir_i + 3], point_rect[cir_i], cv::Scalar(0, 255, 255));
							cv::line(roi_image, point_rect[cir_i], point_rect[cir_i + 1], cv::Scalar(0, 0, 0));
							cv::line(roi_image, point_rect[cir_i + 1], point_rect[cir_i + 2], cv::Scalar(0, 0, 0));
							cv::line(roi_image, point_rect[cir_i + 2], point_rect[cir_i + 3], cv::Scalar(0, 0, 0));
							cv::line(roi_image, point_rect[cir_i + 3], point_rect[cir_i], cv::Scalar(0, 0, 0));
							int x = (point_rect[cir_i].x + point_rect[cir_i + 2].x) >> 1;
							int y = (point_rect[cir_i].y + point_rect[cir_i + 2].y) >> 1;
							cv::Point send = cv::Point(x, y);
							//cv::circle(roi_image, send, 3, cv::Scalar(0, 220, 0));
							point_pos.push_back(send);
						}
					}


					uchar send_data_comb = 0;

					cv::Point image_st = cv::Point((platform_7.center.x - platform_7.height / 2), (platform_7.center.y - platform_7.width / 2));
					cv::circle(roi_image, image_st, 3, cv::Scalar(200, 0, 0));



					//分区界限划分
					int  row_2_3;
					int row_left = 0, row_middle = 0, row_right = 0;
					int col_1_3, col_2_3, region_num = 0;
					col_1_3 = platform_7.center.x - platform_7.height / 6;
					col_2_3 = platform_7.center.x + platform_7.height / 6;
					row_2_3 = platform_7.center.y + platform_7.width *0.1;

					row_left = image_st.y + platform_7.width *0.57;
					row_middle = image_st.y + platform_7.width *0.63;
					row_right = image_st.y + platform_7.width *0.57;

					if (abs(one_col - col_1_3) < 10)
					{
						col_1_3 = one_col;
					}
					else
					{
						one_col = col_1_3;
					}
					if (abs(two_col - col_2_3) < 10)
					{
						col_2_3 = two_col;
					}
					else
					{
						two_col = col_2_3;
					}

					if (abs(row_l - row_left) < 10)
					{
						row_left = row_l;
					}
					else
					{
						row_l = row_left;
					}

					if (abs(row_m - row_middle) < 10)
					{
						row_middle = row_m;
					}
					else
					{
						row_m = row_middle;
					}

					if (abs(row_r - row_right) < 10)
					{
						row_right = row_r;
					}
					else
					{
						row_r = row_right;
					}
					if (abs(two_row - row_2_3) < 10)
					{
						row_2_3 = two_row;
					}
					else
					{
						two_row = row_2_3;
					}

					one_col = col_1_3;
					two_col = col_2_3;
					two_row = row_2_3;
					row_r = row_right;
					row_m = row_middle;
					row_l = row_left;
					LandBoundary landboundary;
					landboundary.one_col = col_1_3;
					landboundary.two_col = col_2_3;
					landboundary.two_row = row_2_3;
					landboundary.row_r = row_right;
					landboundary.row_m = row_middle;
					landboundary.row_l = row_left;


					//std::cout << " lie " << col_1_3 << " " << col_2_3 << " " << row_l << "  " << row_m << "  " << row_r << "  ";

					//飞盘中心点的位置加了一个滤波 
					//连续两个周期位置 x差20 y差10 以内认为是同一个点
					static int disc_signal = 0;
					if (point_pos.size())
					{
						std::vector<cv::Point> firbe_pos_temp;
						for (int i = 0; i < point_pos.size(); i++)
						{
							std::cout << "or: " << point_pos[i] << "  ";
							int i_i = Frisbee_many;
							while (i_i)
							{
								i_i--;
								int temp = abs(Frisbee_posi[i_i].x - point_pos[i].x) + 2 * abs(Frisbee_posi[i_i].y - point_pos[i].y);

								if (temp<20)
								{
									point_pos[i] = Frisbee_posi[i_i];
									std::cout << "same ";
									break;
								}
							}
							firbe_pos_temp.push_back(point_pos[i]);
						}
						for (int i = 0; i < point_pos.size(); i++)
						{
							Frisbee_posi[i] = point_pos[i];
						}
						Frisbee_many = point_pos.size();
						disc_signal = 1;
					}
					else
					{
						disc_signal = 0;
					}
					unsigned char getresult;

					//不追踪飞盘  看见就就控制电机动作
					getresult = NotTrack(point_pos, disc_signal, landboundary);

					SendSignal(getresult);

					if (getresult)
					{
						cv::ellipse(roi_image, cv::Point(roi_image.cols / 8, roi_image.rows / 2), Size(50, 30), 0, 0, 360, Scalar(255, 129, 0));
						cv::line(roi_image, cv::Point(roi_image.cols / 8 - 50, roi_image.rows / 2), cv::Point(roi_image.cols / 8 + 50, roi_image.rows / 2), cv::Scalar(255, 0, 255));
						cv::line(roi_image, cv::Point(roi_image.cols / 8 - 16, roi_image.rows / 2 - 30), cv::Point(roi_image.cols / 8 - 16, roi_image.rows / 2 + 30), cv::Scalar(255, 0, 255));
						cv::line(roi_image, cv::Point(roi_image.cols / 8 + 16, roi_image.rows / 2 - 30), cv::Point(roi_image.cols / 8 + 16, roi_image.rows / 2 + 30), cv::Scalar(255, 0, 255));
						for (int i = 0; i < 6; i++)
						{
							if (getresult & 1 << i)
							{
								cv::Point cen;
								int x, y;
								cen.x = roi_image.cols / 8 - 50 + 33 / 2 + i / 2 * 33;
								if (i % 2)
								{
									cen.y = roi_image.rows / 2 + 15;
								}
								else
								{
									cen.y = roi_image.rows / 2 - 15;
								}
								cv::circle(roi_image, cen, 5, Scalar(200, 200, 0));
							}
						}
					}

					//确定是否有自己的飞盘
					cv::Mat median_image_self;
					cv::medianBlur(self_frisbee, median_image_self, 5);
					cv::imshow("自己飞盘", median_image_self);

					std::vector<cv::Point> point_rect_self;
					//提取边界  但是对内部空洞未能实现滤除
					point_rect_self = fr.find_contours_te(median_image_self, frisbee_black, platform_7);
					if (point_rect_self.size())
					{
						int i_rect = point_rect_self.size() >> 2;
						for (int i = 0; i < i_rect; i++)
						{
							int cir_i = i * 4;
							cv::line(roi_image, point_rect_self[cir_i], point_rect_self[cir_i + 1], cv::Scalar(255, 0, 255));
							cv::line(roi_image, point_rect_self[cir_i + 1], point_rect_self[cir_i + 2], cv::Scalar(255, 0, 255));
							cv::line(roi_image, point_rect_self[cir_i + 2], point_rect_self[cir_i + 3], cv::Scalar(255, 0, 255));
							cv::line(roi_image, point_rect_self[cir_i + 3], point_rect_self[cir_i], cv::Scalar(255, 0, 255));
						}

						self_pan_signal = 1;
					}
					else
					{
						self_pan_signal = 0;
					}
					cv::imshow("roi_image_fix", roi_image);
				}
				else if (cam_status == RECOGNISE_Disc)
				{
					//首先将参考增益进行设定
					static int first_change = 0;
					if (!first_change)
					{
						Gain_Analog_RGB_Init();
						set_expore(Expore_max_time);
						std::cout << "init ok" << std::endl;
						first_change = 1;
					}
					cv::imshow("original", original_image);



					cv::Mat frame_plat7;
					cv::Mat outer_edges_image, median_image7;

					frame_plat7 = original_image;

					auto hsv_thr = thresholdHSV(fr.getHSVImage(), platform_7.v_val, platform_7.s_val);

					cv::medianBlur(hsv_thr, median_image7, 5);
					cv::imshow("median_image7", median_image7);


					cv::Canny(median_image7, outer_edges_image, 3, 9, 3);
					cv::imshow("outer_edges_image", outer_edges_image);

					platform_7.center = fr.MatchPartNearPlat(outer_edges_image, frame_plat7, platform_7.angle, platform_7.height, platform_7.width);


					cv::Mat frisbee_bin_img;
					cv::Mat self_frisbee;
					auto roi_image = original_image;

					auto frisbee_black = fr.thresholdByHSVpanzi([](cv::Vec3b* p)->bool { return (*p)[1] < black_s && (*p)[2] < black_v; });
					cv::medianBlur(frisbee_black, frisbee_black, 5);

					cv::imshow("frisbee_black", frisbee_black);

					if (color_reg == 'r')
					{
						frisbee_bin_img = fr.thresholdByHSVpanzi([](cv::Vec3b* p)->bool { return (((*p)[0] < blue_h_big) && ((*p)[0] > blue_h_small) && (*p)[1] > blue_s && (*p)[2] > blue_v); });//蓝色
						self_frisbee = fr.thresholdByHSVpanzi([](cv::Vec3b* p)->bool { return ((((*p)[0] > red_h_big || (*p)[0] < red_h_small) && (*p)[1] > red_s && (*p)[2] > red_v)); });//红色
					}
					else
					{
						self_frisbee = fr.thresholdByHSVpanzi([](cv::Vec3b* p)->bool { return (((*p)[0] < blue_h_big) && ((*p)[0] > blue_h_small) && (*p)[1] > blue_s && (*p)[2] > blue_v); });//蓝色
						frisbee_bin_img = fr.thresholdByHSVpanzi([](cv::Vec3b* p)->bool { return ((((*p)[0] > red_h_big || (*p)[0] < red_h_small) && (*p)[1] > red_s && (*p)[2] > red_v)); });//红色
					}
					//防守  识别敌人的飞盘
					cv::Mat median_image_roi;
					cv::medianBlur(frisbee_bin_img, median_image_roi, 5);
					cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
					cv::morphologyEx(median_image_roi, median_image_roi, cv::MORPH_OPEN, element);


					cv::imshow("敌人飞盘", median_image_roi);

					//确定是否有自己的飞盘
					cv::Mat median_image_self;
					cv::medianBlur(self_frisbee, median_image_self, 5);
					cv::imshow("自己飞盘", median_image_self);
					//juse
					std::vector<cv::Point> point_rect_self;
					//提取边界  但是对内部空洞未能实现滤除
					point_rect_self = fr.find_contours_model(median_image_self, frisbee_black, platform_7);
					if (point_rect_self.size())
					{
						int i_rect = point_rect_self.size() >> 2;
						for (int i = 0; i < i_rect; i++)
						{
							int cir_i = i * 4;
							cv::line(roi_image, point_rect_self[cir_i], point_rect_self[cir_i + 1], cv::Scalar(255, 0, 255));
							cv::line(roi_image, point_rect_self[cir_i + 1], point_rect_self[cir_i + 2], cv::Scalar(255, 0, 255));
							cv::line(roi_image, point_rect_self[cir_i + 2], point_rect_self[cir_i + 3], cv::Scalar(255, 0, 255));
							cv::line(roi_image, point_rect_self[cir_i + 3], point_rect_self[cir_i], cv::Scalar(255, 0, 255));
						}

						self_pan_signal = 1;
					}
					else
					{
						self_pan_signal = 0;
					}
					cv::imshow("roi_image_fix", roi_image);

				}
				else if (cam_status == ONLY_ORIGIN)
				{
					if (gain_signal)
					{
						gain_signal = 0;
						if (!CameraSetGain(hCamera_cal, gain_r, gain_g, gain_b))
						{
							std::cout << "rgb gain ok" << std::endl;;
						}
						if (!CameraSetAnalogGain(hCamera_cal, gain_analog_total))
						{
							std::cout << "analog gain ok" << std::endl;;
						}
					}
					cv::imshow("original", original_image);
				}
				else if (cam_status == DIVIDE_REGION)
				{
					int cout_endl_signal = 0;
					cv::Mat frame_plat7;
					cv::Mat outer_edges_image, median_image7;

					frame_plat7 = original_image;
					auto hsv_thr = thresholdHSV(fr.getHSVImage(), platform_7.v_val, platform_7.s_val);
					cv::medianBlur(hsv_thr, median_image7, 5);
					cv::imshow("median_image7", median_image7);
					cv::Canny(median_image7, outer_edges_image, 3, 9, 3);
					cv::imshow("outer_edges_image", outer_edges_image);


					platform_7.center = fr.MatchPartNearPlat(outer_edges_image, frame_plat7, platform_7.angle, platform_7.height, platform_7.width);
					std::cout << "cen: " << platform_7.center << "  ";
					cv::Mat frisbee_bin_img;
					cv::Mat self_frisbee;
					auto roi_image = original_image;

					auto frisbee_black = fr.thresholdByHSVpanzi([](cv::Vec3b* p)->bool { return (*p)[1] < black_s && (*p)[2] < black_v; });
					cv::medianBlur(frisbee_black, frisbee_black, 5);

					cv::imshow("frisbee_black", frisbee_black);

					if (color_reg == 'r')
					{
						frisbee_bin_img = fr.thresholdByHSVpanzi([](cv::Vec3b* p)->bool { return (((*p)[0] < blue_h_big) && ((*p)[0] > blue_h_small) && (*p)[1] > blue_s && (*p)[2] > blue_v); });//蓝色
						self_frisbee = fr.thresholdByHSVpanzi([](cv::Vec3b* p)->bool { return ((((*p)[0] > red_h_big || (*p)[0] < red_h_small) && (*p)[1] > red_s && (*p)[2] > red_v)); });//红色
					}
					else
					{
						self_frisbee = fr.thresholdByHSVpanzi([](cv::Vec3b* p)->bool { return (((*p)[0] < blue_h_big) && ((*p)[0] > blue_h_small) && (*p)[1] > blue_s && (*p)[2] > blue_v); });//蓝色
						frisbee_bin_img = fr.thresholdByHSVpanzi([](cv::Vec3b* p)->bool { return ((((*p)[0] > red_h_big || (*p)[0] < red_h_small) && (*p)[1] > red_s && (*p)[2] > red_v)); });//红色
					}


					//防守  识别敌人的飞盘
					cv::Mat median_image_roi;
					cv::medianBlur(frisbee_bin_img, median_image_roi, 5);
					cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
					cv::morphologyEx(median_image_roi, median_image_roi, cv::MORPH_OPEN, element);


					cv::imshow("敌人飞盘", median_image_roi);

					std::vector<cv::Point> point_rect;
					//提取边界  但是对内部空洞未能实现滤除
					point_rect = fr.find_contours_te(median_image_roi, frisbee_black, platform_7);

					std::vector<cv::Point> point_pos;
					if (point_rect.size())
					{
						int i_rect = point_rect.size() >> 2;
						for (int i = 0; i < i_rect; i++)
						{
							int cir_i = i * 4;
							cv::line(roi_image, point_rect[cir_i], point_rect[cir_i + 1], cv::Scalar(0, 255, 255));
							cv::line(roi_image, point_rect[cir_i + 1], point_rect[cir_i + 2], cv::Scalar(0, 255, 255));
							cv::line(roi_image, point_rect[cir_i + 2], point_rect[cir_i + 3], cv::Scalar(0, 255, 255));
							cv::line(roi_image, point_rect[cir_i + 3], point_rect[cir_i], cv::Scalar(0, 255, 255));

							int x = (point_rect[cir_i].x + point_rect[cir_i + 2].x) >> 1;
							int y = (point_rect[cir_i].y + point_rect[cir_i + 2].y) >> 1;
							cv::Point send = cv::Point(x, y);
							point_pos.push_back(send);
						}
					}
					cv::Point image_st = cv::Point((platform_7.center.x - platform_7.height / 2), (platform_7.center.y - platform_7.width / 2));
					cv::circle(roi_image, image_st, 3, cv::Scalar(200, 0, 0));

					int  row_2_3 = 0;
					int col_1_3, col_2_3, region_num = 0;
					col_1_3 = platform_7.center.x - platform_7.height / 6;
					col_2_3 = platform_7.center.x + platform_7.height / 6;

					int row_left, row_middle, row_right;
					row_left = image_st.y + platform_7.width *0.5;
					row_middle = image_st.y + platform_7.width *0.63;
					row_right = image_st.y + platform_7.width *0.5;

					cv::line(roi_image, cv::Point(image_st.x, row_left), cv::Point(col_1_3, row_left), cv::Scalar(0, 0, 0));
					cv::line(roi_image, cv::Point(col_1_3, row_middle), cv::Point(col_2_3, row_middle), cv::Scalar(0, 0, 0));
					cv::line(roi_image, cv::Point(col_2_3, row_right), cv::Point(image_st.x + platform_7.height, row_right), cv::Scalar(0, 0, 0));

					if (abs(one_col - col_1_3) < 10)
					{
						col_1_3 = one_col;
					}
					else
					{
						one_col = col_1_3;
					}
					if (abs(two_col - col_2_3) < 10)
					{
						col_2_3 = two_col;
					}
					else
					{
						two_col = col_2_3;
					}
					if (abs(two_row - row_2_3) < 10)
					{
						row_2_3 = two_row;
					}
					else
					{
						two_row = row_2_3;
					}

					one_col = col_1_3;
					two_col = col_2_3;
					two_row = row_2_3;

					std::cout << " lie " << col_1_3 << " " << col_2_3 << " " << row_2_3 << " ";

					int num = 0;
					uchar send_data_comb = 0;
					//左面50   中间63   右面50
#ifdef REGION_DIV_OLD_METHOD
					if (point_pos.size())
					{
						for (int i = 0; i < point_pos.size(); i++)
						{
							cv::circle(roi_image, point_pos[i], 3, cv::Scalar(0, 200, 200));
							int x = (point_pos[i].x - image_st.x) * 100 / platform_7.height;
							int y = (point_pos[i].y - image_st.y) * 100 / platform_7.width;
							std::cout << "  bili " << x << " " << y << "  ";
						}
						std::cout << std::endl;

						std::vector<cv::Point> firbe_pos_temp;

						for (int i = 0; i < point_pos.size(); i++)
						{
							std::cout << "or: " << point_pos[i] << "  ";
						}
						std::cout << std::endl;

					}

					cv::imshow("roi_image_fix", roi_image);
#endif
				}
				else if (cam_status == MANU_DEFEND)
				{
					unsigned char last_have = 0;
					int row_2_3 = original_image.rows / 2 + 50;

					//if (plot_x < original_image.cols / 3)
					//{
					//	if (plot_y < row_2_3)
					//		last_have = 1;
					//	else
					//		last_have = 2;
					//}
					//else if (plot_x < original_image.cols / 3*2)
					//{
					//	if (plot_y < row_2_3)
					//		last_have = 4;
					//	else
					//		last_have = 8;
					//}
					//else
					//{
					//	if (plot_y < row_2_3)
					//		last_have = 16;
					//	else
					//		last_have = 32;
					//}

					cv::line(original_image, cv::Point(459, 0), cv::Point(459, 600), cv::Scalar(255, 0, 255));
					cv::line(original_image, cv::Point(537, 0), cv::Point(537, 600), cv::Scalar(255, 0, 255));
					cv::line(original_image, cv::Point(200, row_2_3), cv::Point(999, row_2_3), cv::Scalar(255, 0, 255));

					cv::imshow("original", original_image);
					int return_key = 0;
					return_key = cv::waitKey(30);
					if (return_key == 49)
					{
						last_have = 2;
						signal_shot = 1;
					}
					else if (return_key == 50)
					{
						last_have = 8;
						signal_shot = 1;
					}
					else if (return_key == 51)
					{
						last_have = 32;
						signal_shot = 1;
					}
					else if (return_key == 52)
					{
						last_have = 1;
						signal_shot = 1;
					}
					else if (return_key == 53)
					{
						last_have = 4;
						signal_shot = 1;
					}
					else if (return_key == 54)
					{
						last_have = 16;
						signal_shot = 1;
					}

					/*if (plot_x < 459)
					{
					if (plot_y < row_2_3)
					last_have = 1;
					else
					last_have = 2;
					}
					else if (plot_x < 537)
					{
					if (plot_y < row_2_3)
					last_have = 4;
					else
					last_have = 8;
					}
					else
					{
					if (plot_y < row_2_3)
					last_have = 16;
					else
					last_have = 32;
					}*/
					if (signal_shot)
					{
						signal_shot = 0;
						unsigned char send_data_syn = NEAR_PLAT_SYN_BYTE;
						while (mySerialPort.WriteData(&send_data_syn, 1) == false)
						{
							cv::waitKey(10);
						}
						while (mySerialPort.WriteData(&send_data_syn, 1) == false)
						{
							cv::waitKey(10);
						}
						while (mySerialPort.WriteData(&last_have, 1) == false)
						{
							cv::waitKey(10);
						}
					}
				}
				else if (cam_status == WAIT_ARRIVE)
				{
					if (GetSelf_che_signal())
					{
						cam_status = SELFCHECK_POS;
					}
					if (car_stop)
					{
						cam_status = ONLY_DEFENSIVE;
						std::cout << "car stop " << std::endl;
					}
					static int signal_frency = 0;
					signal_frency++;
					if (signal_frency > 8)
					{
#ifdef INIT_SIGNAL
						unsigned char send_data_syn = 0x77;
						while (mySerialPort.WriteData(&send_data_syn, 1) == false)
						{
							cv::waitKey(100);
						}
#endif // INIT_SIGNAL


						signal_frency = 0;
					}

				}
				else
				{
					;
				}
			}//--status

			 //在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
			 //否则再次调用CameraGetImageBuffer时，程序将被挂起，知道其他线程中调用CameraReleaseImageBuffer来释放了buffer
			CameraReleaseImageBuffer(hCamera, pbyBuffer);
		}//while
		if (cv::waitKey(1) == 27)
		{
			m_bExit = TRUE;
			out_stream.close();
			out_hsv_stream.close();
			out_every_data.close();
			cvReleaseVideoWriter(&record_video.writer);

		}

#ifdef  EACH_FRAME_TIME
		std::cout << "a frame: " << A_Frame_Time.runtime() << std::endl;
		A_Frame_Time.start();
#endif //  EACH_FRAME_TIME


	}

	_endthreadex(0);
	return 0;
}

int main(int argc, char* argv[])
{
	tSdkCameraDevInfo sCameraList[20];
	INT iCameraNums;
	CameraSdkStatus status;
	tSdkCameraCapbility sCameraInfo;


#ifdef USB_USART
	if (!mySerialPort.InitPort(5, CBR_115200))
	{
		std::cout << "initPort fail !" << std::endl;
	}
	else
	{
		std::cout << "initPort success !" << std::endl;
	}

	if (!mySerialPort.OpenListenThread())
	{
		std::cout << "OpenListenThread fail !" << std::endl;
	}
	else
	{
		std::cout << "OpenListenThread success !" << std::endl;
	}
#endif

	//CameraSdkInit(1);
	//枚举设备，获得设备列表
	iCameraNums = 10;//调用CameraEnumerateDevice前，先设置iCameraNums = 10，表示最多只读取10个设备，如果需要枚举更多的设备，请更改sCameraList数组的大小和iCameraNums的值
	if (CameraEnumerateDevice(sCameraList, &iCameraNums) != CAMERA_STATUS_SUCCESS || iCameraNums == 0)
	{
		printf("No camera was found!");
		return 1;
	}

	//该示例中，我们只假设连接了一个相机。因此，只初始化第一个相机。(-1,-1)表示加载上次退出前保存的参数，如果是第一次使用该相机，则加载默认参数.
	//In this demo ,we just init the first camera.
	if ((status = CameraInit(&sCameraList[0], -1, -1, &m_hCamera)) != CAMERA_STATUS_SUCCESS)
	{
		printf("Failed to init the camera! Error code is %d", status);
		return FALSE;
	}

	std::cout << "wait" << std::endl;
	//Get properties description for this camera.
	CameraGetCapability(m_hCamera, &sCameraInfo);

	m_pFrameBuffer = (BYTE *)CameraAlignMalloc(sCameraInfo.sResolutionRange.iWidthMax*sCameraInfo.sResolutionRange.iWidthMax * 3, 16);

	if (sCameraInfo.sIspCapacity.bMonoSensor)
	{
		CameraSetIspOutFormat(m_hCamera, CAMERA_MEDIA_TYPE_MONO8);
	}

	strcpy_s(g_CameraName, sCameraList[0].acFriendlyName);

	//属性设置页面
	CameraCreateSettingPage(m_hCamera, NULL, g_CameraName, NULL, NULL, 0);//"通知SDK内部建该相机的属性页面";

	m_hDispThread = (HANDLE)_beginthreadex(NULL, 0, &uiDisplayThread, (PVOID)m_hCamera, 0, &m_threadID);

	CameraPlay(m_hCamera);

	// 是否显示配置界面
	CameraShowSettingPage(m_hCamera, TRUE);//TRUE显示相机配置界面。FALSE则隐藏。

	while (m_bExit != TRUE)
	{
		cv::waitKey(5);
	}

	CameraUnInit(m_hCamera);

	CameraAlignFree(m_pFrameBuffer);

	cv::destroyWindow(g_CameraName);
	return 0;
}


