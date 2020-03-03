#include "recognition.h"
#include "debug.h"
#include <fstream>
#include <iostream> 

/**
* @berif 找中心坐标
*/
inline void GetMostNum(const std::vector<cv::Point> & inputPoints, int &midline)
{
	using ComNum = std::pair<int, int>;
	std::vector<ComNum> get_midline;
	bool if_have = false;
	//统计每一个中间值有多少个
	for (int i = 0; i < inputPoints.size(); i++)
	{
		if_have = false;
		cv::Point temp = inputPoints[i];
		for (int j = 0; j < get_midline.size(); j++)
		{
			if (abs(temp.y-get_midline[j].first)<5)
			{//数已经存在
				get_midline[j].second++;
				if_have = true;
				break;
			}
		}
		if (!if_have)
		{
			get_midline.push_back({ temp.y,1 });
		}
	}
	//找最大值
	int max_num = 0;
	for (int i = 0; i < get_midline.size(); i++)
	{
		if (get_midline[i].second > max_num)
		{
			max_num = get_midline[i].second;
			midline = get_midline[i].first;
		}
	}
}
void act::Recognition::update(cv::Size size, int type, void* data) {

	original_image_ = cv::Mat(size, type, data);
	cv::cvtColor(original_image_, hsv_image_, cv::COLOR_BGR2HSV);
}

inline void GetMostNum(const int* inputPoints, int &midline,int nnn)
{

	
	//找最大值
	int max_num = 0;
	for (int i = 0; i < nnn; i++)
	{
		if (inputPoints[i] > max_num)
		{
			max_num = inputPoints[i];
			midline = i;
		}
	}
}
int drew_line_y=0;



//inline bool fx(cv::Point p1, cv::Point p2)
//{
//	return p1.y > p2.y;
//}

inline int Intersection(int oldfirsty, int oldlasty, int newfirsty, int newlasty)
{
	int status = 0;
	if (oldfirsty >= newfirsty)
	{
		status = 1;  //              -----------
					 //       --------------------
	}
	else {
		status = 2;
	}
	switch (status)
	{
	case 1://newfirst smaller
		if (newlasty > oldfirsty)
		{
			return 1;
		}
		else {
			return 0;
		}
		break;
	case 2://oldfirst smaller
		if (oldlasty > newfirsty)
		{
			return 1;
		}
		else {
			return 0;
		}
		break;
	default:
		break;
	}

}


MatchRes act::Recognition::MatchTemImage(cv::Mat &frame_origin, cv::Mat &frame_template)
{
	cv::Mat g_resultImage;
	MatchRes match;
	cv::Point matchLocation;

	if (frame_origin.type() != 0 || frame_template.type() != 0)
	{
		std::cout << "type error" << std::endl;
		return match;
	}
	int resultImage_cols = frame_origin.cols - frame_template.cols + 1;
	int resultImage_rows = frame_origin.rows - frame_template.rows + 1;

	g_resultImage.create(resultImage_cols, resultImage_rows, CV_32FC1);
	matchTemplate(frame_origin, frame_template, g_resultImage, cv::TM_CCORR);

	double minValue; double maxValue; cv::Point minLocation; cv::Point maxLocation;

	minMaxLoc(g_resultImage, &minValue, &maxValue, &minLocation, &maxLocation, cv::Mat());
	matchLocation = maxLocation;
	match.maxLocation = matchLocation;
	match.maxValue = maxValue;
	return match;
}

cv::Mat act::Recognition::EllipseShape(cv::Size size, cv::RotatedRect &boxtemp, float angle_Rotate)
{
	cv::Mat empty = cv::Mat::zeros(size, 0);
	boxtemp.angle = angle_Rotate;
	boxtemp.center.x = size.width / 2;
	boxtemp.center.y = size.height / 2;

	ellipse(empty, boxtemp, cv::Scalar(255, 0, 0), 1, 8);
	int up, down, left, right;
	int jump_signal = 0;

	for (int i = 0; i < empty.rows; i++)
	{
		uchar *data = empty.ptr<uchar>(i);
		for (int j = 0; j < empty.cols; j++)
		{
			if (data[j])
			{
				up = i;
				jump_signal = 1;
				break;
			}
		}
		if (jump_signal)
			break;
	}

	jump_signal = 0;
	for (int i = empty.rows - 1; i >0; i--)
	{
		uchar *data = empty.ptr<uchar>(i);
		for (int j = 0; j < empty.cols; j++)
		{
			if (data[j])
			{
				down = i + 1;
				jump_signal = 1;
				break;
			}
		}
		if (jump_signal)
			break;
	}

	jump_signal = 0;
	for (int j = 0; j < empty.cols; j++)
	{
		for (int i = empty.rows - 1; i >0; i--)
		{
			uchar *data = empty.ptr<uchar>(i);
			if (data[j])
			{
				left = j;
				jump_signal = 1;
				break;
			}
		}
		if (jump_signal)
			break;
	}

	jump_signal = 0;
	for (int j = empty.cols - 1; j >0; j--)
	{
		for (int i = empty.rows - 1; i >0; i--)
		{
			uchar *data = empty.ptr<uchar>(i);
			if (data[j])
			{
				right = j + 1;
				jump_signal = 1;
				break;
			}
		}
		if (jump_signal)
			break;
	}
	cv::Mat part_cut = empty(cv::Rect(cv::Point(left, up), cv::Point(right, down)));
	return part_cut;
}

cv::Mat act::Recognition::EllipseShapeBorder(cv::Size size, OVAL_parameter &platform, float angle_Rotate)
{
	cv::Mat empty = cv::Mat::zeros(size, 0);
	cv::RotatedRect boxtemp;
	boxtemp.center= cv::Point(size.width / 2, size.height / 2);
	boxtemp.angle = angle_Rotate;
	boxtemp.size.height = platform.height;
	boxtemp.size.width = platform.width;

	
	ellipse(empty, boxtemp, cv::Scalar(255, 0, 0), 1, 8);
	int up, down, left, right;
	int jump_signal = 0;

	for (int i = 0; i < empty.rows; i++)
	{
		uchar *data = empty.ptr<uchar>(i);
		for (int j = 0; j < empty.cols; j++)
		{
			if (data[j])
			{
				up = i;
				jump_signal = 1;
				break;
			}
		}
		if (jump_signal)
			break;
	}

	jump_signal = 0;
	for (int i = empty.rows - 1; i >0; i--)
	{
		uchar *data = empty.ptr<uchar>(i);
		for (int j = 0; j < empty.cols; j++)
		{
			if (data[j])
			{
				down = i + 1;
				jump_signal = 1;
				break;
			}
		}
		if (jump_signal)
			break;
	}

	jump_signal = 0;
	for (int j = 0; j < empty.cols; j++)
	{
		for (int i = empty.rows - 1; i >0; i--)
		{
			uchar *data = empty.ptr<uchar>(i);
			if (data[j])
			{
				left = j;
				jump_signal = 1;
				break;
			}
		}
		if (jump_signal)
			break;
	}

	jump_signal = 0;
	for (int j = empty.cols - 1; j >0; j--)
	{
		for (int i = empty.rows - 1; i >0; i--)
		{
			uchar *data = empty.ptr<uchar>(i);
			if (data[j])
			{
				right = j + 1;
				jump_signal = 1;
				break;
			}
		}
		if (jump_signal)
			break;
	}
	cv::Mat part_cut = empty(cv::Rect(cv::Point(left, up), cv::Point(right, down)));
	return part_cut;
}


cv::Mat act::Recognition::EllipseJudg(cv::Mat &frame_transcript, cv::RotatedRect &boxtemp)
{
	int x0 = 0, y0 = 0;
	int kuandu = (int)boxtemp.size.height;
	int gaodu = (int)boxtemp.size.width;
	x0 = boxtemp.center.x - (OVAL_HEIGHT / 2);
	y0 = boxtemp.center.y - (OVAL_WIDTH / 2);
	x0 = x0 - EXPEND_SIZE;
	y0 = y0 - EXPEND_SIZE;
	kuandu = kuandu + EXPEND_SIZE*2;
	gaodu = gaodu + EXPEND_SIZE * 2;
	cv::Mat ellipse_inside = cv::Mat::zeros(frame_transcript.rows, frame_transcript.cols, 0);


	//因为椭圆翻转角度为90度  所以宽度 高度 互换
	int width = boxtemp.size.height / 2;
	int height = boxtemp.size.width / 2;
	int center_x = boxtemp.center.x;
	int center_y = boxtemp.center.y;
	//椭圆截取
	//for (int i = 0; i < frame_transcript.rows; i++)
	//{
	//	uchar *datain = ellipse_inside.ptr<uchar>(i);
	//	for (int j = 0; j < frame_transcript.cols; j++)
	//	{
	//		double temp = pow((double)(j - center_x) / width, 2) + pow((double)(i - center_y) / height, 2);
	//		if (temp < 1.7)
	//			datain[j] = 1;
	//	}
	//}
	//for (int i = 0; i < frame_transcript.rows; i++)
	//{
	//	uchar *dataframe = frame_transcript.ptr<uchar>(i);
	//	uchar *dataellipse = ellipse_inside.ptr<uchar>(i);
	//	for (int j = 0; j < frame_transcript.cols; j++)
	//	{
	//		int bgr3_1 = 3 * j;
	//		if (!dataellipse[j])
	//		{
	//			dataframe[bgr3_1] = 0;
	//			dataframe[bgr3_1 + 1] = 0;
	//			dataframe[bgr3_1 + 2] = 0;
	//		}
	//	}
	//}


	if (x0 < 0)
	{
		x0 = 0;
		std::cout << "widthover<0 " << std::endl;
	}
	else if ((x0 + kuandu)>frame_transcript.cols)
	{
		kuandu = frame_transcript.cols - x0;
		std::cout << "widthover>max " << std::endl;
	}
	if (y0 < 0)
	{
		y0 = 0;
		std::cout << "heightover<0  " << std::endl;
	}
	else if ((y0 + gaodu) > frame_transcript.rows)
	{
		gaodu = frame_transcript.rows - y0;
		std::cout << "heightover>max  " << std::endl;
	}


	cv::Mat frame_cut_div;
	frame_cut_div = frame_transcript(cv::Rect(x0, y0, kuandu, gaodu));
	boxtemp.center.x = boxtemp.center.x - x0;
	boxtemp.center.y = boxtemp.center.y - y0;

	//cv::imshow("frame_cut_div", frame_cut_div);
	return frame_cut_div;
}


double maxVal_ang;
//匹配整体判断
int act::Recognition::MatchPart(cv::Mat &frame_gray, cv::Mat &frame)
{
	boxt.size.height = OVAL_HEIGHT;
	boxt.size.width = OVAL_WIDTH;

	MatchRes res_match_total;
	res_match_total.maxValue = 0;
	float angle_most;
	cv::Size cut_size;

	double ang_init = maxVal_ang - 2;

	MatchRes match_part;
	float angle_hho = maxVal_ang;

	cv::Mat ell = EllipseShape(frame_gray.size(), boxt, angle_hho);
	match_part = MatchTemImage(frame_gray, ell);

	if (match_part.maxValue>res_match_total.maxValue)
	{
		res_match_total.maxValue = match_part.maxValue;
		res_match_total.maxLocation = match_part.maxLocation;
		angle_most = angle_hho;
		cut_size = ell.size();
	}
	
	cv::Point image_total_center;
	if (res_match_total.maxValue > 100000)
	{
		image_total_center = cv::Point(res_match_total.maxLocation.x + cut_size.width / 2, res_match_total.maxLocation.y + cut_size.height / 2);
		
#ifdef MATCH_RESULT
		std::cout << res_match_total.maxValue << std::endl;
		std::cout << angle_most << std::endl;
#endif

		boxt.angle = angle_most;
		boxt_orgin_center = image_total_center;
		boxt.center.x = image_total_center.x;
		boxt.center.y = image_total_center.y;


		ellipse(frame_gray, boxt, cv::Scalar(200, 0, 0), 1, 8);
		cv::imshow("frfrfe", frame_gray);

		ellipse(frame, boxt, cv::Scalar(0, 220, 0), 1, 8);
		roi_image_ = EllipseJudg(frame, boxt);
		cv::cvtColor(roi_image_, roi_hsv_image_, cv::COLOR_BGR2HSV);

		return 0;
	}
	else {
		return 1;
	}

}

cv::Point act::Recognition::MatchPartRand(cv::Mat &frame_gray, cv::Mat &frame, OVAL_parameter &platform)
{
	MatchRes match_part;
	cv::Mat ell = EllipseShapeBorder(frame.size(), platform, platform.angle);
	match_part = MatchTemImage(frame_gray, ell);

	cv::Point image_total_center;
	image_total_center = cv::Point(match_part.maxLocation.x + ell.size().width / 2, match_part.maxLocation.y + ell.size().height / 2);
	cv::RotatedRect land_ell;

	land_ell.angle = platform.angle;
	land_ell.center.x = image_total_center.x;
	land_ell.center.y = image_total_center.y;
	land_ell.size.height = platform.height;
	land_ell.size.width = platform.width;

	ellipse(frame_gray, land_ell, cv::Scalar(200, 0, 0), 1, 8);
	///*imshow("gray_platform", frame_gray);*/

	//ellipse(frame, land_ell, cv::Scalar(0, 220, 0), 1, 8);

	////imshow("color_plat", frame);
	return image_total_center;
}
MatchValAng act::Recognition::MatchPartNearInit(cv::Mat &frame_gray, cv::Mat &frame, OVAL_parameter &platform)
{
	MatchValAng val_ang;
	MatchRes match_part;
	cv::Point image_total_center;

	//匹配模板的大小
	cv::Size size_model_size;
	size_model_size.height = 0;
	size_model_size.width = 0;
	//匹配结果
	MatchRes res_match_total;
	res_match_total.maxValue = 0;
	res_match_total.maxLocation = cv::Point(0, 0);

	//匹配最值，及对应角度
	val_ang.angle = 0;
	val_ang.maxValue = 0;


	for (int i = 0; i < CIRCLE_i; i++)
	{
		float angle_hho = 90 +0.1*i;
		cv::Mat ell = EllipseShapeBorder(frame.size(), platform, angle_hho);
		//cv::imshow("ell", ell);
		match_part = MatchTemImage(frame_gray, ell);

		//image_total_center = cv::Point(match_part.maxLocation.x + ell.size().width / 2, match_part.maxLocation.y + ell.size().height / 2);
		//cv::RotatedRect land_ell;
		//land_ell.angle = angle_hho;
		//land_ell.center.x = image_total_center.x;
		//land_ell.center.y = image_total_center.y;
		//land_ell.size.height = platform.height;
		//land_ell.size.width = platform.width;

		//cv::Mat frame_graycop = frame_gray.clone();
		//cv::Mat framecop = frame.clone();

		//ellipse(frame_graycop, land_ell, cv::Scalar(200, 0, 0), 1, 8);
		//imshow("frame_graycop", frame_graycop);
		//ellipse(framecop, land_ell, cv::Scalar(0, 220, 0), 1, 8);
		//imshow("framecop", framecop);

		if (match_part.maxValue>res_match_total.maxValue)
		{
			res_match_total.maxValue = match_part.maxValue;
			res_match_total.maxLocation = match_part.maxLocation;

			val_ang.angle = angle_hho;
			val_ang.maxValue = match_part.maxValue;
			size_model_size = ell.size();
		}
	}

	image_total_center = cv::Point(res_match_total.maxLocation.x + size_model_size.width / 2, res_match_total.maxLocation.y + size_model_size.height / 2);

	//绘制椭圆，进行检验
	cv::RotatedRect land_ell;
	land_ell.angle = val_ang.angle;
	land_ell.center.x = image_total_center.x;
	land_ell.center.y = image_total_center.y;
	land_ell.size.height = platform.height;
	land_ell.size.width = platform.width;

	ellipse(frame_gray, land_ell, cv::Scalar(200, 0, 0), 1, 8);
	cv::imshow("gray_platform", frame_gray);

	ellipse(frame, land_ell, cv::Scalar(0, 220, 0), 1, 8);

	cv::imshow("color_plat", frame);
	return val_ang;
}

MatchValAngCenter act::Recognition::MatchPartRandInit(cv::Mat &frame_gray, cv::Mat &frame, OVAL_parameter &platform)
{
	MatchValAngCenter val_ang_center;
	//MatchValAng val_ang;
	MatchRes match_part;
	cv::Point image_total_center;

	//匹配模板的大小
	cv::Size size_model_size;
	size_model_size.height = 0;
	size_model_size.width = 0;
	//匹配结果
	MatchRes res_match_total;
	res_match_total.maxValue = 0;
	res_match_total.maxLocation = cv::Point(0, 0);

	//匹配最值，及对应角度
	val_ang_center.angle = 0;
	val_ang_center.maxValue = 0;
	val_ang_center.center = cv::Point(0, 0);
	double angle_st = platform.angle - 3;
	for (int i = 0; i < 60; i++)
	{
		float angle_hho = angle_st + 0.1*i;
		cv::Mat ell = EllipseShapeBorder(frame.size(), platform, angle_hho);
		match_part = MatchTemImage(frame_gray, ell);

		if (match_part.maxValue>res_match_total.maxValue)
		{
			res_match_total.maxValue = match_part.maxValue;
			res_match_total.maxLocation = match_part.maxLocation;

			val_ang_center.angle = angle_hho;
			size_model_size = ell.size();
		}
	}

	image_total_center = cv::Point(res_match_total.maxLocation.x + size_model_size.width / 2, res_match_total.maxLocation.y + size_model_size.height / 2);
	val_ang_center.center = image_total_center;
	val_ang_center.maxValue = res_match_total.maxValue;
	//绘制椭圆，进行检验
	cv::RotatedRect land_ell;
	land_ell.angle = val_ang_center.angle;
	land_ell.center.x = image_total_center.x;
	land_ell.center.y = image_total_center.y;
	land_ell.size.height = platform.height;
	land_ell.size.width = platform.width;

	ellipse(frame_gray, land_ell, cv::Scalar(200, 0, 0), 1, 8);
	//imshow("gray_platform", frame_gray);

	ellipse(frame, land_ell, cv::Scalar(0, 220, 0), 1, 8);

	//imshow("color_plat", frame);
	return val_ang_center;
}


MatchAngCenter act::Recognition::MatchPartRandArrive(cv::Mat &frame_gray, cv::Mat &frame, OVAL_parameter &platform)
{
	MatchValAng val_ang;
	MatchRes match_part;
	MatchAngCenter ang_cen;
	cv::Point image_total_center;

	//匹配模板的大小
	cv::Size size_model_size;
	size_model_size.height = 0;
	size_model_size.width = 0;
	//匹配结果
	MatchRes res_match_total;
	res_match_total.maxValue = 0;
	res_match_total.maxLocation = cv::Point(0, 0);

	//匹配最值，及对应角度
	val_ang.angle = 0;
	val_ang.maxValue = 0;
	double angle_st = platform.angle - 1;
	for (int i = 0; i < CIRCLE_i; i++)
	{
		float angle_hho = angle_st + 0.1*i;
		cv::Mat ell = EllipseShapeBorder(frame.size(), platform, angle_hho);
		//cv::imshow("ell", ell);
		match_part = MatchTemImage(frame_gray, ell);

		if (match_part.maxValue>res_match_total.maxValue)
		{
			res_match_total.maxValue = match_part.maxValue;
			res_match_total.maxLocation = match_part.maxLocation;

			val_ang.angle = angle_hho;
			val_ang.maxValue = match_part.maxValue;
			size_model_size = ell.size();
		}
	}

	image_total_center = cv::Point(res_match_total.maxLocation.x + size_model_size.width / 2, res_match_total.maxLocation.y + size_model_size.height / 2);
	//绘制椭圆，进行检验
	cv::RotatedRect land_ell;
	land_ell.angle = val_ang.angle;
	land_ell.center.x = image_total_center.x;
	land_ell.center.y = image_total_center.y;
	land_ell.size.height = platform.height;
	land_ell.size.width = platform.width;

	ellipse(frame_gray, land_ell, cv::Scalar(200, 0, 0), 1, 8);
	//imshow("gray_platform", frame_gray);

	ellipse(frame, land_ell, cv::Scalar(0, 220, 0), 1, 8);
	ang_cen.angle = val_ang.angle;
	ang_cen.center = image_total_center;
	//imshow("color_plat", frame);
	return ang_cen;
}
cv::Point act::Recognition::MatchPartNearPlat(cv::Mat &frame_gray, cv::Mat &frame, float angle, int height, int width)
{
	cv::RotatedRect platform;
	platform.size.height = height;
	platform.size.width = width;

	MatchRes match_part;
	cv::Mat ell = EllipseShape(frame_gray.size(), platform, angle);
	match_part = MatchTemImage(frame_gray, ell);

	cv::Point image_total_center;
	image_total_center = cv::Point(match_part.maxLocation.x + ell.size().width / 2, match_part.maxLocation.y + ell.size().height / 2);

	platform.angle = angle;
	platform.center.x = image_total_center.x;
	platform.center.y = image_total_center.y;

	ellipse(frame_gray, platform, cv::Scalar(200, 0, 0), 1, 8);
	imshow("gray_platform", frame_gray);



	//imshow("color_plat", frame);
	roi_image_ = frame.clone();
	//EllipseJudg(frame, platform);
	ellipse(frame, platform, cv::Scalar(0, 220, 0), 1, 8);
	cv::cvtColor(roi_image_, roi_hsv_image_, cv::COLOR_BGR2HSV);
	return image_total_center;
}


double act::Recognition::MatchPart_Move(cv::Mat &frame_gray, cv::Mat &frame)
{
	boxt.size.height = OVAL_HEIGHT;
	boxt.size.width = OVAL_WIDTH;
	//int match_many = 0;
	//int left_right_signal = 0;
	//cv::Point left_border, right_border;

	MatchRes res_match_total;
	res_match_total.maxValue = 0;
	float angle_most;
	cv::Size cut_size;


	double ang_init = maxVal_ang - 1.5;
	for (int i = 0; i <10; i++)
	{
		MatchRes match_part;
		float angle_hho = ang_init + 0.3*i;

		cv::Mat ell = EllipseShape(frame_gray.size(), boxt, angle_hho);
		match_part = MatchTemImage(frame_gray, ell);

		if (match_part.maxValue>res_match_total.maxValue)
		{
			res_match_total.maxValue = match_part.maxValue;
			res_match_total.maxLocation = match_part.maxLocation;
			angle_most = angle_hho;
			cut_size = ell.size();
		}
	}

	cv::Point image_total_center;
	MatchValAng val_ang = { 0,0 };
	if (res_match_total.maxValue > 100)
	{
		//rectangle(frame_gray, res_match_total.maxLocation, Point(res_match_leftmost.maxLocation.x + image_leftmost.cols, res_match_leftmost.maxLocation.y + image_leftmost.rows), Scalar(120, 0, 0), 2, 8, 0);
		image_total_center = cv::Point(res_match_total.maxLocation.x + cut_size.width / 2, res_match_total.maxLocation.y + cut_size.height / 2);

		std::cout << "ang " << angle_most;

		boxt.angle = angle_most;
		boxt_orgin_center = image_total_center;
		boxt.center.x = image_total_center.x;
		boxt.center.y = image_total_center.y;

		val_ang.angle = angle_most;
		val_ang.maxValue = res_match_total.maxValue;

		ellipse(frame_gray, boxt, cv::Scalar(200, 0, 0), 1, 8);
		cv::imshow("frfrfe", frame_gray);

		ellipse(frame, boxt, cv::Scalar(0, 220, 0), 1, 8);
		cv::imshow("really", frame);

		roi_image_ = EllipseJudg(frame, boxt);
		cv::cvtColor(roi_image_, roi_hsv_image_, cv::COLOR_BGR2HSV);


		return angle_most;
	}
	else {
		return 1;
	}

}
//颜色判断
unsigned char act::Recognition::ColorDefine(void)
{
	//hsv_image_   boxt_orgin_center
	int co = boxt_orgin_center.x;
	int ro = boxt_orgin_center.y;
	int i_st = 0, i_end = 0;
	int j_st = 0, j_end = 0;
	int color_b = 0, color_r = 0;
	if (ro - 5 >= 0)
	{
		i_st = ro - 5;
	}
	else
	{
		i_st = 0;
	}
	if (ro + 5 < hsv_image_.rows-1)
	{
		i_end = ro + 5;
	}
	else
	{
		i_end = hsv_image_.rows - 1;
	}


	if (co - 5 > 0)
	{
		j_st = co - 5;
	}
	else
	{
		j_st = 0;
	}
	if (co + 5 < hsv_image_.cols - 1)
	{
		j_end = co + 5;
	}
	else
	{
		j_end = hsv_image_.cols - 1;
	}
	for (int i = i_st; i < i_end; i++)
	{
		unsigned char *data = hsv_image_.ptr<unsigned char>(i);
		for (int j = j_st; j < j_end; j++)
		{
			int act_j = j * 3;
			if (data[act_j + 1] > 200)
			{
				if (data[act_j] < 10 || data[act_j]>150)
				{
					color_r++;
				}
				else
				{
					color_b++;
				}
			}
		}
	}
	std::cout << "r  " << color_r << "  b " << color_b << std::endl;
	if (color_r > color_b)
		return 'b';
	else
		return 'r';
}
//初始化判断
MatchValAng act::Recognition::MatchInit(cv::Mat &frame_gray, cv::Mat &frame, OVAL_parameter &platform)
{
	int match_many = 0;
	int left_right_signal = 0;
	cv::Point left_border, right_border;

	MatchRes res_match_total;
	res_match_total.maxValue = 0;
	float angle_most;
	cv::Size cut_size;

	MatchValAng val_ang = { 0,0 };

	cv::Mat bod = EllipseShapeBorder(frame_gray.size(), platform, 100);
	cv::imshow("boder", bod);

	for (int i = 0; i <CIRCLE_i; i++)
	{
		MatchRes match_part;
		float angle_hho = OVAL_ANGLE + 0.1*i;

		cv::Mat ell = EllipseShapeBorder(frame_gray.size(), platform, angle_hho);
		match_part = MatchTemImage(frame_gray, ell);

		if (match_part.maxValue>res_match_total.maxValue)
		{
			res_match_total.maxValue = match_part.maxValue;
			res_match_total.maxLocation = match_part.maxLocation;
			angle_most = angle_hho;
			cut_size = ell.size();

			val_ang.angle = angle_hho;
			val_ang.maxValue = match_part.maxValue;
		}
	}
	return val_ang;

}

cv::Point eight_nei[8];
std::vector<cv::Point> act::Recognition::find_contours(cv::Mat &inputMeidanImage, cv::Mat &FrisbeeBlack)
{
	cv::Mat border_many = cv::Mat::zeros(inputMeidanImage.rows, inputMeidanImage.cols, inputMeidanImage.type());

	for (int i = 0; i < inputMeidanImage.rows; i++)
	{
		uchar *data = inputMeidanImage.ptr<uchar>(i);
		uchar *data_bor = border_many.ptr<uchar>(i);

		for (int j = 0; j < inputMeidanImage.cols-1; j++)
		{
			if (data[j] != data[j + 1])
			{
				if (!data[j])
					data_bor[j] = 255;
				else
					data_bor[j + 1] = 255;
			}
		}
		if (data[inputMeidanImage.cols - 1])
		{
			data_bor[inputMeidanImage.cols - 1] = 255;
		}
		else if (data[0])
		{
			data_bor[0] = 255;
		}
	}

	for (int j = 0; j < inputMeidanImage.cols; j++)
	{
		for (int i = 0; i < inputMeidanImage.rows-1; i++)
		{
			uchar *data = inputMeidanImage.ptr<uchar>(i);
			uchar *data_next = inputMeidanImage.ptr<uchar>(i+1);
			uchar *data_bor = border_many.ptr<uchar>(i);
			uchar *data_bor_next = border_many.ptr<uchar>(i + 1);		
			if (data[j] != data_next[j])
			{
				if (!data[j])
					data_bor[j] = 255;
				else
					data_bor_next[j] = 255;
			}
		}
		uchar *data_ori = inputMeidanImage.ptr<uchar>(0);
		uchar *data_dat = border_many.ptr<uchar>(0);
		if (data_ori[j])
		{
			data_dat[j] = 255;
		}
		data_ori = inputMeidanImage.ptr<uchar>(inputMeidanImage.rows - 1);
		data_dat = border_many.ptr<uchar>(inputMeidanImage.rows - 1);
		if(data_ori[j])
		{
			data_dat[j] = 255;
		}
	}
	//cv::imshow("border_many", border_many);

	//cv::Mat cut_tuo_exx= cv::Mat::zeros(inputMeidanImage.rows, inputMeidanImage.cols, inputMeidanImage.type());
	//for (int i = 0; i < inputMeidanImage.rows; i++)
	//{
	//	uchar * data = cut_tuo_exx.ptr<uchar>(i);
	//	for (int j = 0; j < inputMeidanImage.cols; j++)
	//	{
	//		if (IF_INNER(abs(j - boxt.center.x), abs(i - boxt.center.y), OVAL_HEIGHT_HALT, OVAL_WIDTH_HALF))
	//		{
	//			data[j] = 255;
	//		}
 //
	//	}
	//}
	//cv::imshow("cutpantuo", cut_tuo_exx);




	cv::Mat canny_co = border_many.clone();
	std::vector<std::vector<cv::Point>> point_total;

	std::vector<cv::Point> point;
	int point_many = 0;
	int round_in_sign = 0;
	cv::Mat zero_text = cv::Mat::zeros(inputMeidanImage.rows, inputMeidanImage.cols, inputMeidanImage.type());

	int jump = 0;
	for (int i = 0; i < inputMeidanImage.rows; i++)
	{
		uchar *data = canny_co.ptr<uchar>(i);
		for (int j = 0; j < inputMeidanImage.cols; j++)
		{
			if (data[j])
			{
				data[j] = 0;
				point.push_back(cv::Point(j, i));
				round_in_sign = 0;
				//记住有一个bug，如果起始点是(0,0)可能会出现不对的现象
				cv::Point last = cv::Point(j - 1, i);
				cv::Point center = cv::Point(j, i);
				cv::Point *text_eight;
				
				int signal = 1;
				while (signal)
				{
					signal = 0;
					text_eight = EightRegion(center, last, inputMeidanImage.rows, inputMeidanImage.cols);
					for (int i_i = 0; i_i < 7; i_i++)
					{
						int temp_y = text_eight[(i_i + 1)].y;
						int temp_x = text_eight[(i_i + 1)].x;
						uchar *in_data = canny_co.ptr<uchar>(temp_y);
						if (in_data[temp_x])
						{
							if (!round_in_sign)
							{
								if (IF_INNER(abs(temp_x - boxt.center.x),abs(temp_y - boxt.center.y), OVAL_HEIGHT_HALT, OVAL_WIDTH_HALF))
								{
									round_in_sign = 1;
								}
							}
							 
							point.push_back(text_eight[(i_i + 1)]);
							in_data[temp_x] = 0;

							last = text_eight[i_i];
							center = text_eight[(i_i + 1)];
							signal = 1;
							break;
						}
					}

				}
				if (point.size() > 30)
				{
					if(round_in_sign)
						point_total.push_back(point);
					else
					{
						for (int k = 0; k < point.size(); k++)
						{
							int row_num = point[k].y;
							if (row_num > inputMeidanImage.rows - 10)
								continue;
							else
							{
								row_num = row_num + 3;
							}

							int col_num = point[k].x;
							uchar *data_down_bla = FrisbeeBlack.ptr<uchar>(row_num);
							while (data_down_bla[col_num])
							{
								if (row_num > (inputMeidanImage.rows - 10))
									break;
								if (IF_INNER(abs(col_num - boxt.center.x), abs(row_num - boxt.center.y), OVAL_HEIGHT_HALT, OVAL_WIDTH_HALF))
								{								
									round_in_sign = 1;
									break;
								}
								row_num++;
								data_down_bla = FrisbeeBlack.ptr<uchar>(row_num);
							}
							if (round_in_sign)
							{
								point_total.push_back(point);
								break;
							}
								
							
						}
					}
				}
					
				point.clear();
			}
		}

	}

	std::vector<cv::Point> point_rect;
	//std::cout << point_total.size() << std::endl;
	if (point_total.size())
	{
		for (int i = 0; i < point_total.size(); i++)
		{
			int up, down, left, right;
			up = point_total[i][0].y;
			down = point_total[i][0].y;
			left = point_total[i][0].x;
			right = point_total[i][0].x;
			for (int j = 1; j < point_total[i].size(); j++)
			{
				if (point_total[i][j].x > right)
					right = point_total[i][j].x;
				if (point_total[i][j].x < left)
					left = point_total[i][j].x;
				if (point_total[i][j].y > down)
					down = point_total[i][j].y;
				if(point_total[i][j].y <up)
					up = point_total[i][j].y;

				uchar *data = zero_text.ptr<uchar>(point_total[i][j].y);
				data[point_total[i][j].x] = 255;
			}
			//point_rect.push_back(cv::Point(left, up));
			//point_rect.push_back(cv::Point(right, up));
			//point_rect.push_back(cv::Point(right, down));
			//point_rect.push_back(cv::Point(left, down));
			if (point_rect.size())
			{
				int signal_contain = 0;
				for (int k = 0; k < point_rect.size() / 4; k++)
				{
					//cv::Point regin_rang_st = point_rect[4 * k];
					//cv::Point regin_rang_end = point_rect[4 * k + 2];
					if( JudgePointRect(point_rect[4 * k], point_rect[4 * k + 2], cv::Point(left, up))==TRUE)
					{
						if (JudgePointRect(point_rect[4 * k], point_rect[4 * k + 2], cv::Point(right, down)) == TRUE)
						{
							signal_contain = 1;
							break;
						}
						
					}
				}
				if(!signal_contain)
				{
					point_rect.push_back(cv::Point(left, up));
					point_rect.push_back(cv::Point(right, up));
					point_rect.push_back(cv::Point(right, down));
					point_rect.push_back(cv::Point(left, down));
				}
			}
			else
			{
				point_rect.push_back(cv::Point(left, up));
				point_rect.push_back(cv::Point(right, up));
				point_rect.push_back(cv::Point(right, down));
				point_rect.push_back(cv::Point(left, down));
			}


		}
	}
	cv::imshow("zero_text", zero_text);
	return point_rect;
}
//寻找边界函数
std::vector<cv::Point> act::Recognition::find_contours_te(cv::Mat &inputMeidanImage, cv::Mat &FrisbeeBlack, OVAL_parameter &platform)
{
	cv::Mat border_many = cv::Mat::zeros(inputMeidanImage.rows, inputMeidanImage.cols, inputMeidanImage.type());

	//如果一副图像左边右边像素点值不一样  上边下边像素点值不一样 认为这个点是边界
	//注意第一行 最后一行 第一列 最后一列
	//判断左右
	for (int i = 0; i < inputMeidanImage.rows; i++)
	{
		uchar *data = inputMeidanImage.ptr<uchar>(i);
		uchar *data_bor = border_many.ptr<uchar>(i);

		for (int j = 0; j < inputMeidanImage.cols - 1; j++)
		{
			if (data[j] != data[j + 1])
			{
				if (!data[j])
					data_bor[j] = 255;
				else
					data_bor[j + 1] = 255;
			}
		}
		if (data[inputMeidanImage.cols - 1])
		{
			data_bor[inputMeidanImage.cols - 1] = 255;
		}
		else if (data[0])
		{
			data_bor[0] = 255;
		}
	}

	//判断上下
	for (int j = 0; j < inputMeidanImage.cols; j++)
	{
		for (int i = 0; i < inputMeidanImage.rows - 1; i++)
		{
			uchar *data = inputMeidanImage.ptr<uchar>(i);
			uchar *data_next = inputMeidanImage.ptr<uchar>(i + 1);
			uchar *data_bor = border_many.ptr<uchar>(i);
			uchar *data_bor_next = border_many.ptr<uchar>(i + 1);
			if (data[j] != data_next[j])
			{
				if (!data[j])
					data_bor[j] = 255;
				else
					data_bor_next[j] = 255;
			}
		}
		uchar *data_ori = inputMeidanImage.ptr<uchar>(0);
		uchar *data_dat = border_many.ptr<uchar>(0);
		if (data_ori[j])
		{
			data_dat[j] = 255;
		}
		data_ori = inputMeidanImage.ptr<uchar>(inputMeidanImage.rows - 1);
		data_dat = border_many.ptr<uchar>(inputMeidanImage.rows - 1);
		if (data_ori[j])
		{
			data_dat[j] = 255;
		}
	}
	//cv::imshow("border_many", border_many);


	cv::Mat canny_co = border_many.clone();
	std::vector<std::vector<cv::Point>> point_total;

	std::vector<cv::Point> point;
	int point_many = 0;
	int round_in_sign = 0;
	cv::Mat zero_text = cv::Mat::zeros(inputMeidanImage.rows, inputMeidanImage.cols, inputMeidanImage.type());

	cv::Mat canny_text = border_many.clone();
	int jump = 0;
	for (int i = 0; i < inputMeidanImage.rows; i++)
	{
		uchar *data = canny_co.ptr<uchar>(i);
		for (int j = 0; j < inputMeidanImage.cols; j++)
		{
			if (data[j])
			{
				data[j] = 0;
				point.push_back(cv::Point(j, i));
				round_in_sign = 0;
				//记住有一个bug，如果起始点是(0,0)可能会出现不对的现象
				cv::Point last = cv::Point(j - 1, i);
				cv::Point center = cv::Point(j, i);
				cv::Point *text_eight;

				int signal = 1;
				while (signal)
				{
					signal = 0;
					text_eight = EightRegion(center, last, inputMeidanImage.rows, inputMeidanImage.cols);
					for (int i_i = 0; i_i < 7; i_i++)
					{
						int temp_y = text_eight[(i_i + 1)].y;
						int temp_x = text_eight[(i_i + 1)].x;
						uchar *in_data = canny_co.ptr<uchar>(temp_y);
						if (in_data[temp_x])
						{
							if (!round_in_sign)
							{
								if (IF_INNER(abs(temp_x - platform.center.x), abs(temp_y - platform.center.y), (platform.height/2), (platform.width/2)))
								{
									
									cv::circle(canny_text, cv::Point(temp_x, temp_y), 3, cv::Scalar(220, 220, 220));
									round_in_sign = 1;
								}
							}

							point.push_back(text_eight[(i_i + 1)]);
							in_data[temp_x] = 0;

							last = text_eight[i_i];
							center = text_eight[(i_i + 1)];
							signal = 1;
							break;
						}
					}
				}
				if (point.size() > 30)
				{
					if (round_in_sign)
						point_total.push_back(point);			
				}
				point.clear();
			}
		}
	}



	cv::imshow("canny_text", canny_text);
	std::vector<cv::Point> point_rect;
	if (point_total.size())
	{
		for (int i = 0; i < point_total.size(); i++)
		{
			int up, down, left, right;
			up = point_total[i][0].y;
			down = point_total[i][0].y;
			left = point_total[i][0].x;
			right = point_total[i][0].x;
			for (int j = 1; j < point_total[i].size(); j++)
			{
				if (point_total[i][j].x > right)
					right = point_total[i][j].x;
				if (point_total[i][j].x < left)
					left = point_total[i][j].x;
				if (point_total[i][j].y > down)
					down = point_total[i][j].y;
				if (point_total[i][j].y <up)
					up = point_total[i][j].y;

				uchar *data = zero_text.ptr<uchar>(point_total[i][j].y);
				data[point_total[i][j].x] = 255;
			}
			//point_rect.push_back(cv::Point(left, up));
			//point_rect.push_back(cv::Point(right, up));
			//point_rect.push_back(cv::Point(right, down));
			//point_rect.push_back(cv::Point(left, down));
			if (point_rect.size())
			{
				int signal_contain = 0;
				for (int k = 0; k < point_rect.size() / 4; k++)
				{
					//cv::Point regin_rang_st = point_rect[4 * k];
					//cv::Point regin_rang_end = point_rect[4 * k + 2];
					if (JudgePointRect(point_rect[4 * k], point_rect[4 * k + 2], cv::Point(left, up)) == TRUE)
					{
						if (JudgePointRect(point_rect[4 * k], point_rect[4 * k + 2], cv::Point(right, down)) == TRUE)
						{
							signal_contain = 1;
							break;
						}

					}
				}
				if (!signal_contain)
				{
					point_rect.push_back(cv::Point(left, up));
					point_rect.push_back(cv::Point(right, up));
					point_rect.push_back(cv::Point(right, down));
					point_rect.push_back(cv::Point(left, down));
				}
			}
			else
			{
				point_rect.push_back(cv::Point(left, up));
				point_rect.push_back(cv::Point(right, up));
				point_rect.push_back(cv::Point(right, down));
				point_rect.push_back(cv::Point(left, down));
			}


		}
	}
	return point_rect;
}


std::vector<cv::Point> act::Recognition::find_contours_model(cv::Mat &inputMeidanImage, cv::Mat &FrisbeeBlack, OVAL_parameter &platform)
{
	cv::Mat border_many = cv::Mat::zeros(inputMeidanImage.rows, inputMeidanImage.cols, inputMeidanImage.type());

	for (int i = 0; i < inputMeidanImage.rows; i++)
	{
		uchar *data = inputMeidanImage.ptr<uchar>(i);
		uchar *data_bor = border_many.ptr<uchar>(i);

		for (int j = 0; j < inputMeidanImage.cols - 1; j++)
		{
			if (data[j] != data[j + 1])
			{
				if (!data[j])
					data_bor[j] = 255;
				else
					data_bor[j + 1] = 255;
			}
		}
		if (data[inputMeidanImage.cols - 1])
		{
			data_bor[inputMeidanImage.cols - 1] = 255;
		}
		else if (data[0])
		{
			data_bor[0] = 255;
		}
	}

	for (int j = 0; j < inputMeidanImage.cols; j++)
	{
		for (int i = 0; i < inputMeidanImage.rows - 1; i++)
		{
			uchar *data = inputMeidanImage.ptr<uchar>(i);
			uchar *data_next = inputMeidanImage.ptr<uchar>(i + 1);
			uchar *data_bor = border_many.ptr<uchar>(i);
			uchar *data_bor_next = border_many.ptr<uchar>(i + 1);
			if (data[j] != data_next[j])
			{
				if (!data[j])
					data_bor[j] = 255;
				else
					data_bor_next[j] = 255;
			}
		}
		uchar *data_ori = inputMeidanImage.ptr<uchar>(0);
		uchar *data_dat = border_many.ptr<uchar>(0);
		if (data_ori[j])
		{
			data_dat[j] = 255;
		}
		data_ori = inputMeidanImage.ptr<uchar>(inputMeidanImage.rows - 1);
		data_dat = border_many.ptr<uchar>(inputMeidanImage.rows - 1);
		if (data_ori[j])
		{
			data_dat[j] = 255;
		}
	}
	cv::imshow("border_many", border_many);


	cv::Mat canny_co = border_many.clone();
	std::vector<std::vector<cv::Point>> point_total;

	std::vector<cv::Point> point;
	int point_many = 0;
	int round_in_sign = 0;
	cv::Mat zero_text = cv::Mat::zeros(inputMeidanImage.rows, inputMeidanImage.cols, inputMeidanImage.type());


	std::vector<int> encircle_size;


	cv::Mat canny_text = border_many.clone();

	int jump = 0;
	for (int i = 0; i < inputMeidanImage.rows; i++)
	{
		uchar *data = canny_co.ptr<uchar>(i);
		for (int j = 0; j < inputMeidanImage.cols; j++)
		{
			if (data[j])
			{
				data[j] = 0;
				point.push_back(cv::Point(j, i));
				round_in_sign = 0;
				//记住有一个bug，如果起始点是(0,0)可能会出现不对的现象
				cv::Point last = cv::Point(j - 1, i);
				cv::Point center = cv::Point(j, i);
				cv::Point *text_eight;

				int left = j;
				int right = j;
				int up = i;
				int down = i;

				int signal = 1;
				while (signal)
				{
					signal = 0;
					text_eight = EightRegion(center, last, inputMeidanImage.rows, inputMeidanImage.cols);
					for (int i_i = 0; i_i < 7; i_i++)
					{
						int temp_y = text_eight[(i_i + 1)].y;
						int temp_x = text_eight[(i_i + 1)].x;
						uchar *in_data = canny_co.ptr<uchar>(temp_y);
						if (in_data[temp_x])
						{
							point.push_back(text_eight[(i_i + 1)]);

							left = left < temp_x ? left : temp_x;
							right = right > temp_x ? right : temp_x;
							up = up < temp_y ? up : temp_y;
							down = down > temp_y ? down : temp_y;

							in_data[temp_x] = 0;
							last = text_eight[i_i];
							center = text_eight[(i_i + 1)];
							signal = 1;
							break;
						}
					}
				}
				int total_size = down - up + right - left;
				encircle_size.push_back(total_size);
				point_total.push_back(point);
				point.clear();
			}
		}
	}
	int i_rec = 0;
	std::vector<cv::Point> point_outer;
	if (encircle_size.size() > 1)
	{
		for (int i = 1; i < encircle_size.size(); i++)
		{
			if (encircle_size[i] > encircle_size[i_rec])
			{
				i_rec = i;
			}
		}
		point_outer = point_total[i_rec];
	}
	else if (encircle_size.size())
	{
		point_outer = point_total[0];
	}
	else
	{
		;
	}

	cv::Mat outer_boder = cv::Mat::zeros(inputMeidanImage.rows, inputMeidanImage.cols, inputMeidanImage.type());
	if (point_outer.size())
	{
		for (int i = 0; i < point_outer.size(); i++)
		{
			uchar *data = outer_boder.ptr<uchar>(point_outer[i].y);
			data[point_outer[i].x] = 255;
		}
	}

	cv::imshow("outer_boder", outer_boder);

	if (point_outer.size() > 50)
	{
		auto fite = cv::fitEllipse(point_outer);
		cv::ellipse(outer_boder, fite, cv::Scalar(250, 0, 0), 1, CV_AA);
		cv::imshow("fittuoyuan", outer_boder);
		float hei_wid_scale = fite.size.height / fite.size.width;
		std::cout << (int)fite.angle << "  " << (int)fite.size.height <<"  "<<(int)fite.size.width<<"  "<< hei_wid_scale<< std::endl;
	}

	std::vector<cv::Point> point_rect;
	return point_rect;
}




std::vector<cv::Point> act::Recognition::find_contours_self(cv::Mat &inputMeidanImage, cv::Mat &FrisbeeBlack)
{
	cv::Mat border_many = cv::Mat::zeros(inputMeidanImage.rows, inputMeidanImage.cols, inputMeidanImage.type());

	for (int i = 0; i < inputMeidanImage.rows; i++)
	{
		uchar *data = inputMeidanImage.ptr<uchar>(i);
		uchar *data_bor = border_many.ptr<uchar>(i);

		for (int j = 0; j < inputMeidanImage.cols - 1; j++)
		{
			if (data[j] != data[j + 1])
			{
				if (!data[j])
					data_bor[j] = 255;
				else
					data_bor[j + 1] = 255;
			}
		}
		if (data[inputMeidanImage.cols - 1])
		{
			data_bor[inputMeidanImage.cols - 1] = 255;
		}
		else if (data[0])
		{
			data_bor[0] = 255;
		}
	}

	for (int j = 0; j < inputMeidanImage.cols; j++)
	{
		for (int i = 0; i < inputMeidanImage.rows - 1; i++)
		{
			uchar *data = inputMeidanImage.ptr<uchar>(i);
			uchar *data_next = inputMeidanImage.ptr<uchar>(i + 1);
			uchar *data_bor = border_many.ptr<uchar>(i);
			uchar *data_bor_next = border_many.ptr<uchar>(i + 1);
			if (data[j] != data_next[j])
			{
				if (!data[j])
					data_bor[j] = 255;
				else
					data_bor_next[j] = 255;
			}
		}
		uchar *data_ori = inputMeidanImage.ptr<uchar>(0);
		uchar *data_dat = border_many.ptr<uchar>(0);
		if (data_ori[j])
		{
			data_dat[j] = 255;
		}
		data_ori = inputMeidanImage.ptr<uchar>(inputMeidanImage.rows - 1);
		data_dat = border_many.ptr<uchar>(inputMeidanImage.rows - 1);
		if (data_ori[j])
		{
			data_dat[j] = 255;
		}
	}
	//cv::imshow("border_many_self", border_many);

	//cv::Mat cut_tuo_exx = cv::Mat::zeros(inputMeidanImage.rows, inputMeidanImage.cols, inputMeidanImage.type());
	//for (int i = 0; i < inputMeidanImage.rows; i++)
	//{
	//	uchar * data = cut_tuo_exx.ptr<uchar>(i);
	//	for (int j = 0; j < inputMeidanImage.cols; j++)
	//	{
	//		if (IF_INNER(abs(j - boxt.center.x), abs(i - boxt.center.y), OVAL_HEIGHT_HALT, OVAL_WIDTH_HALF))
	//		{
	//			data[j] = 255;
	//		}

	//	}
	//}
	//cv::imshow("cutpantuo_self", cut_tuo_exx);


	cv::Mat canny_co = border_many.clone();
	std::vector<std::vector<cv::Point>> point_total;

	std::vector<cv::Point> point;
	int point_many = 0;
	int round_in_sign = 0;
	cv::Mat zero_text = cv::Mat::zeros(inputMeidanImage.rows, inputMeidanImage.cols, inputMeidanImage.type());

	int jump = 0;
	for (int i = 0; i < inputMeidanImage.rows; i++)
	{
		uchar *data = canny_co.ptr<uchar>(i);
		for (int j = 0; j < inputMeidanImage.cols; j++)
		{
			if (data[j])
			{
				data[j] = 0;
				point.push_back(cv::Point(j, i));
				round_in_sign = 0;
				//记住有一个bug，如果起始点是(0,0)可能会出现不对的现象
				cv::Point last = cv::Point(j - 1, i);
				cv::Point center = cv::Point(j, i);
				cv::Point *text_eight;

				int signal = 1;
				while (signal)
				{
					signal = 0;
					text_eight = EightRegion(center, last, inputMeidanImage.rows, inputMeidanImage.cols);
					for (int i_i = 0; i_i < 7; i_i++)
					{
						int temp_y = text_eight[(i_i + 1)].y;
						int temp_x = text_eight[(i_i + 1)].x;
						uchar *in_data = canny_co.ptr<uchar>(temp_y);
						if (in_data[temp_x])
						{
							if (!round_in_sign)
							{
								if (IF_INNER(abs(temp_x - boxt.center.x), abs(temp_y - boxt.center.y), OVAL_HEIGHT_HALT, OVAL_WIDTH_HALF))
								{
									round_in_sign = 1;
								}
							}

							point.push_back(text_eight[(i_i + 1)]);
							in_data[temp_x] = 0;

							last = text_eight[i_i];
							center = text_eight[(i_i + 1)];
							signal = 1;
							break;
						}
					}

				}
				if (point.size() > 30)
				{
					if (round_in_sign)
						point_total.push_back(point);
					else
					{
						for (int k = 0; k < point.size(); k++)
						{
							int row_num = point[k].y;
							if (row_num > inputMeidanImage.rows - 10)
								continue;
							else
							{
								row_num = row_num + 3;
							}

							int col_num = point[k].x;
							uchar *data_down_bla = FrisbeeBlack.ptr<uchar>(row_num);
							while (data_down_bla[col_num])
							{
								if (row_num > (inputMeidanImage.rows - 10))
									break;
								if (IF_INNER(abs(col_num - boxt.center.x), abs(row_num - boxt.center.y), OVAL_HEIGHT_HALT, OVAL_WIDTH_HALF))
								{
									round_in_sign = 1;
									break;
								}
								row_num++;
								data_down_bla = FrisbeeBlack.ptr<uchar>(row_num);
							}
							if (round_in_sign)
							{
								point_total.push_back(point);
								break;
							}


						}
					}
				}

				point.clear();
			}
		}

	}

	std::vector<cv::Point> point_rect;
	//std::cout << point_total.size() << std::endl;
	if (point_total.size())
	{
		for (int i = 0; i < point_total.size(); i++)
		{
			int up, down, left, right;
			up = point_total[i][0].y;
			down = point_total[i][0].y;
			left = point_total[i][0].x;
			right = point_total[i][0].x;
			for (int j = 1; j < point_total[i].size(); j++)
			{
				if (point_total[i][j].x > right)
					right = point_total[i][j].x;
				if (point_total[i][j].x < left)
					left = point_total[i][j].x;
				if (point_total[i][j].y > down)
					down = point_total[i][j].y;
				if (point_total[i][j].y < up)
					up = point_total[i][j].y;

				uchar *data = zero_text.ptr<uchar>(point_total[i][j].y);
				data[point_total[i][j].x] = 255;
			}
			//point_rect.push_back(cv::Point(left, up));
			//point_rect.push_back(cv::Point(right, up));
			//point_rect.push_back(cv::Point(right, down));
			//point_rect.push_back(cv::Point(left, down));
			if (point_rect.size())
			{
				int signal_contain = 0;
				for (int k = 0; k < point_rect.size() / 4; k++)
				{
					//cv::Point regin_rang_st = point_rect[4 * k];
					//cv::Point regin_rang_end = point_rect[4 * k + 2];
					if (JudgePointRect(point_rect[4 * k], point_rect[4 * k + 2], cv::Point(left, up)) == TRUE)
					{
						if (JudgePointRect(point_rect[4 * k], point_rect[4 * k + 2], cv::Point(right, down)) == TRUE)
						{
							signal_contain = 1;
							break;
						}

					}
				}
				if (!signal_contain)
				{
					point_rect.push_back(cv::Point(left, up));
					point_rect.push_back(cv::Point(right, up));
					point_rect.push_back(cv::Point(right, down));
					point_rect.push_back(cv::Point(left, down));
				}
			}
			else
			{
				point_rect.push_back(cv::Point(left, up));
				point_rect.push_back(cv::Point(right, up));
				point_rect.push_back(cv::Point(right, down));
				point_rect.push_back(cv::Point(left, down));
			}


		}
	}
	cv::imshow("zero_text_self", zero_text);
	return point_rect;
	/*std::vector<cv::Point> point_pos;
	if (point_rect.size())
	{
	int i_rect = point_rect.size() >> 2;
	for (int i = 0; i <i_rect; i++)
	{
	int cir_i = i * 4;
	cv::line(zero_text, point_rect[cir_i], point_rect[cir_i + 1], cv::Scalar(250, 255, 255));
	cv::line(zero_text, point_rect[cir_i + 1], point_rect[cir_i + 2], cv::Scalar(250, 255, 255));
	cv::line(zero_text, point_rect[cir_i + 2], point_rect[cir_i + 3], cv::Scalar(250, 255, 255));
	cv::line(zero_text, point_rect[cir_i + 3], point_rect[cir_i], cv::Scalar(250, 255, 255));

	int x = (point_rect[cir_i].x + point_rect[cir_i + 2].x) >> 1;
	int y = (point_rect[cir_i].y + point_rect[cir_i + 2].y) >> 1;

	cv::Point send = cv::Point(x, y);
	point_pos.push_back(send);
	}
	}
	cv::imshow("zero_text", zero_text);
	return point_pos;*/
}

bool JudgePointRect(cv::Point st, cv::Point end, cv::Point judgepoint)
{
	if (judgepoint.x > st.x&&judgepoint.x < end.x)
	{
		if (judgepoint.y > st.y&&judgepoint.y < end.y)
		{
			return TRUE;
		}
	}
	return FALSE;
}
cv::Point *EightRegion(cv::Point center, cv::Point last,int rows,int cols)
{
	
	cv::Point clockwise[8] = { { -1, -1 } ,{ 0, -1 },{ 1, -1 },{ 1, 0 },{ 1, 1 },{ 0, 1 },{ -1, 1 },{ -1, 0 } };  // 顺时针
	cv::Point dert_x_y = last - center;
	int i = 0;
	for (i = 0; i < 8; i++)
	{
		if (dert_x_y == clockwise[i])
			break;
	}
	int start_num = (i) % 8;

	int reg_i = 0;
	for (int j = 0; j < 8; j++)
	{
		reg_i = (start_num + j) % 8;
		eight_nei[j] = center + clockwise[reg_i];
		if (eight_nei[j].x < 0 || eight_nei[j].y < 0)
		{
			if (j == 0)
			{
				;
				
//#ifdef DEBUG_8_REGION
//				std::cout << "less than 0 error" << std::endl;
//#endif
			}
			//eight_nei[j] = eight_nei[j - 1];
			eight_nei[j] = center;
		}
		else if (eight_nei[j].x >= cols || eight_nei[j].y >= rows)
		{
			if (j == 0)
			{
				;
//#ifdef DEBUG_8_REGION
//				std::cout << "bigger than max error" << std::endl;
//#endif
			}
			/*eight_nei[j] = eight_nei[j - 1];*/
			eight_nei[j] = center;
		}
	}
	return eight_nei;
}

int SureHaveBall(cv::Point platform, cv::Mat &inputImage)
{
	int x_st = platform.x;
	int y_st = platform.y;
	if (x_st < 2 || y_st < 2)
	{
		std::cout << "yanzhon error xiaoyu2" << std::endl;
		return 0;
	}
	int count_many = 0;
	int signal_have_ball = 0;
	int up_go = y_st;
	do {
		uchar *data = inputImage.ptr<uchar>(y_st);
		if (data[x_st] > 120)
		{
			y_st--;
			count_many++;
			if (count_many > 13)
			{
				signal_have_ball = 1;
				break;
			}
			
		}
		else {
			if (up_go - y_st > 2)
			{
				break;			
			}
			y_st--;
		}

	} while (y_st>0);


	//std::cout << count_many << std::endl;
	if (signal_have_ball)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

int SureHaveBallNew(cv::Point platform, cv::Mat &inputImage_rgb, Point3Kind white_plat)
{
	int x_st = platform.x;
	int y_st = platform.y - 3;//越过临界点
	if (x_st < 2 || y_st < 2)
	{
		std::cout << "yanzhon error smaller than  2" << std::endl;
		return 0;
	}
	
	int signal_have_ball = 0;
	int countmany = 0;
	int countmany_x_left = 0;
	int countmany_x_right = 0;

	int j = x_st * 3;

	while ((y_st-3) > 0)
	{
		uchar *data = inputImage_rgb.ptr<uchar>(y_st);
		uchar *data_last3 = inputImage_rgb.ptr<uchar>(y_st - 3);
		y_st--;

		if (abs(data[j] - data_last3[j] < 40) && abs(data[j + 1] - data_last3[j + 1] < 40) && abs(data[j + 2] - data_last3[j + 2] < 40))
		{
			if (abs(data[j] - white_plat.b) > 60 || abs(data[j + 1] - white_plat.g)> 60 || abs(data[j + 2] - white_plat.r)> 60)
			{
				countmany++;
				if (countmany > 5)
				{
					int jj = x_st;

					while ((jj - 3) > 0)
					{
						int jjj = jj * 3;
						int jjj_la3 = (jj - 3) * 3;
						jj--;
						if (abs(data[jjj] - data[jjj_la3]) < 40 && abs(data[jjj + 1] - data[jjj_la3 + 1]) < 40 && abs(data[jjj + 2] - data[jjj_la3 + 2]) < 40)
						{
							countmany_x_left++;
							if (countmany_x_left > 5)
								break;
						}
						else
						{
							break;
						}
					}
					jj = x_st;
					while ((jj + 3) < inputImage_rgb.cols - 1)
					{
						int jjj = jj * 3;
						int jjj_la3 = (jj + 3) * 3;
						jj++;
						if (abs(data[jjj] - data[jjj_la3]) < 40 && abs(data[jjj + 1] - data[jjj_la3 + 1]) < 40 && abs(data[jjj + 2] - data[jjj_la3 + 2]) < 40)
						{
							countmany_x_right++;
							if (countmany_x_right > 5)
								break;
						}
						else
						{
							break;
						}
					}
					break;
				}
			}
			else
			{
				break;
			}
			
		}
		else
		{
			break;
		}
	}

	if ((countmany_x_right+countmany_x_left)>5)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
int SureHaveBallStablePoint(cv::Point platform, cv::Mat &inputImage_rgb, Point3Kind white_plat)
{
	cv::Point start_up5_center;
	cv::Point start_up13_center;
	cv::Point start_up13_left30_center;
	cv::Point start_up13_right30_center;

	start_up5_center = platform + cv::Point(0, -5);
	start_up13_center = platform + cv::Point(0, -10);
	start_up13_left30_center = platform + cv::Point(-30, -10);
	start_up13_right30_center = platform + cv::Point(30, -10);
	//因为取3*3范围所以需要与3比较 意外判断
	if (start_up5_center.y < 5) {
		start_up5_center.y = 5;
	}
	if (start_up13_center.y < 3){
		start_up13_center.y = 3;
	}
	if (start_up13_left30_center.y < 3){
		start_up13_left30_center.y = 3;
	}	
	if (start_up13_right30_center.y < 3){
		start_up13_right30_center.y = 3;
	}
	if (start_up13_left30_center.x < 3) {
		start_up13_left30_center.x = 3;
	}
	if (start_up13_right30_center.x > (inputImage_rgb.cols-4)) {
		start_up13_right30_center.x = inputImage_rgb.cols - 4;
	}
	//第一个阶段判断向上移动5个距离的3*3与背景色差别 
	SimilarSigColor res_state1;
	res_state1 = similarity(inputImage_rgb, start_up5_center, white_plat, 2);
	if (!res_state1.signal)
		return 0;
	//第二个阶段判断向上移动13个距离的3*3与取色点的RGB差距
	SimilarSigColor res_state2;
	res_state2 = similarity(inputImage_rgb, start_up13_center, res_state1.color, 1);
	if (!res_state2.signal)
		return 0;
	//第三个阶段判断左右各移动30个距离的3*3与取色点的RGB差距
	SimilarSigColor res_state3_left, res_state3_right;
	res_state3_left = similarity(inputImage_rgb, start_up13_left30_center, res_state2.color, 0);
	if (!res_state3_left.signal)
		return 0;

	res_state3_right = similarity(inputImage_rgb, start_up13_right30_center, res_state2.color, 0);
	if (!res_state3_left.signal)
		return 0;
	return 1;
}


SimilarSigColor similarity(cv::Mat &inputImage_rgb, cv::Point center1, Point3Kind white_plat,int same_dif_sig)
{
	SimilarSigColor res;
	int count_fit = 0;
	res.color.b = 0; res.color.g = 0; res.color.r = 0;
	int color_r = 0, color_g = 0, color_b = 0;

	//球上的颜色周围判断
	if (same_dif_sig == 1)
	{
		for (int i = center1.y - 1; i < center1.y + 2; i++)//-1 0 1
		{
			uchar *data = inputImage_rgb.ptr<uchar>(i);
			for (int j = center1.x - 3; j < center1.x + 4; j++)// -3 -2 -1 0 1 2 3
			{
				int temp = j * 3;
				int div_b = (data[temp] - white_plat.b);
				int div_g = (data[temp + 1] - white_plat.g);
				int div_r = (data[temp + 2] - white_plat.r);
				if ( abs(div_b)<40 && abs(div_g)<40 && abs(div_r)<40)
				{
					color_b += data[temp];
					color_g += data[temp + 1];
					color_r += data[temp + 2];
					count_fit++;
				}
			}
		}
		if (count_fit >15)
		{
			uchar data_b = (uchar)(color_b / count_fit);
			uchar data_g = (uchar)(color_g / count_fit);
			uchar data_r = (uchar)(color_r / count_fit);
			res.color.b = data_b; res.color.g = data_g; res.color.r = data_r;
			res.signal = 1;
		}
		else
			res.signal = 0;		
	}
	//球的颜色和白色区别
	else if (same_dif_sig == 2)
	{
		for (int i = center1.y - 3; i < center1.y + 3; i++)//-3 -2 -1 0 1 2
		{
			uchar *data = inputImage_rgb.ptr<uchar>(i);
			for (int j = center1.x - 1; j < center1.x + 2; j++)//-1 0 1
			{
				int temp = j * 3;
				int div_b = (data[temp] - white_plat.b);
				int div_g = (data[temp + 1] - white_plat.g);
				int div_r = (data[temp + 2] - white_plat.r);
				if (abs(div_b) > 50 || abs(div_g) > 50 || abs(div_r) > 50 || abs(div_b - div_g) > 50 || abs(div_b - div_r) > 50 || abs(div_g - div_r) > 50)
				{
					color_b += data[temp];
					color_g += data[temp + 1];
					color_r += data[temp + 2];
					count_fit++;
				}
			}
		}
		if (count_fit > 15)
		{
			uchar data_b = (uchar)(color_b / count_fit);
			uchar data_g = (uchar)(color_g / count_fit);
			uchar data_r = (uchar)(color_r / count_fit);
			res.color.b = data_b; res.color.g = data_g; res.color.r = data_r;
			res.signal = 1;
		}
		else
			res.signal = 0;
	}
	else
	{
		for (int i = center1.y - 1; i < center1.y + 2; i++)
		{
			uchar *data = inputImage_rgb.ptr<uchar>(i);
			for (int j = center1.x - 1; j < center1.x + 2; j++)
			{
				int temp = j * 3;
				int div_b = (data[temp] - white_plat.b);
				int div_g = (data[temp + 1] - white_plat.g);
				int div_r = (data[temp + 2] - white_plat.r);
				if (abs(div_b) > 30 || abs(div_g) > 30 || abs(div_r) > 30)
				{
					color_b += data[temp];
					color_g += data[temp + 1];
					color_r += data[temp + 2];
					count_fit++;
				}
			}
		}
		if (count_fit > 4)
		{
			uchar data_b = (uchar)(color_b / count_fit);
			uchar data_g = (uchar)(color_g / count_fit);
			uchar data_r = (uchar)(color_r / count_fit);
			res.color.b = data_b; res.color.g = data_g; res.color.r = data_r;
			res.signal = 1;
		}
		else
			res.signal = 0;
	}
	return res;
}
int SureHaveBallMod(cv::Point platform, cv::Mat &inputImage)
{
	int x_st = platform.x ;
	int y_st = platform.y ;
	if (x_st < 2 || y_st < 2)
	{
		std::cout << "yanzhon error xiaoyu2" << std::endl;
		return 0;
	}
	int count_many = 0;
	int signal_have_ball = 0;
	int up_go = y_st;
	do {
		uchar *data = inputImage.ptr<uchar>(y_st);
		if (data[x_st*3+1] > 120)
		{
			y_st--;
			count_many++;
			if (count_many > 15)
			{
				signal_have_ball = 1;
				break;
			}

		}
		else if ((data[x_st * 3 + 1] < 80) && (data[x_st * 3 + 2] < 120))
		{
			uchar *datalast = inputImage.ptr<uchar>(y_st-1);
			if (abs(datalast[x_st * 3 + 1] - data[x_st * 3 + 1]) < 25)
			{
				count_many++;
				if (count_many > 10)
				{
					signal_have_ball = 1;
					break;
				}
			}
			else
				break;

		}
		else {
			if (up_go - y_st > 2)
			{
				break;
			}
			y_st--;
		}

	} while (y_st>1);
	if (signal_have_ball)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
int SureHaveFirsbee(OVAL_parameter &platformg, cv::Mat &inputImage,char color)
{
	//OVAL_parameter platform;
	//platform.angle = platformg.angle;
	//platform.center = platformg.center;
	//platform.height = platformg.height*0.9;
	//platform.width = platformg.width*0.9;


	cv::Mat kuang = cv::Mat::zeros(inputImage.size(), 0);
	cv::RotatedRect boxtemp;
	boxtemp.angle = platformg.angle;
	boxtemp.center= platformg.center;
	boxtemp.size.height = platformg.height * 9 / 10;
	boxtemp.size.width = platformg.width * 9 / 10;

	ellipse(kuang, boxtemp, cv::Scalar(255, 0, 0), 1, 8);

	for (int i = 0; i < kuang.rows; i++)
	{
		uchar *data = kuang.ptr<uchar>(i);
		int j_st = 0,j_end = 0;
		for (int j = 0; j < kuang.cols; j++)
		{
			if (data[j])
			{
				j_st = j;
				break;
			}
		}
		if (j_st == (kuang.cols - 1))
		{
			continue;
		}
		for (int j = kuang.cols-1; j>0; j--)
		{
			if (data[j])
			{
				j_end = j;
				break;
			}
		}
		for (int j = j_st; j < j_end; j++)
		{
			data[j] = 255;
		}		
	}
	cv::imshow("kuang", kuang);

	int ball_signal = 0;
	cv::Mat tai=cv::Mat::zeros(inputImage.size(), 0);
	for (int i = 0; i < inputImage.rows; i++)
	{
		uchar *data = inputImage.ptr<uchar>(i);
		uchar *datatai = tai.ptr<uchar>(i);
		for (int j = 0; j < inputImage.cols; j++)
		{
			int temp = j * 3;
			if (color == 'r')
			{
				if (data[temp + 1] > 100 && ((data[temp] > 160)|| data[temp] <5)/* && data[temp + 2] > 40 && data[temp + 2] < 140*/)
				{
					datatai[j] = 255;
				}
			}
			else
			{
				if (data[temp + 1] > 60 && (data[temp] > 100) && (data[temp] < 130)&& (data[temp+2] < 180)/* && data[temp + 2] > 40 && data[temp + 2] < 140*/)
				{
					datatai[j] = 255;
				}
			}
		}
	}

	cv::imshow("yuan", tai);
	//cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,1));
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 2));

	//进行形态学操作
	morphologyEx(tai, tai, CV_MOP_ERODE, element);
	//显示效果图  
	cv::imshow("【效果图】开运算", tai);

	for (int i = 0; i < inputImage.rows; i++)
	{
		uchar *datatai = tai.ptr<uchar>(i);
		uchar *datakuan= kuang.ptr<uchar>(i);

		for (int j = 0; j < inputImage.cols; j++)
		{
			if(datatai[j])
			{
				if (datakuan[j])
				{
					ball_signal = 1;
					return 1;
				}
				else
				{
					int ii = i;
					while (ii < inputImage.rows-3)
					{
						ii++;
						//uchar *datatai = tai.ptr<uchar>(ii);
						uchar *datakuan_in = kuang.ptr<uchar>(ii);
						uchar *data = inputImage.ptr<uchar>(ii);

						if (data[3*j+1] < 80&& data[3 * j + 2] < 80)
						{
							if (datakuan_in[j])
							{
								ball_signal = 1;
								return 1;
							}
						}
						else
						{
							if (ii - i > 3)
								break;
						}
					}

				}

				//double aa = sqrt(pow((j - left_jiaodian.x), 2) + pow((i - left_jiaodian.y), 2)) + sqrt(pow((j - right_jiaodian.x), 2) + pow((i - right_jiaodian.y), 2));
				//if (aa < platform.height)
				//{

				//}
			}		
		}
	}
	return 0;
}

cv::Mat thresholdHSV(cv::Mat &inputImage, int v_que,int s_que) 
{
	cv::Mat temp = cv::Mat::zeros(inputImage.size(), CV_8UC1);
	for (int i = 0; i < inputImage.rows; ++i) 
	{
		uchar *data = temp.ptr<uchar>(i);
		uchar *data_act = inputImage.ptr<uchar>(i);

		for (int j = 0; j < inputImage.cols; ++j) 
		{
			int act_j = j * 3;
			if (data_act[act_j + 1]<s_que&&data_act[act_j + 2]>v_que)
			{
				data[j] = 255;
			}
		}
	}
	return temp;
}
int JudgeInthresholdRange(OVAL_parameter &platform, double angle)
{
	float Difference = abs(platform.angle_ref - angle);
	if (Difference < platform.angle_dert)
		return 1;
	else
		return 0;
}
int JudgeCenterInthresholdRange(OVAL_parameter &platform, cv::Point center_act)
{
	int Difference_x = abs(platform.center_ref.x - center_act.x);
	int Difference_y = abs(platform.center_ref.y - center_act.y);

	if (Difference_x > 15 || Difference_y > 10)
	{
		return 0;
	}
	else
		return 1;
}

void act::Recognition::PlatValInit(cv::Mat &inputImage,int s_que, PlatPara &platparameter, OVAL_parameter &platform)
{
	while (platparameter.v_val > platparameter.v_val_end)
	{
		cv::Mat plat_rgb = inputImage.clone();
		platform.v_val = platparameter.v_val;
		platparameter.v_val -= platparameter.v_val_div;

		cv::Mat platform_hsv;
		cv::cvtColor(plat_rgb, platform_hsv, cv::COLOR_BGR2HSV);
		cv::Mat zhuolu_que = cv::Mat::zeros(plat_rgb.size(), 0);

		for (int i = 0; i < platform_hsv.rows; i++)
		{
			uchar *data = zhuolu_que.ptr<uchar>(i);
			uchar *datasv = platform_hsv.ptr<uchar>(i);
			for (int j = 0; j < platform_hsv.cols; j++)
			{
				int jj = j * 3;
				if (datasv[jj + 2] > platform.v_val&&datasv[jj + 1] < s_que)
				{
					data[j] = 255;
				}
			}
		}

		cv::Mat median_zhuolu;
		cv::medianBlur(zhuolu_que, median_zhuolu, 5);
		cv::imshow("median_zhuolu", median_zhuolu);
		cv::Mat outer_edges_image;
		cv::Canny(median_zhuolu, outer_edges_image, 3, 9, 3);
		cv::imshow("outer_edges_image", outer_edges_image);

		MatchValAngCenter res = MatchPartRandInit(outer_edges_image, plat_rgb, platform);
		if (res.maxValue > platparameter.maxValue_value)
		{
			if (JudgeCenterInthresholdRange(platform, res.center))
			{
				platparameter.maxValue_value = res.maxValue;
				platparameter.maxValue_angle = res.angle;
				platparameter.maxValue_V_val = platform.v_val;
				platform.center_ref = res.center;
				std::cout << " v " << platparameter.maxValue_V_val << " ang  " << platparameter.maxValue_angle << std::endl;
			}
			else
			{
				std::cout << "position center no fit "<< std::endl;
			}

		}
	}
	platform.v_val = platparameter.maxValue_V_val;
	platform.angle = platparameter.maxValue_angle;
	int line_st = platform.center_ref.x - platform.height / 2;
	int line_end= platform.center_ref.x + platform.height / 2;
	if (line_st < 0)
	{
		line_st = 0;
	}
	if (line_end > platform.RectEnd.x - platform.RectSt.x - 1)
	{
		line_end = platform.RectEnd.x - platform.RectSt.x - 1;
	}

	Point3Kind temp;
	uchar *color_data = inputImage.ptr<uchar>(platform.center_ref.y);
	int col_num = platform.center_ref.x * 3;
	temp.b = color_data[col_num];
	temp.g = color_data[col_num + 1];
	temp.r = color_data[col_num + 2];
	int temp_total = temp.b + temp.g + temp.r;
	for (int i = platform.center_ref.x; i < platform.center_ref.x + platform.height / 2; i++)
	{
		col_num = i * 3;
		int total = color_data[col_num] + color_data[col_num + 1] + color_data[col_num + 2];
		if (total > temp_total)
		{
			temp_total = total;
			temp.b = color_data[col_num];
			temp.g = color_data[col_num + 1];
			temp.r = color_data[col_num + 2];
		}
	}
	for (int i = platform.center_ref.x; i > platform.center_ref.x - platform.height / 2; i--)
	{
		col_num = i * 3;
		int total = color_data[col_num] + color_data[col_num + 1] + color_data[col_num + 2];
		if (total > temp_total)
		{
			temp_total = total;
			temp.b = color_data[col_num];
			temp.g = color_data[col_num + 1];
			temp.r = color_data[col_num + 2];
		}
	}
	platform.White_Point.b = temp.b;
	platform.White_Point.g = temp.g;
	platform.White_Point.r = temp.r;

	std::cout << "finally: v " << platparameter.maxValue_V_val << " ang  " << platparameter.maxValue_angle;
	std::cout << " b" << (int)platform.White_Point.b << " g" << (int)platform.White_Point.g << "  r " << (int)platform.White_Point.r << std::endl;
}
void ChangeLight(cv::Mat &inputImage, OVAL_parameter &platform, int max_val)
{
	for (int i = 0; i < inputImage.rows; i++)
	{
		uchar *data = inputImage.ptr<uchar>(i);
		for (int j = 0; j < inputImage.cols; j++)
		{
			int temp = j * 3;
			int val_b = data[temp] * max_val / platform.White_Point.b;
			int val_g = data[temp + 1] * max_val / platform.White_Point.g;
			int val_r = data[temp + 2] * max_val / platform.White_Point.r;
			if (val_b > 255)
				val_b = 255;
			if (val_g > 255)
				val_g = 255;
			if (val_r > 255)
				val_r = 255;
			data[temp] = val_b;
			data[temp + 1] = val_g;
			data[temp + 2] = val_r;
		}
	}
}
void Change1245Vval(cv::Mat &inputImage, OVAL_parameter &platform, int max_val)
{
	if (platform.White_Point.b > max_val || platform.White_Point.g > max_val || platform.White_Point.r > max_val)
	{
		;
	}
	else {
		ChangeLight(inputImage, platform, max_val);
	}
	return;
}
void WhiteBalancePicture(cv::Mat &inputImage, WBAdjust &wbpram)
{
	for (int i = 0; i < inputImage.rows; i++)
	{
		uchar *data = inputImage.ptr<uchar>(i);
		for (int j = 0; j < inputImage.cols; j++)
		{
			int data_r = 0, data_g = 0, data_b = 0;
			int temp = j * 3;
			data_b = data[temp] * wbpram.b_prop_coeff;
			data_g = data[temp + 1] * wbpram.g_prop_coeff;
			data_r = data[temp + 2] * wbpram.r_prop_coeff;
			data_b = data_b > 255 ? 255 : data_b;
			data_g = data_g > 255 ? 255 : data_g;
			data_r = data_r > 255 ? 255 : data_r;
			data[temp] = data_b;
			data[temp + 1] = data_g;
			data[temp + 2] = data_r;
		}
	}
}
#define NUM_FRAME 300 //只处理前300帧，根据视频帧数可修改
void Image_to_video()
{
	int i = 0;
	IplImage* img = 0;
	char image_name[13];
	printf("------------- image to video ... ----------------n");
	//初始化视频编写器，参数根据实际视频文件修改
	CvVideoWriter *writer = 0;
	int isColor = 1;
	int fps = 30; // or 25
	int frameW = 400; // 744 for firewire cameras
	int frameH = 240; // 480 for firewire cameras
	writer = cvCreateVideoWriter("out.avi", CV_FOURCC('X', 'V', 'I', 'D'), fps, cvSize(frameW, frameH), isColor);
	printf("tvideo height : %dntvideo width : %dntfps : %dn", frameH, frameW, fps);
	//创建窗口
	cvNamedWindow("mainWin", CV_WINDOW_AUTOSIZE);
	while (i<NUM_FRAME)
	{
		sprintf(image_name, "%s%d%s", "image", ++i, ".jpg");
		img = cvLoadImage(image_name);
		if (!img)
		{
			printf("Could not load image file...n");
			exit(0);
		}
		cvShowImage("mainWin", img);
		char key = cvWaitKey(20);
		cvWriteFrame(writer, img);
	}
	cvReleaseVideoWriter(&writer);
	cvDestroyWindow("mainWin");
}

unsigned char Track(std::vector<cv::Point> disc_pos, int signal_disc, LandBoundary &landboundary)
{
	static std::vector<cv::Point> disc_pos_record;
	static std::vector<cv::Point> disc_pos_contin_exist;
	static cv::Point disc_pos_contin_times[9];
	static int contin_times[9];
	static int count_add_num = 0;



	uchar send_data_comb = 0;
	std::vector<cv::Point> disc_standstill;

	//如果没有飞盘 将记录的盘子清空
	if (!signal_disc)
	{
		disc_pos_record.clear();
		disc_pos_contin_exist.clear();
		for (int i = 0; i < 9; i++)
		{
			disc_pos_contin_times[i].x = 0;
			disc_pos_contin_times[i].y = 0;
			contin_times[i] = 0;
		}
		return 0;
	}

	if (disc_pos_record.size())
	{
		for (int i = 0; i < disc_pos.size(); i++)
		{
			for (int j = 0; j < disc_pos_record.size(); j++)
			{
				if (disc_pos[i].x == disc_pos_record[j].x&&disc_pos[i].y == disc_pos_record[j].y)
				{
					disc_standstill.push_back(disc_pos[i]);
					break;
				}
			}
		}
	}
	//最后需要将上一个周期所有的点存放起来 下一个周期进行对比
	disc_pos_record.clear();
	for (int i = 0; i < disc_pos.size(); i++)
	{
		disc_pos_record.push_back(disc_pos[i]);
	}
	if (disc_standstill.size())
	{
		for (int j = 0; j < 9; j++)
		{
			int signal = 0;
			for (int i = 0; i < disc_standstill.size(); i++)
			{
				if (disc_pos_contin_times[j].x == disc_standstill[i].x&&disc_pos_contin_times[j].y == disc_standstill[i].y)
				{
					signal = 1;
					break;
				}
			}
			if (!signal)
			{
				disc_pos_contin_times[j].x = 0;
				disc_pos_contin_times[j].y = 0;
				contin_times[j] = 0;
			}
			else
			{
				contin_times[j]++;
				if (contin_times[j] > 10)
					contin_times[j] = 10;
			}

		}
		for (int i = 0; i < disc_standstill.size(); i++)
		{
			int count_diff = 0;
			for (int j = 0; j < 9; j++)
			{
				if (disc_pos_contin_times[j].x != disc_standstill[i].x||disc_pos_contin_times[j].y != disc_standstill[i].y)
				{
					count_diff++;
				}
			}
			if (count_diff == 9)
			{
				for (int j = 0; j < 9; j++)
				{
					if (!disc_pos_contin_times[j].x && !disc_pos_contin_times[j].y)
					{
						contin_times[j] = 1;
						disc_pos_contin_times[j].x = disc_standstill[i].x;
						disc_pos_contin_times[j].y = disc_standstill[i].y;
						break;
					}
					if (j == 8)
						std::cout << "plenty disc error" << std::endl;
				}
			}
		}
	}
	else
	{
		for (int i = 0; i < 9; i++)
		{
			disc_pos_contin_times[i].x = 0;
			disc_pos_contin_times[i].y = 0;
			contin_times[i] = 0;
		}
	}



	for (int i = 0; i < 9; i++)
	{
		if (contin_times[i] > 2)
		{
			if (disc_pos_contin_times[i].x < landboundary.one_col)       //0 1  60
			{
				if (disc_pos_contin_times[i].y < landboundary.row_l)
					send_data_comb |= 1 << 0;
				else
					send_data_comb |= 1 << 1;

			}
			else if (disc_pos_contin_times[i].x < landboundary.two_col)//80
			{
				if (disc_pos_contin_times[i].y < landboundary.row_m)
					send_data_comb |= 1 << 2;
				else
					send_data_comb |= 1 << 3;
			}
			else//68
			{
				if (disc_pos_contin_times[i].y <landboundary.row_r)
					send_data_comb |= 1 << 4;
				else
					send_data_comb |= 1 << 5;
			}
		}

	}
	return send_data_comb;

}

unsigned char NotTrack(std::vector<cv::Point> disc_pos, int signal_disc, LandBoundary &landboundary)
{
	static std::vector<cv::Point> disc_pos_record;
	static std::vector<cv::Point> disc_pos_contin_exist;
	static cv::Point disc_pos_contin_times[9];
	static int contin_times[9];
	static int count_add_num = 0;


	uchar send_data_comb = 0;
	if (disc_pos.size())
	{
		for (int i = 0; i < disc_pos.size(); i++)
		{
			if (disc_pos[i].x < landboundary.one_col)       //0 1  60
			{
				if (disc_pos[i].y < landboundary.row_l)
					send_data_comb |= 1 << 0;
				else
					send_data_comb |= 1 << 1;

			}
			else if (disc_pos[i].x < landboundary.two_col)//80
			{
				if (disc_pos[i].y < landboundary.row_m)
					send_data_comb |= 1 << 2;
				else
					send_data_comb |= 1 << 3;
			}
			else//68
			{
				if (disc_pos[i].y <landboundary.row_r)
					send_data_comb |= 1 << 4;
				else
					send_data_comb |= 1 << 5;
			}
		}
	}
	return send_data_comb;

}