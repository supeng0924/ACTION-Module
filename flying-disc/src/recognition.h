#ifndef __RECOGNITION_H
#define __RECOGNITION_H

#include <vector>
#include <opencv2\opencv.hpp>



#define MAX_GAP  10
#define ALREADY(x,y) (abs(x-y)<MAX_GAP)
#define MAX_GAP_P  30
#define ALREADY_P(x,y) (abs(x-y)<MAX_GAP_P)
#define line_color cv::Scalar(0, 0, 0)
#define POINT_SIZE 10
#define IF_INNER(x,y,aa,bb)   (  (double)( pow((double)x/aa,2) + pow((double)y/bb,2 ) ) <0.8 )
#define X_GAP 3

#define JudeGray(s,v)  ( (s<130) && (v<70) )
typedef struct {
	bool oval_exist;
	int oval_exixt_num;
}FIT_OVAL;
typedef struct GainARGB
{
	int b;
	int g;
	int r;
	int analog;
}GainARGB;
typedef struct LandBoundary
{
	int one_col;
	int two_col;
	int two_row;
	int row_r;
	int row_m;
	int row_l;
}LandBoundary;
typedef struct {
	uchar b;
	uchar g;
	uchar r;
}Point3Kind;
typedef struct {
	Point3Kind color;
	int signal;
}SimilarSigColor;
typedef struct
{
	int start_x;
	int start_y;
	int end_x;
	int end_y;
}Calibration;
typedef struct {
	int plat_num;     //着陆台号
	int ballMark;     //着陆台上球判断
	int height;       //着陆台椭圆匹配高度
	int width;        //着陆台椭圆匹配宽度
	int v_val;        //着陆台椭圆匹配hsv空间中v限制
	int s_val;        //着陆台椭圆匹配hsv空间中v限制

	float angle;      //着陆台椭圆匹配角度
	int angle_10x;    //着陆台椭圆匹配角度*10，滑动条控制
	float angle_ref;  //着陆台椭圆匹配角度参考值
	float angle_dert; //着陆台椭圆匹配角度偏差范围允许

	cv::Point center; //着陆台椭圆匹配中心
	cv::Point center_ref; //着陆台椭圆匹配中心参考值
	cv::Size center_dert; //着陆台椭圆匹配中心偏差允许范围

	cv::Point RectSt;  //着陆台椭圆截取起始点
	cv::Point RectEnd; //着陆台椭圆截取终止点
	Point3Kind White_Point;//记录最高亮度点
}OVAL_parameter;

typedef struct {
	cv::Point first;
	cv::Point last;
}POINT_AND_POINT;

typedef struct matchresult
{
	double maxValue;
	cv::Point maxLocation;
}MatchRes;

typedef struct MatchValAng
{
	double maxValue;
	double angle;
}MatchValAng;
typedef struct MatchAngCenter
{
	double angle;
	cv::Point center;
}MatchAngCenter;
typedef struct 
{
	cv::Point center;
	double maxValue;
	double angle;
}MatchValAngCenter;

//曝光结构体
typedef struct ExporeTime
{
	double maxValue_value;     //曝光匹配最大值对应的值
	double maxValue_angle;     //曝光匹配最大值对应的角度
	double maxValue_exporeTime;//曝光匹配最大值对应的曝光时间
	double exporeTime;         //曝光时间临时变量
	double expore_time_start;  //曝光时间开始
	double expore_time_end;    //曝光时间结束
	double expore_time_div;    //曝光时间递减梯度
}ExporeTime;

//曝光结构体
typedef struct PlatPara
{
	double maxValue_value;  //匹配参数最大值对应最大值
	double maxValue_angle;  //匹配参数最大值对应角度
	int maxValue_V_val;     //匹配参数最大值对应HSV中V阈值

	int v_val;              //匹配时HSV中V阙值临时V值
	int v_val_start;        //匹配时V阙值起始值
	int v_val_end;          //匹配时V阙值终止值
	int v_val_div;          //匹配时V阙值递减梯度

}PlatPara;

//白平衡结构体
typedef struct WBAdjust
{
	cv::Point first;     //框选的起始点
	cv::Point last;      //框选的终止点
	int complete_signal;
	float b_prop_coeff;  //b（蓝色）比例系数
	float g_prop_coeff;  //g（绿色）比例系数
	float r_prop_coeff;  //r（红色）比例系数
}WBAdjust;

typedef struct VideoRecord
{
	CvVideoWriter *writer;
	int isColor;
	double fps;
	int frameW;
	int frameH;
}VideoRecord;
//typedef struct {
//	cv::Point point;
//	int num;
//}COL_INFOR;
namespace act {

	//FIT_OVAL GetFitOval(void);
	//void SetFItOvalExist(bool tar);
	//void SetFItOvalNum(int tar);
    /**
     * @class Recognition
     */
    class Recognition {
    public:
		Recognition() = default;
        Recognition(cv::Size size, int type, void* data) : original_image_(size, type, data) {
            cv::cvtColor(original_image_, hsv_image_, cv::COLOR_BGR2HSV);
        }

        Recognition(const Recognition&r) : original_image_(r.original_image_), hsv_image_(r.hsv_image_) {  }
        Recognition &operator=(const Recognition&r) { original_image_ = r.original_image_; hsv_image_ = r.hsv_image_; return *this; }
        ~Recognition() = default;

		void update(cv::Size size, int type, void* data);
		
		inline void setROIRect(const cv::Rect &rect) { roi_image_ = original_image_(rect);  cv::cvtColor(roi_image_, roi_hsv_image_, cv::COLOR_BGR2HSV); }
        inline std::vector<cv::Mat>  getHSVChannels() {
            std::vector<cv::Mat> chs(3);
            cv::split(hsv_image_, chs);
            return chs;
        }

        inline cv::Mat thresholdByHSV(bool(*_callback)(cv::Vec3b *)) {
            cv::Mat temp = cv::Mat::zeros(original_image_.size(), CV_8UC1);

            for (int i = 0; i < hsv_image_.rows; ++i) {
                for (int j = 0; j < hsv_image_.cols; ++j) {
                    if (_callback(hsv_image_.ptr<cv::Vec3b>(i, j))) {
                        *temp.ptr<uchar>(i, j) = 255;
                    }
                }
            }

            return temp;
        }
		inline cv::Mat thresholdByHSVpanzi(bool(*_callback)(cv::Vec3b *)) {
			cv::Mat temp = cv::Mat::zeros(roi_image_.size(), CV_8UC1);
			for (int i = 0; i < roi_hsv_image_.rows; ++i) 
			{
				for (int j = 0; j < roi_hsv_image_.cols; ++j) 
				{
					if (_callback(roi_hsv_image_.ptr<cv::Vec3b>(i, j))) {
						*temp.ptr<uchar>(i, j) = 255;
					}
				}
			}
			return temp;
		}
		//inline void SetOval(cv::RotatedRect tar) {
		//	oval_imfor = tar;
		//	SetFItOvalExist(true);
		//	SetFItOvalNum(GetFitOval().oval_exixt_num + 1);
		//}
		inline cv::RotatedRect GetOval(void) { return oval_imfor; }
		/**
		* \ 
		*/
	
		inline cv::Mat getOriginalImage() { return original_image_; }
		inline cv::Mat getHSVImage() { return hsv_image_; }
		inline cv::Mat getROIImage() { return roi_image_; }
		inline cv::Mat getROIHSVImage() { return roi_hsv_image_; }
		inline cv::Point getboxt_orgin_center() { return boxt_orgin_center; }

		inline cv::RotatedRect getBOX() { return boxt; }
	    int MatchPart(cv::Mat &frame_gray, cv::Mat &frame);
		cv::Point MatchPartRand(cv::Mat &frame_gray, cv::Mat &frame, OVAL_parameter &platform);
		cv::Point MatchPartNearPlat(cv::Mat &frame_gray, cv::Mat &frame, float angle, int height, int width);
		MatchValAng MatchPartNearInit(cv::Mat &frame_gray, cv::Mat &frame, OVAL_parameter &platform);
		MatchValAngCenter MatchPartRandInit(cv::Mat &frame_gray, cv::Mat &frame, OVAL_parameter &platform);
		MatchAngCenter MatchPartRandArrive(cv::Mat &frame_gray, cv::Mat &frame, OVAL_parameter &platform);
		cv::Mat EllipseJudg(cv::Mat &frame_transcript, cv::RotatedRect &boxtemp);
		cv::Mat EllipseShape(/*cv::Mat &frame_gray*/cv::Size size, cv::RotatedRect &boxtemp, float angle_Rotate);
		cv::Mat EllipseShapeBorder(cv::Size size, OVAL_parameter &platform, float angle_Rotate);
		MatchRes MatchTemImage(cv::Mat &frame_origin, cv::Mat &frame_template);
		MatchValAng MatchInit(cv::Mat &frame_gray, cv::Mat &frame, OVAL_parameter &platform);
		void PlatValInit(cv::Mat &inputImage, int s_que, PlatPara &platparameter, OVAL_parameter &platform);

		unsigned char act::Recognition::ColorDefine(void);

		std::vector<cv::Point> find_contours(cv::Mat &inputMeidanImage, cv::Mat &FrisbeeBlack);
		std::vector<cv::Point> find_contours_self(cv::Mat &inputMeidanImage, cv::Mat &FrisbeeBlack);
		std::vector<cv::Point> find_contours_te(cv::Mat &inputMeidanImage, cv::Mat &FrisbeeBlack, OVAL_parameter &platform);
		std::vector<cv::Point> find_contours_model(cv::Mat &inputMeidanImage, cv::Mat &FrisbeeBlack, OVAL_parameter &platform);

		double act::Recognition::MatchPart_Move(cv::Mat &frame_gray, cv::Mat &frame);
    private:
        cv::Mat original_image_;
        cv::Mat hsv_image_;
		cv::RotatedRect oval_imfor;
		cv::Mat roi_image_;                     // ROI
		cv::Mat roi_hsv_image_;                 // ROI 的 HSV图像
		cv::RotatedRect boxt;
		cv::Point boxt_orgin_center;

	
    };

	//std::vector<cv::Point> outerEdges_c3(cv::Mat &inputEdgesImage);//自动找中心线 + 连通性
	//std::vector<std::vector<cv::Point>> contours_c1(cv::Mat &inputMeidanImage, int a, int b, int x, int y, cv::Mat hsv);

	

	
}

cv::Point *EightRegion(cv::Point center, cv::Point last, int rows = 0, int cols = 0);
bool JudgePointRect(cv::Point st, cv::Point end, cv::Point judgepoint);
int SureHaveBall(cv::Point platform, cv::Mat &inputImage);
int SureHaveFirsbee(OVAL_parameter &platformg, cv::Mat &inputImage, char color);
int SureHaveBallMod(cv::Point platform, cv::Mat &inputImage);
cv::Mat thresholdHSV(cv::Mat &inputImage, int v_que, int s_que);
int JudgeInthresholdRange(OVAL_parameter &platform, double angle);
int JudgeCenterInthresholdRange(OVAL_parameter &platform, cv::Point center_act);
int SureHaveBallNew(cv::Point platform, cv::Mat &inputImage_rgb, Point3Kind white_plat);
SimilarSigColor similarity(cv::Mat &inputImage_rgb, cv::Point center1, Point3Kind white_plat, int same_dif_sig);
int SureHaveBallStablePoint(cv::Point platform, cv::Mat &inputImage_rgb, Point3Kind white_plat);
void ChangeLight(cv::Mat &inputImage, OVAL_parameter &platform, int max_val);
void Change1245Vval(cv::Mat &inputImage, OVAL_parameter &platform, int max_val);
void WhiteBalancePicture(cv::Mat &inputImage, WBAdjust &wbpram);
unsigned char Track(std::vector<cv::Point> disc_pos, int signal_disc, LandBoundary &landboundary);
unsigned char NotTrack(std::vector<cv::Point> disc_pos, int signal_disc, LandBoundary &landboundary);
#endif // !__RECOGNITION_H