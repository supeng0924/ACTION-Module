#ifndef __FILEDEAL_H
#define __FILEDEAL_H
#include <vector>
#include <string>
using namespace std;
// #define DEBUGTIME
#define COLOR_PATH "color/"
#define DEPTH_PATH "depth/"
#define XYZ_PATH   "xyz/"
#define XDATA_PATH   "xdata/"

#define COLOR_UP_SHOW "up_color"
#define COLOR_DOWN_SHOW "down_color"

#define DEPTH_UP_SHOW "up_depth"
#define DEPTH_DOWN_SHOW "down_depth"
#define WINDOW_NAME COLOR_UP_SHOW


#define UP_RGB "UPRGB/"
#define DOWN_RGB "DOWNRGB/"
#define BACKUP_CAM_NUMBER "819612070967"

#define UP_SERIAL_NUMBER "819612072028"
// #define UP_SERIAL_NUMBER BACKUP_CAM_NUMBER
#define UP_COLOR_PATH 	"UPcolor/"
#define UP_DEPTH_PATH 	"UPdepth/"
#define UP_XDATA_PATH 	"UPxdata/"

#define DOWN_SERIAL_NUMBER "819112072323"
#define DOWN_COLOR_PATH "DOWNcolor/"
#define DOWN_DEPTH_PATH "DOWNdepth/"
#define DOWN_XDATA_PATH "DOWNxdata/"



#define SAMPLE_SAVE_IMAGE_PATH "sampleImg/"

#define RECOGNITION_MODE        //识别中，采集图片模式
// #define SAMPLING_MODE           //采集图片，训练模式
// #define SHOW_IMAGE_MODE            //显示图片模式

// 扫描PATH_SCAN路径下的文件
vector<string> getFile(string path);
// 将int类型转为string类型
std::string Int_to_String(int n);
// 清空扫描路径下的文件，进行照片的存放
void Clear_file(string path);
// 删除指定路径下的jpg文件
void Delete_file(string path,string suffix,int num);
// 删除png文件
void Delete_png_dep(string path);

#endif
