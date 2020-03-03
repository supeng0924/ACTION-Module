#include <iostream>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include "filedeal.h"
#include <time.h>
#include <pthread.h>
#include <thread>
#include <string>
#include <map>
#include <algorithm>
#include <mutex>                    // std::mutex, std::lock_guard
#include <cmath>                    // std::ceil

// 计时函数
// clock_t time_start=clock();
// cout<<"deal "<<((double)(clock()- time_start) / CLOCKS_PER_SEC)<<endl;
using namespace std;
using namespace cv;
// 上摄像头按键扫描
void on_MouseHandle_up(int event, int x, int y, int flags, void *param);
Point draw_point_up;        // 扫描按下的键的坐标
bool mouse_signal_up=false;   // 扫描按下的键的标志

// 下摄像头按键扫描
void on_MouseHandle_down(int event, int x, int y, int flags, void *param);
Point draw_point_down;        // 扫描按下的键的坐标
bool mouse_signal_down=false;   // 扫描按下的键的标志


const std::string platform_camera_name = "Platform Camera";
int image_samp_number=61000;
int image_number_up=7;
int image_number_down=7;
int num_cam_work=2;
string lines;
ifstream instatus;
void poll_up(rs2::frameset frameset)
{
    if(frameset.size()<2)
      return ;
    Mat color_temp(Size(640, 480), CV_8UC3, (void*)frameset[1].get_data(), Mat::AUTO_STEP);				
    rs2::align align(RS2_STREAM_COLOR);    		
    
#ifdef DEBUGTIME
    clock_t time_start=clock();
#endif
    //Get processed aligned frame
    auto processed = align.process(frameset);
    rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();
#ifdef DEBUGTIME
    cout<<"up "<<((double)(clock()- time_start) / CLOCKS_PER_SEC)<<endl;
#endif
//     rs2::depth_frame aligned_depth_frame=frameset[0];
    
    

    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;
    
    points = pc.calculate(aligned_depth_frame);
    pc.map_to(frameset[1]);
    const rs2::vertex* pc_pointer=points.get_vertices();		    		
    
    Mat xImage(Size(640, 480), CV_16UC1);	
    Mat depImage(Size(640, 480), CV_16UC1);
    for(int row=0;row<480;row++)
    {
	unsigned short * data_x=xImage.ptr<unsigned short>(row);
	unsigned short * data_dep=depImage.ptr<unsigned short>(row);
	for(int col=0;col<640;col++)
	{
	    float x=((pc_pointer+row*640+col)->x)*1000;
	    float y=((pc_pointer+row*640+col)->y)*1000;
	    float z=((pc_pointer+row*640+col)->z)*1000;
	    data_x[col]=(int)abs(x);
	    data_dep[col]=(int)abs(z);
	} 
    }	
		

		
		
#ifdef RECOGNITION_MODE
	
	// 	保存图片到color和depth路径下
		string img_num_str=Int_to_String(image_number_up);
		
		cv::imwrite(UP_COLOR_PATH+img_num_str+".jpg",color_temp);
		cv::imwrite(UP_DEPTH_PATH+img_num_str+".png",depImage);
		cv::imwrite(UP_XDATA_PATH+img_num_str+".png",xImage);
		
	//      路径下只保存10张图片	
		vector<string> files=getFile(UP_COLOR_PATH);
		if(files.size()>10)
		{
		    Delete_file(UP_COLOR_PATH,".jpg",image_number_up-10);	    
		    Delete_file(UP_DEPTH_PATH,".png",image_number_up-10);
		    Delete_file(UP_XDATA_PATH,".png",image_number_up-10);
		    
		}
// 		Delete_png_dep(UP_DEPTH_PATH);
// 		Delete_png_dep(UP_XDATA_PATH);
		image_number_up++;
#endif	

	
#ifdef SAMPLING_MODE
		if(mouse_signal_up)
		{
		    string img_num_str=Int_to_String(image_samp_number);
		    cout<<image_samp_number<<endl;
		    cv::imwrite(SAMPLE_SAVE_IMAGE_PATH+img_num_str+".jpg",color_temp);
		    image_samp_number++;
		}
		imshow(COLOR_UP_SHOW,color_temp);
		depImage=depImage.mul(30);
		imshow(DEPTH_UP_SHOW,depImage);	
#endif	

		if(mouse_signal_up)
		{
		    mouse_signal_up=false;	
		    int xx=draw_point_up.x*3;
		    cout<<"y "<<draw_point_up.y<<" x "<<draw_point_up.x;
		    cout<<"  b "<<(int)(color_temp.ptr<unsigned char>(draw_point_up.y)[xx])<<"  g "<<int(color_temp.ptr<unsigned char>(draw_point_up.y)[xx+1])<<"  r "<<int(color_temp.ptr<unsigned char>(draw_point_up.y)[xx+2]);		    
		    cout<<"  x: "<<(pc_pointer+draw_point_up.y*640+draw_point_up.x)->x<<"  z: "<<(pc_pointer+draw_point_up.y*640+draw_point_up.x)->z;	    
		    cout<<"  x: "<<xImage.ptr<ushort>(draw_point_up.y)[draw_point_up.x];
		    cout<<" dis: "<<depImage.ptr<unsigned short>(draw_point_up.y)[draw_point_up.x]<<endl;
		}
#ifdef SHOW_IMAGE_MODE	
		imshow(COLOR_UP_SHOW,color_temp);
		depImage=depImage.mul(30);
		imshow(DEPTH_UP_SHOW,depImage);	
#endif

}

void poll_down(rs2::frameset frameset)
{
    if(frameset.size()<2)
      return ;
    Mat color_temp(Size(640, 480), CV_8UC3, (void*)frameset[1].get_data(), Mat::AUTO_STEP);				
    rs2::align align(RS2_STREAM_COLOR);    
#ifdef DEBUGTIME
    clock_t time_start=clock();
#endif
    //Get processed aligned frame
    auto processed = align.process(frameset);
    rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();
#ifdef DEBUGTIME
    cout<<"down "<<((double)(clock()- time_start) / CLOCKS_PER_SEC)<<endl;
#endif

    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;
    
    points = pc.calculate(aligned_depth_frame);
    pc.map_to(frameset[1]);
    const rs2::vertex* pc_pointer=points.get_vertices();		    		
    
    Mat xImage(Size(640, 480), CV_16UC1);	
    Mat depImage(Size(640, 480), CV_16UC1);
    for(int row=0;row<480;row++)
    {
	unsigned short * data_x=xImage.ptr<unsigned short>(row);
	unsigned short * data_dep=depImage.ptr<unsigned short>(row);
	for(int col=0;col<640;col++)
	{
	    float x=((pc_pointer+row*640+col)->x)*1000;
	    float y=((pc_pointer+row*640+col)->y)*1000;
	    float z=((pc_pointer+row*640+col)->z)*1000;
	    data_x[col]=(int)abs(x);
	    data_dep[col]=(int)abs(z);
	} 
    }	
    

    
    
#ifdef RECOGNITION_MODE

// 	保存图片到color和depth路径下
    string img_num_str=Int_to_String(image_number_down);
    
    cv::imwrite(DOWN_COLOR_PATH+img_num_str+".jpg",color_temp);
    cv::imwrite(DOWN_DEPTH_PATH+img_num_str+".png",depImage);
    cv::imwrite(DOWN_XDATA_PATH+img_num_str+".png",xImage);
    
//      路径下只保存10张图片	
    vector<string> files=getFile(DOWN_COLOR_PATH);
    if(files.size()>10)
    {
	Delete_file(DOWN_COLOR_PATH,".jpg",image_number_down-10);
	Delete_file(DOWN_DEPTH_PATH,".png",image_number_down-10);
	Delete_file(DOWN_XDATA_PATH,".png",image_number_down-10);
	
    }
//     Delete_png_dep(DOWN_DEPTH_PATH);
//     Delete_png_dep(DOWN_XDATA_PATH);
    image_number_down++;
#endif	


#ifdef SAMPLING_MODE
    
    if(mouse_signal_down)
    {
	string img_num_str=Int_to_String(image_samp_number);
	cout<<image_samp_number<<endl;
	cv::imwrite(SAMPLE_SAVE_IMAGE_PATH+img_num_str+".jpg",color_temp);
	image_samp_number++;
    }
		
    imshow(COLOR_DOWN_SHOW,color_temp);
    depImage=depImage.mul(30);
    imshow(DEPTH_DOWN_SHOW,depImage);	
#endif	

    if(mouse_signal_down)
    {
	mouse_signal_down=false;	
	int xx=draw_point_down.x*3;
	cout<<"y "<<draw_point_down.y<<" x "<<draw_point_down.x;
	cout<<"  b "<<(int)(color_temp.ptr<unsigned char>(draw_point_down.y)[xx])<<"  g "<<int(color_temp.ptr<unsigned char>(draw_point_down.y)[xx+1])<<"  r "<<int(color_temp.ptr<unsigned char>(draw_point_down.y)[xx+2]);		    
	cout<<"  x: "<<(pc_pointer+draw_point_down.y*640+draw_point_down.x)->x<<"  z: "<<(pc_pointer+draw_point_down.y*640+draw_point_down.x)->z;	    
	cout<<"  x: "<<xImage.ptr<ushort>(draw_point_down.y)[draw_point_down.x];
	cout<<" dis: "<<depImage.ptr<unsigned short>(draw_point_down.y)[draw_point_down.x]<<endl;
    }
#ifdef SHOW_IMAGE_MODE	
    imshow(COLOR_DOWN_SHOW,color_temp);
    depImage=depImage.mul(30);
    imshow(DEPTH_DOWN_SHOW,depImage);	
#endif

}

int rgb_down=0;
int rgb_up=0;
void poll_up_rgb(rs2::frameset frameset)
{
    if(frameset.size()<2)
      return ;
    Mat color_temp(Size(640, 480), CV_8UC3, (void*)frameset[1].get_data(), Mat::AUTO_STEP);				
    
#ifdef DEBUGTIME
    clock_t time_start=clock();
#endif
    
#ifdef DEBUGTIME
    cout<<"up "<<((double)(clock()- time_start) / CLOCKS_PER_SEC)<<endl;
#endif
	
		
#ifdef RECOGNITION_MODE
	
	// 	保存图片到color和depth路径下
		string img_num_str=Int_to_String(rgb_up);
		
		cv::imwrite(UP_RGB+img_num_str+".jpg",color_temp);

		
	//      路径下只保存10张图片	
		vector<string> files=getFile(UP_RGB);
		if(files.size()>10)
		{
		    Delete_file(UP_RGB,".jpg",rgb_up-10);
		}
		rgb_up++;
#endif	


#ifdef SHOW_IMAGE_MODE	
		imshow(COLOR_UP_SHOW,color_temp);
		if(mouse_signal_up)
		{
		    mouse_signal_up=false;	
		    int xx=draw_point_up.x*3;
		    cout<<"up y "<<draw_point_up.y<<" x "<<draw_point_up.x<<endl;
		}
		
#endif

}

void poll_down_rgb(rs2::frameset frameset)
{
    if(frameset.size()<2)
      return ;
    Mat color_temp(Size(640, 480), CV_8UC3, (void*)frameset[1].get_data(), Mat::AUTO_STEP);				
#ifdef DEBUGTIME
    clock_t time_start=clock();
#endif

#ifdef DEBUGTIME
    cout<<"down "<<((double)(clock()- time_start) / CLOCKS_PER_SEC)<<endl;
#endif


#ifdef RECOGNITION_MODE

// 	保存图片到color和depth路径下
    string img_num_str=Int_to_String(rgb_down);
    
    cv::imwrite(DOWN_RGB+img_num_str+".jpg",color_temp);
    
//      路径下只保存10张图片	
    vector<string> files=getFile(DOWN_RGB);
    if(files.size()>10)
    {
	Delete_file(DOWN_RGB,".jpg",rgb_down-10);
    }
    rgb_down++;
#endif	


#ifdef SHOW_IMAGE_MODE	
    imshow(COLOR_DOWN_SHOW,color_temp);	
    
    if(mouse_signal_down)
    {
	mouse_signal_down=false;	
	int xx=draw_point_down.x*3;
	cout<<"down y "<<draw_point_down.y<<" x "<<draw_point_down.x<<endl;
    }
#endif

}



void updata_picture(rs2::device dev){
//     仅仅更新图片
    
}

 // Helper struct per pipeline
  // Helper struct per pipeline
struct view_port
{
    std::map<int, rs2::frame> frames_per_stream;
    rs2::colorizer colorize_frame;
    rs2::pipeline pipe;
    rs2::pipeline_profile profile;
};

class device_container
{
public:

    void enable_device(rs2::device dev)
    {
        std::string serial_number(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
	cout<<serial_number<<endl;
        std::lock_guard<std::mutex> lock(_mutex);

        if (_devices.find(serial_number) != _devices.end())
        {
            return; //already in
        }

        // Ignoring platform cameras (webcams, etc..)
        if (platform_camera_name == dev.get_info(RS2_CAMERA_INFO_NAME))
        {
            return;
        }
        // Create a pipeline from the given device
        rs2::pipeline p;
	
        rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
	cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        cfg.enable_device(serial_number);
	
        // Start the pipeline with the configuration
        rs2::pipeline_profile profile = p.start(cfg);
        // Hold it internally
        _devices.emplace(serial_number, view_port{ {},{}, p, profile });

    }

    void remove_devices(const rs2::event_information& info)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        // Go over the list of devices and check if it was disconnected
        auto itr = _devices.begin();
        while(itr != _devices.end())
        {
            if (info.was_removed(itr->second.profile.get_device()))
            {
                itr = _devices.erase(itr);
            }
            else
            {
                ++itr;
            }
        }
    }

    size_t device_count()
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return _devices.size();
    }

    int stream_count()
    {
        std::lock_guard<std::mutex> lock(_mutex);
        int count = 0;
        for (auto&& sn_to_dev : _devices)
        {
            for (auto&& stream : sn_to_dev.second.frames_per_stream)
            {
                if (stream.second)
                {
                    count++;
                }
            }
        }
        return count;
    }

    void poll_frames()
    {
	string lines_camdep;
	ifstream instatus_camdep;
	instatus_camdep.open("camdep.txt");
	getline(instatus_camdep,lines_camdep);
	instatus_camdep.close();
	
	
      //         std::lock_guard<std::mutex> lock(_mutex);
	
	
	if(lines[0]=='1')
	{
	    view_port upup=_devices[UP_SERIAL_NUMBER];
	    rs2::frameset frameset;
// 	    upup.pipe.try_wait_for_frames(&frameset);
	    upup.pipe.poll_for_frames(&frameset);
	    thread uprun(poll_up,frameset);
	    uprun.join();
// 	    thread uprun(poll_up_rgb,frameset);
// 	    uprun.join();
	}
	else{
	    if(lines_camdep[0]=='6'){
		  view_port upup=_devices[UP_SERIAL_NUMBER];
		  rs2::frameset frameset;
		  upup.pipe.poll_for_frames(&frameset);  
		  view_port downdown=_devices[DOWN_SERIAL_NUMBER];
		  rs2::frameset framesetdw;
		  downdown.pipe.poll_for_frames(&framesetdw);
		while(frameset.size()<2){
		      upup.pipe.poll_for_frames(&frameset); 
		}  
		while(framesetdw.size()<2){
		      downdown.pipe.poll_for_frames(&framesetdw);
		}
		
		thread downrun(poll_down,framesetdw);
		thread uprun(poll_up,frameset);
		downrun.join();
		uprun.join();	

		ofstream instatus;
		instatus.open("camdep.txt");		
// 		string img_str=Int_to_String(image_number_up-1);		
		instatus<<'5'<<endl;
		instatus.close();
		cout<<"rgbd ok"<<endl;
	    }
	    else{
		view_port upup=_devices[UP_SERIAL_NUMBER];
		rs2::frameset frameset;
		upup.pipe.poll_for_frames(&frameset);  
		// 	  upup.pipe.try_wait_for_frames(&frameset);
		view_port downdown=_devices[DOWN_SERIAL_NUMBER];
		rs2::frameset framesetdw;
		downdown.pipe.poll_for_frames(&framesetdw);
		// 	  downdown.pipe.try_wait_for_frames(&framesetdw);
		thread downrun(poll_down_rgb,framesetdw);
		thread uprun(poll_up_rgb,frameset);
		downrun.join();
		uprun.join();	

	    }
	}
	
    }
    
public:
    std::mutex _mutex;
    std::map<std::string, view_port> _devices;
};


int main(int argc, char **argv) {
// 	string lines;
// 	ofstream instatus;
// 	instatus.open("status.txt");
// 	instatus<<"2"<<endl;
// 	instatus.close();
    
    

    
    
    
    instatus.open("status.txt");
    getline(instatus,lines);
    instatus.close();
#ifdef RECOGNITION_MODE
    // 清空路径下的文件，重新存放图片
    Clear_file(UP_COLOR_PATH);
    Clear_file(UP_DEPTH_PATH);
    Clear_file(UP_XDATA_PATH); 
    Clear_file(DOWN_COLOR_PATH);
    Clear_file(DOWN_DEPTH_PATH);
    Clear_file(DOWN_XDATA_PATH); 
    Clear_file(UP_RGB); 
    Clear_file(DOWN_RGB); 
#endif
    
    namedWindow(COLOR_UP_SHOW, WINDOW_AUTOSIZE);
    namedWindow(COLOR_DOWN_SHOW, WINDOW_AUTOSIZE);
    cv::Mat color_image_up=cv::Mat(480,640,CV_16UC1);
    cv::Mat color_image_down=cv::Mat(480,640,CV_16UC1);
    setMouseCallback(COLOR_UP_SHOW,on_MouseHandle_up,(void*)&color_image_up);
    setMouseCallback(COLOR_DOWN_SHOW,on_MouseHandle_down,(void*)&color_image_down);
    
    
    device_container connected_devices;

    rs2::context ctx;    // Create librealsense context for managing devices

                         // Register callback for tracking which devices are currently connected
    ctx.set_devices_changed_callback([&](rs2::event_information& info)
    {
        connected_devices.remove_devices(info);
        for (auto&& dev : info.get_new_devices())
        {
            connected_devices.enable_device(dev);
        }
    });

    // Initial population of the device list
    for (auto&& dev : ctx.query_devices()) // Query the list of connected RealSense devices
    {
        connected_devices.enable_device(dev);
    }
    cout<<"init successs"<<endl;

    while (1) // Application still alive?
    {
#ifdef DEBUGTIME
	clock_t time_start=clock();
#endif
        connected_devices.poll_frames();
	int a=waitKey(1);
// 	int a=waitKey(1);
	if(a==27)
	  break;
#ifdef DEBUGTIME
	cout<<((double)(clock()- time_start) / CLOCKS_PER_SEC)<<endl;
#endif

    }
//  
    return 0;
}

// 鼠标回调函数
void on_MouseHandle_down(int event, int x, int y, int flags, void *param){
	switch( event)
	{
	    //鼠标移动消息
	    case EVENT_LBUTTONDOWN: 
	    {
		mouse_signal_down=true;
		draw_point_down.x=x;
		draw_point_down.y=y;
	    }
		break;
	    default :
		break;
	}
}
// 鼠标回调函数
void on_MouseHandle_up(int event, int x, int y, int flags, void *param){
	switch( event)
	{
	    //鼠标移动消息
	    case EVENT_LBUTTONDOWN: 
	    {
		mouse_signal_up=true;
		draw_point_up.x=x;
		draw_point_up.y=y;
	    }
		break;
	    default :
		break;
	}
}
