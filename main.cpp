#include <iostream>
#include <fstream>
#include <thread>
#include <memory>
#include <string.h> 
#ifdef _WIN32
#include <windows.h>
#include <tchar.h>
#endif
#include "Serial.h"  
#include "Display.h"
#include <opencv2/opencv.hpp>
#include "GenCameraDriver.h"
#include "GimCamera.hpp"
#include "GimUtil.h"
#include "GimPanorama.hpp"
#include "GimExec.h"
#include "YOLOTracker.hpp"

#include <nanogui/nanogui.h>

std::shared_ptr<GimCamera> GimCameraPtr;
std::shared_ptr<CSerial> SerialPtr;
std::shared_ptr<GimUtil> GimUtilPtr;
std::shared_ptr<GimPano> GimPanoPtr;
std::shared_ptr<Display> DisplayPtr;
std::shared_ptr<GimExec> GimExecPtr;
std::shared_ptr<YOLOTracker> YOLOTrackerPtr;
calib::FeatureMatch match;

std::string GimUtil::datapath = "E:/data/system_data";
int GimUtil::dX = 1500;
int GimUtil::X_MIN = 0;
int GimUtil::dY = 1500;
int GimUtil::Y_MIN = 0;
int GimUtil::Row = 6;
int GimUtil::Col = 8;
float GimUtil::scale = 0.215;

void init_base()
{
	GimCameraPtr = std::make_shared<GimCamera>();
	SerialPtr = std::make_shared<CSerial>();
	GimUtilPtr = std::make_shared<GimUtil>(SerialPtr, GimCameraPtr);
	GimPanoPtr = std::make_shared<GimPano>(GimUtilPtr, GimCameraPtr);
	DisplayPtr = std::make_shared<Display>();
	YOLOTrackerPtr = std::make_shared<YOLOTracker>();
	GimExecPtr = std::make_shared<GimExec>(GimCameraPtr, DisplayPtr, GimPanoPtr,
		YOLOTrackerPtr, GimUtilPtr);
	GimCameraPtr->init();
	GimPanoPtr->init();
	//init serial
#ifdef _WIN32
	SerialPtr->OpenSerialPort(_T("COM5:"), 9600, 8, 1);
#else
	serial.Serial_Init();
#endif
	cv::Mat ref, local;
	GimUtil::sleep(1000);
}

void init_ext(int argc)
{
	//init gimble
	cv::Mat ref, local;
	//GimUtilPtr->gimbal_init();   //转到初始位置
	//update now current_point and current_pulse
	GimCameraPtr->shoot(ref, local);
	GimUtilPtr->video_width = local.cols;
	GimUtilPtr->video_height = local.rows;
	GimUtilPtr->find_position(ref, local, GimUtilPtr->current_point);    //update now point
	std::cout << "init point is      " << GimUtilPtr->current_point << std::endl;
	if (argc > 1)
	{
		//extract panorama and imgs
		GimPanoPtr->read_panorama();
		GimUtilPtr->current_pulse = GimUtilPtr->read_pulse();
	}
	else
	{
		//generate panorama and save it, generate imgs and save it, save parameters and features
		GimUtilPtr->current_pulse.x = (GimUtilPtr->current_point.x - 20) * GimUtilPtr->pulse_per_pixel;
		GimUtilPtr->current_pulse.y = (GimUtilPtr->current_point.y - 15) * GimUtilPtr->pulse_per_pixel;    //得到这个以便生成全景图时
		GimPanoPtr->get_panorama();
	}
	//extract cameras, corners, sizes
	GimPanoPtr->read_para();
	//extract features
	GimPanoPtr->read_features();
	GimExecPtr->ref_roi = cv::Rect(ref.cols / 4, ref.rows / 4, ref.cols / 2, ref.rows / 2);
	GimExecPtr->track_init(ref(GimExecPtr->ref_roi));
	//init picture parameters
}

void test_ref2local()
{
	cv::Point point;
	cv::Mat ref, local;
	GimUtilPtr->current_pulse = cv::Point(0, 0);
	for (int i = 0; i < 10; i++)
	{
		for (int j = 0; j < 10; j++)
		{
			GimUtilPtr->send_pulse(cv::Point(i * 500, j * 500));
			GimUtil::move_delay();
			GimCameraPtr->shoot(ref, local);
			GimUtilPtr->find_position_onlycolor(ref, local, point);
			std::cout << "current pulse index is " << i * 500 << "   " << j * 500 << " current point is " << point << std::endl;
			cv::Rect roi(point, cv::Size(local.cols * GimUtil::scale, local.rows * GimUtil::scale));
			cv::rectangle(ref, roi, cv::Scalar(0, 0, 255));
			cv::imwrite(cv::format("E:/data/test_data/%d_%d_%d_%d_local.jpg", i * 500, j * 500, point.x, point.y), local);
			cv::imwrite(cv::format("E:/data/test_data/%d_%d_%d_%d_ref.jpg", i * 500, j * 500, point.x, point.y), ref);
			cv::imwrite(cv::format("E:/data/test_data/%d_%d_%d_%d_refblock.jpg", i * 500, j * 500, point.x, point.y), ref(roi));
		}
	}
}


void test_tracking()
{
	cv::VideoCapture cap("E:/datasets/data/2/video_03_scale.avi");
	cv::Mat frame;
	cap >> frame;
	GimUtilPtr = std::make_shared<GimUtil>(SerialPtr, GimCameraPtr);
	GimPanoPtr = std::make_shared<GimPano>(GimUtilPtr, GimCameraPtr);
	YOLOTrackerPtr = std::make_shared<YOLOTracker>();
	DisplayPtr = std::make_shared<Display>();
	GimExecPtr = std::make_shared<GimExec>(GimCameraPtr, DisplayPtr, GimPanoPtr,
		YOLOTrackerPtr, GimUtilPtr);
	cv::resize(frame, frame, cv::Size(2064, 1544));
	GimUtilPtr->video_width = frame.cols;
	GimUtilPtr->video_height = frame.rows;
	cv::Mat img = cv::imread("E:/datasets/result.jpg");
	cv::resize(img, img, cv::Size(2064, 1544));
	img.copyTo(GimPanoPtr->current_panorama);
	GimExecPtr->ref_roi = cv::Rect(frame.cols / 4, frame.rows / 4 , frame.cols / 2 , frame.rows / 2);
	//GimExecPtr->ref_roi = cv::Rect(0, 0, frame.cols, frame.rows);
	GimExecPtr->track_init(frame(GimExecPtr->ref_roi));
	GimExecPtr->test_Thstart();
	while (1)
	{
		cap >> frame;
		cv::resize(frame, frame, cv::Size(2064, 1544));
		frame.copyTo(GimExecPtr->current_ref);
		GimUtil::sleep(50);
	}
}

void test_serial()
{
	cv::Mat ref, local;
	for (int i = 0; i < 1; i++)
	{
		GimUtilPtr->send_pulse(cv::Point(1000, 1000), std::pair<bool, bool>(0, 1));
		GimUtil::move_delay();
		GimCameraPtr->shoot(ref, local);
		cv::imwrite(cv::format("E:/data/test_data/%d.jpg", i), local);
		cv::imwrite(cv::format("E:/data/test_data/%d_ref.jpg", i), ref);
	}
	//for (int i = 0; i < 10; i++)
	//{
	//	GimUtilPtr->send_pulse(cv::Point(1000, 1000), std::pair<bool, bool>(0, 1));
	//	GimUtil::sleep(600);
	//	GimCameraPtr->shoot(ref, local);
	//	cv::imwrite(cv::format("E:/data/test_data/%d_duiying.jpg", 8 - i), local);
	//}
	
}

void test_face()
{
	cv::Mat face = cv::imread("E:/data/test_data/HRpeople/0.jpg");
	cv::Rect roi(0, 0, face.cols, face.rows / 2);
	std::vector<cv::Rect> faceresult;
	faceresult = YOLOTrackerPtr->single_detect_face(face, roi);
	for(int i = 0; i < faceresult.size(); i++)
		std::cout << faceresult[i] << std::endl;
	system("pause");
}


void Execute()
{
	GimExecPtr->Thstart();
	while (1)
	{
		if (GimExecPtr->get_dst() == 1) //get dst_point
		{
			//update current_pulse and current_point
			if (GimUtilPtr->move(GimExecPtr->dst_point) == -1)   //不符合move要求
			{
				//std::cout << "move error!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
				continue;
			}
			GimUtil::move_delay();
			//以后可去掉
			cv::Mat find;
			GimExecPtr->current_ref.copyTo(find);
			cv::Rect find_before = cv::Rect(GimExecPtr->dst_point,
				cv::Size(GimUtilPtr->video_width * GimUtil::scale,
					GimUtilPtr->video_height * GimUtil::scale));
			cv::rectangle(find, find_before, cv::Scalar(255, 0, 0), 4);
			GimUtilPtr->gimble_find_position(GimExecPtr->current_ref,
				GimExecPtr->current_local, GimUtilPtr->current_point, 2,
				GimUtilPtr->current_point);
			cv::Rect find_after = cv::Rect(GimExecPtr->dst_point,
				cv::Size(GimUtilPtr->video_width * GimUtil::scale,
					GimUtilPtr->video_height * GimUtil::scale));
			cv::rectangle(find, find_after, cv::Scalar(0, 0, 255), 4);
			find.copyTo(GimExecPtr->find_before_after);
			//push face_img to NeedToShow
			GimExecPtr->detect_face(GimUtilPtr->current_point);
			GimUtil::sleep(2000);
		}
		else
			std::cout << "dont have tracker or all error" << std::endl;
	}
}



int main(int argc, char* argv[])
{
	
	init_base();
	init_ext(argc);
	//test_ref2local();
	//GimUtilPtr->initposition(cv::Point(0, 0), std::pair<bool, bool>(1, 0)); // 1 1左上
	//GimUtilPtr->estimate_scale();
	//test_tracking();
	//test_serial();
	//test_face();
	Execute();
}