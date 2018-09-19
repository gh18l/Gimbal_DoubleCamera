#ifndef __PROPERTY_HPP__
#define __PROPERTY_HPP__

#include <iostream>
#include <fstream>
#include <chrono>
#include <memory>
#include <thread>
#ifdef WIN32
#include <Windows.h>
#endif
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/ximgproc.hpp>
#include <opencv2/opencv.hpp>
#include "Serial.h"  
#include "GimCamera.hpp"
#define PM 6200
#define YM 3900

class GimUtil{
public:
    static int sleep(int miliseconds);
	static void move_delay();
    cv::Mat SEDDetector(cv::Mat img, float scale);
    static int colorCorrectRGB(cv::Mat & srcImg, cv::Mat dstImg);
    bool isInside(cv::Point2f pt, cv::Rect rect);
public:
	GimUtil(std::shared_ptr<CSerial> SerialPtr,
		std::shared_ptr<GimCamera> GimCameraPtr);
    ~GimUtil();

    int find_position(cv::Mat refImg, cv::Mat localImg, cv::Point &out_point);
	int find_position(cv::Mat refImg, cv::Mat localImg, cv::Point &out_point, double &max_value);
	int find_position_onlycolor(cv::Mat refImg, cv::Mat localImg, cv::Point &out_point);
	int find_position_onlycolor(cv::Mat refImg, cv::Mat localImg, cv::Point &out_point, double &max_value);
	int gimble_find_position(cv::Mat refImg, cv::Mat localImg, cv::Point ref_point, float region_mul,cv::Point &out_point);
	void initposition(cv::Point pulse, std::pair<bool, bool> dir);
	void estimate_scale();
	void write_pulse(cv::Point current_pulse);
	cv::Point read_pulse();
public:
    static int dX, X_MIN, dY, Y_MIN;
	static int Row;
	static int Col;
    cv::Point current_point;
	cv::Point current_pulse;
    cv::Size sizeBlock;
	int pulse_per_pixel;
public:
    cv::Ptr<cv::ximgproc::StructuredEdgeDetection> ptr;
    static float scale;
    int video_width;
    int video_height;
	static std::string datapath;

	std::shared_ptr<CSerial> serial_ptr;
	std::shared_ptr<GimCamera> gimcamera_ptr;
	std::shared_ptr<GimUtil> gimutilptr;
	void send_pulse(cv::Point pulse, std::pair<bool, bool> dir);
	void send_pulse(cv::Point dst_pulse);   //send absolute pulse

	int move(cv::Point dst_point);
	cv::Point dst2pulse(cv::Point dst);
	cv::Point init;
};
#endif
