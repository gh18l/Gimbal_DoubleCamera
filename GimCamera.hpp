#ifndef __GIMCAMERA_HPP__
#define __GIMCAMERA_HPP__
#include "GenCameraDriver.h"
#include <opencv2/opencv.hpp>
class GimCamera
{
public:
	GimCamera();
	~GimCamera();
	std::vector<cam::GenCamInfo> camInfos;
	std::shared_ptr<cam::GenCamera> cameraPtr;
	void init();
	void shoot(cv::Mat &ref, cv::Mat &local);
	void camera_close();
	int colorCorrectRGB(cv::Mat & srcImg, cv::Mat dstImg);
};


#endif
