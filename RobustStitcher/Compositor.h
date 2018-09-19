/**
@brief Compositor.h
C++ head file for compositing images into panorams
@author Shane Yuan
@date Feb 9, 2018
*/

#ifndef _ROBUST_STITCHER_COMPOSITOR_H_
#define _ROBUST_STITCHER_COMPOSITOR_H_ 

// std
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <thread>

// opencv
#include <opencv2/opencv.hpp>

#include "CameraParamEstimator.h"
#include "Warper.h"

namespace calib {
	class Compositor {
	private:
		std::vector<cv::Rect> roiRect;
		std::vector<cv::Mat> imgWarped;
		std::vector<cv::Mat> imgMask;
		std::vector<cv::Mat> warpMaps;
		std::vector<cv::Mat> backwarpMaps;
	public:
		std::vector<cv::Mat> imgs;
		std::vector<calib::CameraParams> cameras;

		std::vector<cv::Point> corners;
		std::vector<cv::Size> sizes;
	private:

	public:
		Compositor();
		~Compositor();

		/**
		@brief init function set image and cameras
		@param std::vector<cv::Mat> imgs: input images
		@param std::vector<calib::CameraParams> cameras: input camera params
		@return int
		*/
		int init(std::vector<cv::Mat> imgs, std::vector<calib::CameraParams> cameras);

		/**
		@brief composite panorama
		@return int
		*/

		int single_composite(calib::CameraParams& current_camera, cv::Mat& src, cv::Mat& result, std::vector<cv::Point> &corners, 
						std::vector<cv::Size>&sizes);

		int composite(cv::Mat& panorama);
	};

};

#endif