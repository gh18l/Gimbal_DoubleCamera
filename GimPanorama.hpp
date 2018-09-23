#ifndef __GIMPANORAMA_HPP__
#define __GIMPANORAMA_HPP__
#include <opencv2/opencv.hpp>
#include "GimUtil.h"
#include "FeatureMatch.h"
#include "CameraParamEstimator.h"
#include "Compositor.h"
#include "opencv2/video/background_segm.hpp"
class GimPano
{
public:
	GimPano(std::shared_ptr<GimUtil> GimUtilPtr,
		std::shared_ptr<GimCamera> GimCameraPtr);
	~GimPano();

	std::shared_ptr<calib::FeatureMatch> match_ptr;   //不知道有没有必要定义成指针
	std::shared_ptr<GimUtil> gimutil_ptr;
	std::shared_ptr<GimCamera> gimcamera_ptr;

	int dX, X_MIN, dY, Y_MIN;
	int Row;
	int Col;

	cv::Point corner_current;
	cv::Size size_current;
	std::vector<cv::Point> corners;
	std::vector<cv::Size> sizes;
	std::vector<int> index;
	std::vector<cv::Mat> imgs;   //采集的图像
	cv::Mat panorama;   //拼出来的全景图
	cv::Mat current_panorama;   //拼接后的全景图
	std::vector<calib::CameraParams> cameras;
	calib::CameraParams current_para;
	std::vector<calib::Imagefeature> features;
	cv::Ptr<cv::BackgroundSubtractor> bg_model;;
	cv::Mat pulse_pixel_param;

	void init();
	bool isOverlap(const cv::Rect &rc1, const cv::Rect &rc2);
	int findoverlap();
	int save_para(std::vector<calib::CameraParams>& cameras,
		std::vector<cv::Point>& corners, std::vector<cv::Size>& sizes);
	int read_para();
	int GetCurrentPara(std::vector<calib::CameraParams>& cameras,
		cv::Point2f current_point, calib::CameraParams &current_para);
	void get_panorama();
	void collection();
	int warp(cv::Mat src);
	int read_features();
	void read_panorama();
	void save_point_connection_pulse_para(std::vector<std::pair<cv::Point, cv::Point>> point_and_pulse);
};

#endif
