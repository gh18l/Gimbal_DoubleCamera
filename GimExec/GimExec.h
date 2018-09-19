#ifndef __GIMEXEC_HPP__
#define __GIMEXEC_HPP__

#include "YOLOTracker.hpp"
#include <opencv2/opencv.hpp>
#include <memory>
#include "GimCamera.hpp"
#include "GimUtil.h"
#include "Display.h"
#include "GimPanorama.hpp"

class GimExec
{
public:
	GimExec(std::shared_ptr<GimCamera> GimCameraPtr, std::shared_ptr<Display> DisplayPtr, std::shared_ptr<GimPano> GimPanoPtr,
		std::shared_ptr<YOLOTracker> YOLOTrackerPtr, std::shared_ptr<GimUtil> GimUtilPtr);
	~GimExec();
	void track_init(cv::Mat frame);
	void track_people(cv::Mat ref);
	std::vector<cv::Rect> single_detection_people(cv::Mat frame, cv::Rect crop);
	std::vector<cv::Rect> single_detection_face(cv::Mat frame, cv::Rect crop);
	cv::Rect netroi2refroi(cv::Rect netroi);
	cv::Rect netroi2roi(cv::Rect netroi);
	bool get_dst();
	bool detect_face(cv::Point current_point);
	void draw_tracking(cv::Mat ref);
	void Thtracking();
	void Thshoot();
	void Thshowref();
	void Thshowpanorama();
	void Thstitch();
	void Thstart();
	void test_Thstart();

	std::shared_ptr<GimCamera> gimcameraptr;
	std::shared_ptr<Display> displayptr;
	std::shared_ptr<GimPano> gimpanoptr;
	std::shared_ptr<YOLOTracker> yolotrackerptr;
	std::shared_ptr<GimUtil> gimutilptr;

	cv::Mat current_ref;
	cv::Mat current_local;
	cv::Mat current_local_stitch;
	cv::Mat current_ref_draw;
	////////////////////////////////
	cv::Mat localdraw;
	cv::Mat find_before_after;

	std::map<int, cv::Rect> current_tracker;
	cv::Rect ref_roi;
	std::vector<cv::Mat> NeedToShow;
	int NeedToShow_index;
	std::vector<int> tracked_id;    //已经跟踪过的行人id，在提取到高清人脸后应该放入
	std::pair<int, cv::Rect> dst_tracker;
	cv::Point dst_point;
	bool IsTracking;

	//fps
	size_t start, end;
	int flag;
};



#endif