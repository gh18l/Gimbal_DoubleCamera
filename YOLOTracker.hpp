#ifndef __YOLO_TRACKER_HPP__
#define __YOLO_TRACKER_HPP__
#define YOLO_TRACKER_EXPORTS     ////////////////////////////////
#ifdef YOLO_TRACKER_EXPORTS
#if defined(_MSC_VER)
#define YOLO_TRACKER_API __declspec(dllexport) 
#else
#define YOLO_TRAKCER_API __attribute__((visibility("default")))
#endif
#else
#if defined(_MSC_VER)
#define YOLO_TRACKER_API __declspec(dllimport) 
#else
#define YOLOTRACKERDLL_API
#endif
#endif

#include <memory>
#include <opencv2/opencv.hpp>
#include <map>
class YOLOTracker 
{
private:
	std::shared_ptr<void> YOLOTracker_cuda;


public:
	YOLO_TRACKER_API YOLOTracker();
	YOLO_TRACKER_API ~YOLOTracker();
	
	std::map<int, cv::Rect> tracker;

	YOLO_TRACKER_API int init(cv::Mat frame, std::string cfgfile, std::string weightfile, std::string objnamefile);
	YOLO_TRACKER_API int init(cv::Mat frame, std::string cfgfile, std::string weightfile, std::string objnamefile, std::string face_cfgfile, std::string face_weightfile);
	YOLO_TRACKER_API int detector_people(cv::Mat frame, int currFrame);
	YOLO_TRACKER_API int tracker_people(int currFrame);
	YOLO_TRACKER_API std::vector<cv::Rect> single_detect_people(cv::Mat frame, cv::Rect crop);
	YOLO_TRACKER_API std::vector<cv::Rect> single_detect_face(cv::Mat frame, cv::Rect crop);
	YOLO_TRACKER_API std::map<int, cv::Rect> get_tracker_map();
	YOLO_TRACKER_API void update_tracker_map(std::map<int, cv::Rect> temp);
};


#endif
