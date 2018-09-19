#include "GimExec.h"

#define DEBUG

GimExec::GimExec(std::shared_ptr<GimCamera> GimCameraPtr, std::shared_ptr<Display> DisplayPtr, std::shared_ptr<GimPano> GimPanoPtr,
	                std::shared_ptr<YOLOTracker> YOLOTrackerPtr, std::shared_ptr<GimUtil> GimUtilPtr) : 
	NeedToShow_index(0), flag(0), IsTracking(0)
{
	gimcameraptr = GimCameraPtr;
	displayptr = DisplayPtr;
	gimpanoptr = GimPanoPtr;
	yolotrackerptr = YOLOTrackerPtr;
	gimutilptr = GimUtilPtr;
}
GimExec::~GimExec() {}

void GimExec::track_init(cv::Mat frame)
{
	std::string cfgfile = "E:/data/YOLO/yolov3.cfg";
	std::string weightfile = "E:/data/YOLO/yolov3.weights";
	std::string objnamefile = "E:/data/YOLO/coco.names";
	std::string face_cfgfile = "E:/data/YOLO/yolo-face.cfg";
	std::string face_weightfile = "E:/data/YOLO/yolo-face.weights";

	yolotrackerptr->init(frame, cfgfile, weightfile, objnamefile, face_cfgfile, face_weightfile);
}

void GimExec::track_people(cv::Mat ref)
{
	cv::Mat ref_temp;
	ref(ref_roi).copyTo(ref_temp);
	cv::resize(ref_temp, ref_temp, cv::Size(416, 416));
	if (IsTracking == 0)
	{
		IsTracking = 1;
		yolotrackerptr->detector_people(ref_temp, 0);
		IsTracking = 0;
	}
	else
	{
		while (IsTracking != 0)
		{
			std::cout << "track people is locked " << IsTracking << std::endl;
		}
		
		IsTracking = 1;
		yolotrackerptr->detector_people(ref_temp, 0);
		IsTracking = 0;
	}
	yolotrackerptr->tracker_people(0);  //get data into trakcer

	//convert netroi to refroi
	std::map<int, cv::Rect> temp;
	std::map<int, cv::Rect>::iterator it;
	temp = yolotrackerptr->get_tracker_map();

	for (it = temp.begin(); it != temp.end(); it++)
	{
		std::cout << "first              " << it->second << std::endl;
		if (it->second.x > 2000 || it->second.y > 1500)
		{
			std::cout << "first error !!!!!!!!!!!!!!!!!" << std::endl;
		}
	}

	for (it = temp.begin(); it != temp.end(); it++)
	{
		cv::Rect region(it->second.x, it->second.y, it->second.width, it->second.height);
		cv::Rect roi = netroi2refroi(region);
		temp[it->first] = roi;
	}

	for (it = temp.begin(); it != temp.end(); it++)
	{
		if (it->second.x > 2000 || it->second.y > 1500)
		{
			std::cout << "second error !!!!!!!!!!!!!!!!!" << std::endl;
		}
	}

	yolotrackerptr->update_tracker_map(temp);
	current_tracker = yolotrackerptr->get_tracker_map();
}

cv::Rect GimExec::netroi2refroi(cv::Rect netroi)
{
	std::cout << "before              " << netroi << std::endl;
	cv::Rect roi = cv::Rect((float)netroi.x * (float)ref_roi.width / 416.0 + (float)ref_roi.x,
		(float)netroi.y * (float)ref_roi.height / 416.0 + (float)ref_roi.y, (float)netroi.width * (float)ref_roi.width / 416.0,
		(float)netroi.height * (float)ref_roi.height / 416.0);
	std::cout << "after              " << roi << std::endl;
	return roi;
}

void GimExec::draw_tracking(cv::Mat ref)
{
	std::map<int, cv::Rect> tracker_temp;
	tracker_temp = yolotrackerptr->get_tracker_map();
	if (tracker_temp.size() != 0)
	{
		std::cout << "draw roi size is    " << tracker_temp.size() << std::endl;
		std::map<int, cv::Rect>::iterator it;
		it = tracker_temp.begin();
		char showMsg[10];
		for (it; it != tracker_temp.end(); it++)
		{
			int ID = it->first;
			cv::Rect roi = it->second;
			std::cout << "draw roi is    " << roi << std::endl;
			cv::rectangle(ref, roi, cv::Scalar(255, 255, 0), 4);

			sprintf(showMsg, "%d", ID);
			cv::putText(ref, showMsg, cv::Point(roi.x, roi.y), CV_FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255, 0, 0), 4);
		}
#ifdef DEBUG
		cv::rectangle(ref, ref_roi, cv::Scalar(255, 0, 0), 4);
#endif
	}
	
}

std::vector<cv::Rect> GimExec::single_detection_people(cv::Mat frame, cv::Rect crop)
{
	std::vector<cv::Rect> vector;
	vector = yolotrackerptr->single_detect_people(frame, crop);
	return vector;
}

std::vector<cv::Rect> GimExec::single_detection_face(cv::Mat frame, cv::Rect crop)
{
	std::vector<cv::Rect> vector;
	vector = yolotrackerptr->single_detect_face(frame, crop);
	return vector;
}

//get dst_point
bool GimExec::get_dst()
{
	std::map<int, cv::Rect> tracker_temp = current_tracker;
	if (tracker_temp.size() == 0)
	{
		//std::cout << "we dont detect any person!!!" << std::endl;
		return 0;
	}
	cv::Point dst;

	std::map<int, cv::Rect>::iterator it;
	it = tracker_temp.begin();
	for(it; it != tracker_temp.end(); it++)
	{
		std::vector<int>::iterator result = std::find(tracked_id.begin(), tracked_id.end(), it->first);
		if (result == tracked_id.end() && it->second.x < ref_roi.x + ref_roi.width && it->second.x > 0 
			&& it->second.y < ref_roi.y + ref_roi.height && it->second.y > 0)   //没有跟踪过且数据是正确的
		{
			dst_tracker = std::make_pair(it->first, it->second);
			break;
		}
		else
		{
			continue;
		}
	}
	if (it == tracker_temp.end())   //没找到没跟踪过的，就随机找一个吧
	{
		//这里没做任何处理
		std::map<int, cv::Rect>::iterator it;
		it = tracker_temp.begin();
		while (it->second.x > ref_roi.x + ref_roi.width || it->second.y > ref_roi.y + ref_roi.height
			|| it->second.x < 0 || it->second.y < 0)
		{
			if (it == tracker_temp.end())
			{
				std::cout << "All tracking coordinate is error!!!!!!!!!" << std::endl;
				return 0;
			}
			it++;
		}
		dst_tracker = std::make_pair(it->first, it->second);
	}

	dst_point = cv::Point(abs(dst_tracker.second.x - gimutilptr->video_width * GimUtil::scale / 2.0),
		abs(dst_tracker.second.y - gimutilptr->video_height * GimUtil::scale / 2.0));
	return 1;
}

bool GimExec::detect_face(cv::Point current_point)
{
	cv::Mat detect_local;
	current_local.copyTo(detect_local);
	std::vector<cv::Rect> people_roi;
	if (IsTracking == 0)
	{
		IsTracking = 1;
		people_roi = yolotrackerptr->single_detect_people(detect_local, cv::Rect(0, 0, detect_local.cols,
			detect_local.rows));
		IsTracking = 0;
	}
	else
	{
		while (IsTracking != 0)
		{
			std::cout << "detect people is locked     " << IsTracking << std::endl;
		}
		IsTracking = 1;
		people_roi = yolotrackerptr->single_detect_people(detect_local, cv::Rect(0, 0, detect_local.cols,
			detect_local.rows));
		IsTracking = 0;
	}
	
	if (people_roi.size() == 0)
	{
		//std::cout << "Cant find any people in local~~~!!!" << std::endl;
		return 0;
	}

	cv::Point relative_point;
	relative_point = cv::Point(dst_tracker.second.x - current_point.x,
		dst_tracker.second.y - current_point.y);
	//这里最终生成的是整数吧
	relative_point.x = ((float)relative_point.x) / GimUtil::scale;
	relative_point.y = ((float)relative_point.y) / GimUtil::scale;
	int mindst = 9999;
	int dst;
	int index;
	for (int i = 0; i < people_roi.size(); i++)
	{
		dst = std::abs(relative_point.x - people_roi[i].x)
			+ std::abs(relative_point.y - people_roi[i].y);
		if (dst < mindst)
		{
			mindst = dst;
			index = i;
		}
	}
	if (people_roi[index].x - 50 > 0
		&& people_roi[index].x + people_roi[index].width + 50 < detect_local.cols)
	{
		people_roi[index].x -= 50;
		people_roi[index].width += 100;
	}
	if (people_roi[index].y - 50 > 0
		&& people_roi[index].y + people_roi[index].height + 50 < detect_local.rows)
	{
		people_roi[index].y -= 50;
		people_roi[index].height += 100;
	}
	cv::Rect face(people_roi[index].x, people_roi[index].y,
		people_roi[index].width, people_roi[index].height / 2);

	std::vector<cv::Rect> face_roi;
	if (IsTracking == 0)
	{
		IsTracking = 1;
		face_roi = yolotrackerptr->single_detect_face(detect_local, face);
		IsTracking = 0;
	}
	else
	{
		while (IsTracking != 0)
		{
			std::cout << "detect face is locked       " << IsTracking << std::endl;
		}
		IsTracking = 1;
		face_roi = yolotrackerptr->single_detect_face(detect_local, face);
		IsTracking = 0;
	}
	if (face_roi.size() == 0)
	{
		//std::cout << "Only have back view~~~!!!" << std::endl;
		return 0;
	}
	else
	{
		//在脸部画标号
		char showMsg[10];
		sprintf(showMsg, "%d", dst_tracker.first);
		cv::putText(detect_local(face), showMsg, cv::Point(0, 0), CV_FONT_HERSHEY_SIMPLEX, 0.8,
			cv::Scalar(255, 0, 0), 4);
		if(NeedToShow.size() < 6)   //这里的6根据display.cpp里的来
			NeedToShow.push_back(detect_local(face));
		if (NeedToShow.size() == 6)
		{
			if (NeedToShow_index > 5)
				NeedToShow_index = 0;
			detect_local(face).copyTo(NeedToShow[NeedToShow_index]);
			NeedToShow_index++;
		}
	}
	return 1;
}

void GimExec::Thshoot()
{
	while (1)
	{
		cv::Mat ref, local;
		gimcameraptr->shoot(ref, local);
		ref.copyTo(current_ref);
		local.copyTo(current_local);

		GimUtil::sleep(25);
	}
}

void GimExec::Thtracking()
{
	while (1)
	{
		if (!current_ref.empty())
		{
			cv::Mat current_ref_tracking;
			current_ref.copyTo(current_ref_tracking);
			track_people(current_ref_tracking);
			draw_tracking(current_ref_tracking);
			current_ref_tracking.copyTo(current_ref_draw);
			if (flag == 99)
			{
				start = clock();
				flag++;
			}
			else if (flag == 100)
			{
				end = clock();
				float dur = (float)(end - start);
				std::cout << "current fps is " << 1.0f / (dur / CLOCKS_PER_SEC) << std::endl;
				flag = 0;
			}
			else
			{
				flag++;
			}
		}
		
	}
}

void GimExec::Thshowref()
{
	while (1)
	{
		if (!current_ref_draw.empty())
		{
			cv::Mat show;
			current_ref_draw.copyTo(show);
			cv::resize(show, show, cv::Size(1200, 1000));
			
			cv::imshow("ref", show);
			cv::waitKey(1);
		}
	}
	
}

void GimExec::Thshowpanorama()
{
	cv::Mat black = cv::Mat::zeros(600, 600, CV_8UC3);
	std::vector<cv::Mat> blackvec;
	blackvec.push_back(black);
	displayptr->display_init(gimpanoptr->current_panorama);
	while (1)
	{
		if (!gimpanoptr->current_panorama.empty())
		{
			cv::Mat current_panorama;
			gimpanoptr->current_panorama.copyTo(current_panorama);
			cv::Size size(current_panorama.cols / 100 * 100, current_panorama.rows / 100 * 100);
			cv::resize(current_panorama, current_panorama, size);
			if (NeedToShow.size() == 0)
				displayptr->display(current_panorama, blackvec);
			else
				displayptr->display(current_panorama, NeedToShow);
		}
	}
		
}


void GimExec::Thstitch()
{
	while (1)
	{
		if (!current_local.empty())
		{
			current_local.copyTo(current_local_stitch);
			gimpanoptr->warp(current_local_stitch);
		}
		
	}
	
}

void GimExec::Thstart()
{
	std::thread Thread1(&GimExec::Thshoot, this);
	std::thread Thread2(&GimExec::Thtracking, this);
	std::thread Thread3(&GimExec::Thshowref, this);
	//std::thread Thread4(&GimExec::Thshowpanorama, this);
	//std::thread Thread5(&GimExec::Thstitch, this);
	Thread1.detach();
	Thread2.detach();
	Thread3.detach();
	//Thread4.detach();
	//Thread5.detach();
}

void GimExec::test_Thstart()
{
	std::thread Thread1(&GimExec::Thtracking, this);
	std::thread Thread2(&GimExec::Thshowref, this);
	std::thread Thread3(&GimExec::Thshowpanorama, this);
	Thread1.detach();
	Thread2.detach();
	Thread3.detach();
}