#include "GimUtil.h"
#include <time.h>
GimUtil::GimUtil(std::shared_ptr<CSerial> SerialPtr,
	std::shared_ptr<GimCamera> GimCameraPtr) : pulse_per_pixel(7)
{
	serial_ptr = SerialPtr;
	gimcamera_ptr = GimCameraPtr;
	ptr = cv::ximgproc::createStructuredEdgeDetection(cv::format("%s/model.yml", datapath.c_str()));
}
GimUtil::~GimUtil(){}


int GimUtil::sleep(int miliseconds)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(miliseconds));
		return 0;
}

void GimUtil::move_delay()
{
	sleep(1000);
}

bool GimUtil::isInside(cv::Point2f pt, cv::Rect rect) {
	if (rect.contains(pt))
		return true;
	else return false;
}

cv::Mat GimUtil::SEDDetector(cv::Mat img, float scale) {
	cv::Mat edgeImg;
	cv::Size size_large = img.size();
	cv::Size size_small = cv::Size(size_large.width * scale, size_large.height * scale);
	cv::resize(img, img, size_small);
	img.convertTo(img, cv::DataType<float>::type, 1 / 255.0);
	ptr->detectEdges(img, edgeImg);
	edgeImg = edgeImg * 255;
	cv::resize(edgeImg, edgeImg, size_large);
	edgeImg.convertTo(edgeImg, CV_8U);
	return edgeImg;
}

int GimUtil::colorCorrectRGB(cv::Mat & srcImg, cv::Mat dstImg) 
{
	cv::Scalar meanSrc, stdSrc, meanDst, stdDst;
	cv::meanStdDev(srcImg, meanSrc, stdSrc);
	cv::meanStdDev(dstImg, meanDst, stdDst);
	std::vector<cv::Mat> channel;
	cv::split(srcImg, channel);
	for (int i = 0; i < 3; i++)
	{
		channel[i].convertTo(channel[i], -1, stdDst.val[i] / stdSrc.val[i],
			meanDst.val[i] - stdDst.val[i] / stdSrc.val[i] * meanSrc.val[i]);
	}
	cv::merge(channel, srcImg);
	
	return 0;
}

int GimUtil::find_position(cv::Mat refImg, cv::Mat localImg, cv::Point &out_point)
{
	cv::Mat refEdge = this->SEDDetector(refImg, 1.0);
	cv::Mat localEdge = this->SEDDetector(localImg, 1.0);
	// resize localview image
	cv::Mat templ, templEdge;
	sizeBlock = cv::Size(localImg.cols * scale, localImg.rows * scale);
	cv::resize(localImg, templ, sizeBlock);
	cv::resize(localEdge, templEdge, sizeBlock);

	cv::Mat result, resultEdge;
	cv::matchTemplate(refImg, templ, result, cv::TM_CCOEFF_NORMED);
	cv::matchTemplate(refEdge, templEdge, resultEdge, cv::TM_CCOEFF_NORMED);
	result = result.mul(resultEdge);

	cv::Point maxLoc;
	cv::minMaxLoc(result, NULL, NULL, NULL, &maxLoc);
	out_point = maxLoc;

	return 0;
}

int GimUtil::find_position(cv::Mat refImg, cv::Mat localImg, cv::Point &out_point, double &max_value)
{
	cv::Mat refEdge = this->SEDDetector(refImg, 1.0);
	cv::Mat localEdge = this->SEDDetector(localImg, 1.0);
	// resize localview image
	cv::Mat templ, templEdge;
	sizeBlock = cv::Size(localImg.cols * scale, localImg.rows * scale);
	cv::resize(localImg, templ, sizeBlock);
	cv::resize(localEdge, templEdge, sizeBlock);

	cv::Mat result, resultEdge;
	cv::matchTemplate(refImg, templ, result, cv::TM_CCOEFF_NORMED);
	cv::matchTemplate(refEdge, templEdge, resultEdge, cv::TM_CCOEFF_NORMED);
	result = result.mul(resultEdge);

	cv::Point maxLoc;
	cv::minMaxLoc(result, NULL, &max_value, NULL, &maxLoc);
	out_point = maxLoc;

	return 0;
}

int GimUtil::find_position_onlycolor(cv::Mat refImg, cv::Mat localImg, cv::Point &out_point)
{
	// resize localview image
	cv::Mat templ, templEdge;
	sizeBlock = cv::Size(localImg.cols * scale, localImg.rows * scale);
	cv::resize(localImg, templ, sizeBlock);

	cv::Mat result;
	cv::matchTemplate(refImg, templ, result, cv::TM_CCOEFF_NORMED);

	cv::Point maxLoc;
	cv::minMaxLoc(result, NULL, NULL, NULL, &maxLoc);
	out_point = maxLoc;

	return 0;
}

int GimUtil::find_position_onlycolor(cv::Mat refImg, cv::Mat localImg, cv::Point &out_point, double &max_value)
{
	// resize localview image
	cv::Mat templ, templEdge;
	sizeBlock = cv::Size(localImg.cols * scale, localImg.rows * scale);
	cv::resize(localImg, templ, sizeBlock);

	cv::Mat result;
	cv::matchTemplate(refImg, templ, result, cv::TM_CCOEFF_NORMED);

	cv::Point maxLoc;
	cv::minMaxLoc(result, NULL, &max_value, NULL, &maxLoc);
	out_point = maxLoc;

	return 0;
}

int GimUtil::gimble_find_position(cv::Mat refImg, cv::Mat localImg, cv::Point ref_point, 
	                               float region_mul,cv::Point &out_point)     //region_mul must bigger than 1,reginal:243*182
{
	int region_width, region_height;
	cv::Rect imgrect(0, 0, refImg.cols, refImg.rows);
	cv::Size origin_size(refImg.cols*scale, refImg.rows*scale);
	cv::Size new_size(refImg.cols*scale*region_mul, refImg.rows*scale*region_mul);
	cv::Point2f new_point = cv::Point2f(ref_point.x - (new_size.width - origin_size.width) / 2,
		ref_point.y - (new_size.height - origin_size.height) / 2);
	cv::Rect new_region;
	cv::Mat new_ref;

	if (isInside(new_point, imgrect) && isInside(cv::Point2f(new_point.x+new_size.width, new_point.y + new_size.height), imgrect))
	{
		new_region=cv::Rect(new_point, new_size);
	}
	else
	{
		out_point = ref_point;
		return -1;
	}
	refImg(new_region).copyTo(new_ref);

	
	cv::Mat refEdge = this->SEDDetector(new_ref, 0.5);
	cv::Mat localEdge = this->SEDDetector(localImg, 0.5);
	// resize localview image
	cv::Mat templ, templEdge;
	sizeBlock = cv::Size(localImg.cols * scale, localImg.rows * scale);
	cv::resize(localImg, templ, sizeBlock);
	cv::resize(localEdge, templEdge, sizeBlock);
	//colorCorrectRGB(templ, new_ref);    ///////////////////////

	cv::Mat result, resultEdge;
	cv::matchTemplate(new_ref, templ, result, cv::TM_CCOEFF_NORMED);
	cv::matchTemplate(refEdge, templEdge, resultEdge, cv::TM_CCOEFF_NORMED);
	result = result.mul(resultEdge);

	cv::Point maxLoc;
	cv::minMaxLoc(result, NULL, NULL, NULL, &maxLoc);
	out_point.x = maxLoc.x + new_point.x;
	out_point.y = maxLoc.y + new_point.y;


	return 0;
}

//only in test
void GimUtil::send_pulse(cv::Point pulse, std::pair<bool, bool> dir)
{
	serial_ptr->Serial_Send_Yaw(pulse.x, dir.first);
	GimUtil::sleep(30);
	serial_ptr->Serial_Send_Pitch(pulse.y, dir.second);
	if (dir.first == 0)
	{
		current_pulse.x = current_pulse.x + pulse.x;
	}
	else
	{
		current_pulse.x = current_pulse.x - pulse.x;
	}
	if (dir.second == 0)
	{
		current_pulse.y = current_pulse.y + pulse.y;
	}
	else
	{
		current_pulse.y = current_pulse.y - pulse.y;
	}
	write_pulse(current_pulse);
}

void GimUtil::send_pulse(cv::Point dst_pulse)   //send absolute pulse,完毕后更新current_pulse
{
	if (dst_pulse.x < current_pulse.x)  //需要往左转
	{
		serial_ptr->Serial_Send_Yaw(current_pulse.x - dst_pulse.x, 1);
		GimUtil::sleep(30);
	}
	else
	{
		serial_ptr->Serial_Send_Yaw(dst_pulse.x - current_pulse.x, 0);
		GimUtil::sleep(30);
	}
	if (dst_pulse.y < current_pulse.y)  //需要往上转
	{
		serial_ptr->Serial_Send_Pitch(current_pulse.y - dst_pulse.y, 1);
		GimUtil::sleep(30);
	}
	else
	{
		serial_ptr->Serial_Send_Pitch(dst_pulse.y - current_pulse.y, 0);
		GimUtil::sleep(30);
	}
	current_pulse = dst_pulse;
	write_pulse(current_pulse);
}

//cv::Point GimUtil::dst2pulse(cv::Point dst)
//{
//	cv::Point pulse;
//	//暂时做一个平均
//	cv::Point delta_point = cv::Point(dst.x - current_point.x,
//		dst.y - current_point.y);
//	pulse = cv::Point(delta_point.x * pulse_per_pixel + current_pulse.x,
//		delta_point.y * pulse_per_pixel + current_pulse.y);
//	return pulse;
//}

cv::Point GimUtil::dst2pulse(cv::Point dst)
{
	double min_area = 9999.0;
	int index = 0;
	for (int i = 0; i < quad_position.size(); i++)
	{
		double area = cv::contourArea(quad_position[i]);
		std::vector<cv::Point> point(3);
		point[0] = dst;
		point[1] = (quad_position[i])[0];
		point[2] = (quad_position[i])[1];
		double area0 = cv::contourArea(point);
		point[0] = dst;
		point[1] = (quad_position[i])[1];
		point[2] = (quad_position[i])[2];
		double area1 = cv::contourArea(point);
		point[0] = dst;
		point[1] = (quad_position[i])[2];
		point[2] = (quad_position[i])[3];
		double area2 = cv::contourArea(point);
		point[0] = dst;
		point[1] = (quad_position[i])[3];
		point[2] = (quad_position[i])[0];
		double area3 = cv::contourArea(point);
		double delta_area = std::fabs(area - area0 - area1 - area2 - area3);
		if (delta_area < min_area)
		{
			min_area = delta_area;
			index = i;
		}
	}
	//bilinear interpolation
	float up_ratio = ((float)dst.x - (float)(quad_position[index])[0].x) / ((float)(quad_position[index])[1].x - (float)(quad_position[index])[0].x);
	float down_ratio = ((float)dst.x - (float)(quad_position[index])[2].x) / ((float)(quad_position[index])[3].x - (float)(quad_position[index])[2].x);
	float up_y = up_ratio * ( (float)(quad_position[index])[0].y - (float)(quad_position[index])[1].y );
	up_y = (float)(quad_position[index])[0].y - up_y;
	float down_y = down_ratio * ((float)(quad_position[index])[3].y - (float)(quad_position[index])[2].y);
	down_y = (float)(quad_position[index])[2].y + down_y;

	float up_pulse_x = (float)(quad_pulse_value[index])[1].x - (float)(quad_pulse_value[index])[0].x;
	up_pulse_x = up_pulse_x * up_ratio;
	up_pulse_x = up_pulse_x + (float)(quad_pulse_value[index])[0].x;

	float up_pulse_y = (float)(quad_pulse_value[index])[1].y - (float)(quad_pulse_value[index])[0].y;
	up_pulse_y = up_pulse_y * up_ratio;
	up_pulse_y = up_pulse_y + (float)(quad_pulse_value[index])[0].y;

	float down_pulse_x = (float)(quad_pulse_value[index])[3].x - (float)(quad_pulse_value[index])[2].x;
	down_pulse_x = down_pulse_x * down_ratio;
	down_pulse_x = down_pulse_x + (float)(quad_pulse_value[index])[0].x;

	float down_pulse_y = (float)(quad_pulse_value[index])[3].y - (float)(quad_pulse_value[index])[2].y;
	down_pulse_y = down_pulse_y * down_ratio;
	down_pulse_y = down_pulse_y + (float)(quad_pulse_value[index])[0].y;

	float y_ratio = ((float)dst.y - up_y) / (down_y - up_y);

	float pulse_x = (down_pulse_x - down_pulse_x) * y_ratio + up_pulse_x;
	float pulse_y = (down_pulse_y - down_pulse_y) * y_ratio + up_pulse_y;

	cv::Point pulse((int)pulse_x, (int)pulse_y);

	return pulse;

}

int GimUtil::move(cv::Point dst_point)
{
	//std::cout << "next point is               " << dst_point << std::endl;
	//std::cout << "current point is             " << current_point << std::endl;
	cv::Point dst_pulse;
	dst_pulse = dst2pulse(dst_point);
	if (dst_pulse.x >= (GimUtil::Col - 1) * GimUtil::dX)
	{
		std::cout << "1" << std::endl;
		return -1;
	}
	if (dst_pulse.y >= (GimUtil::Row - 1) * GimUtil::dY)
	{
		std::cout << "2" << std::endl;
		return -1;
	}
	if (dst_point.x > 1900)
	{
		std::cout << "3" << std::endl;
		return -1;
	}
	if (dst_point.y > 1400)
	{
		std::cout << "4" << std::endl;
		return -1;
	}
	if (dst_pulse.x < 0)
	{
		std::cout << "5" << std::endl;
		return -1;
	}
	if (dst_point.y < 0)
	{
		std::cout << "6" << std::endl;
		return -1;
	}
	/*if (dst_pulse.x >= (GimUtil::Col - 1) * GimUtil::dX || dst_pulse.y >= (GimUtil::Row - 1) * GimUtil::dY
		|| dst_point.x > 1900 || dst_point.y > 1400 || dst_pulse.x < 0 || dst_pulse.y < 0 
		|| dst_point.x < 0 || dst_point.y < 0)
		return -1;*/
	send_pulse(dst_pulse);
	current_point = dst_point;
	return 0;
}

void GimUtil::initposition(cv::Point pulse, std::pair<bool, bool> dir)
{
	send_pulse(pulse, dir);
	while (1)
	{
		cv::Mat ref, local;
		gimcamera_ptr->shoot(ref, local);
		cv::resize(ref, ref, cv::Size(1000, 800));
		cv::resize(local, local, cv::Size(1000, 800));
		cv::imshow("ref", ref);
		cv::imshow("local", local);
		cv::waitKey(1);
	}
}

void GimUtil::estimate_scale()
{
	//0.18~0.2
	float base_scale = 0.18;
	float temp_scale;
	cv::Mat ref, local;
	cv::Point point;
	double max = 0.0;
	float optimal_scale = 0.0f;
	for (int i = 0; i < 80; i++)
	{
		temp_scale = base_scale + i * 0.001;
		gimcamera_ptr->shoot(ref, local);
		double max_val;
		find_position(ref, local, point, max_val);
		if (max_val > max)
		{
			max = max_val;
			optimal_scale = temp_scale;
		}
		cv::Rect roi(point, cv::Size(local.cols * temp_scale, local.rows * temp_scale));
		cv::rectangle(ref, roi, cv::Scalar(0, 0, 255));
		cv::imwrite(cv::format("E:/data/test_data/test_estimate_scale/%f_local.jpg", temp_scale), local);
		cv::imwrite(cv::format("E:/data/test_data/test_estimate_scale/%f_ref.jpg", temp_scale), ref);
		cv::imwrite(cv::format("E:/data/test_data/test_estimate_scale/%f_refblock.jpg", temp_scale), ref(roi));
		std::cout << i << std::endl;
	}
	std::cout << optimal_scale << std::endl;
	system("pause");
}

void GimUtil::write_pulse(cv::Point current_pulse)
{
	std::ofstream pulse;
	pulse.open(cv::format("%s/pulse.txt", GimUtil::datapath.c_str()), std::ios::out);
	pulse << current_pulse.x << '#' << current_pulse.y << '#';
	pulse.close();
}

cv::Point GimUtil::read_pulse()
{
	std::ifstream pulse;
	cv::Point current_pulse;
	pulse.open(cv::format("%s/pulse.txt", GimUtil::datapath.c_str()), std::ios::in);
	std::string str;
	std::getline(pulse, str, '#');
	current_pulse.x = std::stoi(str);
	std::getline(pulse, str, '#');
	current_pulse.y = std::stoi(str);
	pulse.close();

	return current_pulse;
}

void GimUtil::read_point_connection_pulse_para()
{
	std::ifstream file;
	file.open(cv::format("%s/point_connection_pulse.txt", GimUtil::datapath.c_str()), std::ios::in);
	if (!file.is_open())
		std::cout << "open file failed" << std::endl;

	std::string str;
	for (int i = 0; i < (Row - 1) * (Col - 1); i++)
	{
		std::vector<cv::Point> vertices;
		for (int j = 0; j < 4; j++)
		{
			cv::Point vertex;
			std::getline(file, str, '#');
			vertex.x = std::stoi(str);
			std::getline(file, str, '#');
			vertex.y = std::stoi(str);
			vertices.push_back(vertex);
		}
		quad_position.push_back(vertices);
		std::getline(file, str, ' ');
	}
	for (int i = 0; i < (Row - 1) * (Col - 1); i++)
	{
		std::vector<cv::Point> pulse_values;
		for (int j = 0; j < 4; j++)
		{
			cv::Point pulse_value;
			std::getline(file, str, '#');
			pulse_value.x = std::stoi(str);
			std::getline(file, str, '#');
			pulse_value.y = std::stoi(str);
			pulse_values.push_back(pulse_value);
		}
		quad_pulse_value.push_back(pulse_values);
		std::getline(file, str, ' ');
	}
	file.close();
}