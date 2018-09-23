#include "GimPanorama.hpp"

GimPano::GimPano(std::shared_ptr<GimUtil> GimUtilPtr, 
	std::shared_ptr<GimCamera> GimCameraPtr) : dX(GimUtil::dX), X_MIN(GimUtil::X_MIN),
		dY(GimUtil::dY), Y_MIN(GimUtil::Y_MIN), Row(GimUtil::Row), Col(GimUtil::Col)
{
	gimutil_ptr = GimUtilPtr;
	gimcamera_ptr = GimCameraPtr;
	imgs.resize(Row*Col);
	cameras.resize(GimUtil::Row*GimUtil::Col);
	corners.resize(GimUtil::Row*GimUtil::Col + 1);
	sizes.resize(GimUtil::Row*GimUtil::Col + 1);
}
GimPano::~GimPano() {}

void GimPano::init()
{
	match_ptr = std::make_shared<calib::FeatureMatch>();  //不知道有没有必要定义成指针
	bg_model = cv::createBackgroundSubtractorMOG2().dynamicCast<cv::BackgroundSubtractor>();
}

bool GimPano::isOverlap(const cv::Rect &rc1, const cv::Rect &rc2)
{
	if (rc1.x + rc1.width  > rc2.x &&
		rc2.x + rc2.width  > rc1.x &&
		rc1.y + rc1.height > rc2.y &&
		rc2.y + rc2.height > rc1.y
		)
		return true;
	else
		return false;
}

int GimPano::findoverlap()
{
	cv::Rect Rect_current(corner_current, size_current);
	for (int i = 0; i < Row*Col; i++)
	{
		cv::Rect temp(corners[i], sizes[i]);
		if (isOverlap(Rect_current, temp))
		{
			index.push_back(i);  
		}
	}
	return 0;
}

int GimPano::save_para(std::vector<calib::CameraParams>& cameras, 
	std::vector<cv::Point>& corners, std::vector<cv::Size>& sizes)
{
	std::ofstream para;
	cv::Mat K;
	para.open(cv::format("%s/para.txt", GimUtil::datapath.c_str()), std::ios::out);
	if (!para)
		std::cout << "No have txt" << std::endl;
	for (int i = 0; i < cameras.size(); i++)
	{
		para << cameras[i].focal << " " << cameras[i].aspect << " "
			<< cameras[i].ppx << " " << cameras[i].ppy << " ";
		for (int j = 0; j < 3; j++)
		{
			for (int k = 0; k < 3; k++)
			{
				para << cameras[i].R.at<double>(j, k) << " ";
			}
		}
		para << corners[i].x << " " << corners[i].y << " " << sizes[i].width << " " << sizes[i].height << " ";
		/*for (int j = 0; j < 3; j++)
		{
		para << cameras[i].t.at<float>(j,0) << " ";
		}*/
		para << std::endl;
	}
	para.close();

	return 0;
}

int GimPano::read_para()
{
	std::ifstream para;
	cv::Mat K;
	para.open(cv::format("%s/para.txt", GimUtil::datapath.c_str()));
	if (!para.is_open())
	{
		std::cout << "can not open txt" << std::endl;
		return -1;
	}
	std::string str;

	for (int i = 0; i < Row * Col; i++) 
	{
		std::getline(para, str, ' ');
		cameras[i].focal = std::stof(str);
		std::getline(para, str, ' ');
		cameras[i].aspect = std::stof(str);
		std::getline(para, str, ' ');
		cameras[i].ppx = std::stof(str);
		std::getline(para, str, ' ');
		cameras[i].ppy = std::stof(str);
		cameras[i].R.create(3, 3, CV_64FC1);
		for (int j = 0; j < 3; j++)
		{
			for (int k = 0; k < 3; k++)
			{
				std::getline(para, str, ' ');
				cameras[i].R.at<double>(j, k) = std::stof(str);
			}
		}
		std::getline(para, str, ' ');
		corners[i].x = std::stoi(str);
		std::getline(para, str, ' ');
		corners[i].y = std::stoi(str);
		std::getline(para, str, ' ');
		sizes[i].width = std::stoi(str);
		std::getline(para, str, ' ');
		sizes[i].height = std::stoi(str);
		/*for (int j = 0; j < 3; j++)
		{
		getline(para, str, ' ');
		cameras[i].t.at<float>(j, 0) = stof(str);
		}*/
		std::getline(para, str);
	}
	para.close();
	return 0;
}

int GimPano::read_features()
{
	std::ifstream para;

	para.open(cv::format("%s/featurepara.txt", GimUtil::datapath.c_str()), std::ios::binary | std::ios::in);
	if (!para.is_open())
	{
		std::cout << "can't open txt" << std::endl;
		return -1;
	}
	int num;
	para.read((char*)(&num), sizeof(int));
	for (int k = 0; k < num; k++)
	{
		calib::featureParaList featurePara;
		calib::Imagefeature current_feature;
		para.read((char*)(&featurePara), sizeof(calib::featureParaList));
		current_feature.imgsize.width = featurePara.width;
		current_feature.imgsize.height = featurePara.height;
		current_feature.keypt.resize(featurePara.keypt_size);
		for (int i = 0; i < featurePara.keypt_size; i++)
		{
			calib::keyptvalue keypt;
			para.read((char*)(&keypt), sizeof(calib::keyptvalue));
			current_feature.keypt[i].pt.x = keypt.ptx;
			current_feature.keypt[i].pt.y = keypt.pty;
		}
		cv::Mat temp(featurePara.des_row, featurePara.des_col, CV_32F);
		para.read((char*)(temp.data), sizeof(float) * temp.cols * temp.rows);
		temp.copyTo(current_feature.des);
		features.push_back(current_feature);
	}
	para.close();

	return 0;
}

//这个函数有问题 
int GimPano::GetCurrentPara(std::vector<calib::CameraParams>& cameras, 
	cv::Point2f current_pulse, calib::CameraParams &current_para)
{
	///build a para matrix
	cv::Mat focals(Row, Col, CV_64FC1), ppxs(Row, Col, CV_64FC1), ppys(Row, Col, CV_64FC1);
	int index = 0;
	//////R and T
	std::vector<cv::Mat>Rs(9);   //ÿ���Ӧһ��rԪ��, ÿ��Ԫ�صĸ�����ӦͼƬ����
	for (int i = 0; i < 9; i++)
	{
		Rs[i].create(Row, Col, CV_64FC1);
	}
	//vector<cv::Mat>Ts(3);
	for (int i = 0; i < Col; i++)    
	{
		for (int j = 0; j < Row; j++)
		{
			focals.at<double>(j, i) = cameras[index].focal;      
			ppxs.at<double>(j, i) = cameras[index].ppx;
			ppys.at<double>(j, i) = cameras[index].ppy;
			for (int k = 0; k < 3; k++)
			{
				for (int l = 0; l < 3; l++)
				{
					Rs[k * 3 + l].at<double>(j, i) = cameras[index].R.at<double>(k, l); //
				}
			}
			index++;
		}
	}

	std::vector<double> value;
	std::vector<float>mapx(1, (current_pulse.x - X_MIN) * 1.0 / (float)dX);
	std::vector<float>mapy(1, (current_pulse.y - Y_MIN) * 1.0 / (float)dY);
	cv::remap(focals, value, mapx, mapy, cv::INTER_LINEAR);   
	current_para.focal = value[0];
	value.clear();

	cv::remap(ppxs, value, mapx, mapy, cv::INTER_LINEAR);
	current_para.ppx = value[0];
	value.clear();

	cv::remap(ppys, value, mapx, mapy, cv::INTER_LINEAR);
	current_para.ppy = value[0];
	value.clear();
	current_para.R.create(3, 3, CV_64FC1);
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			cv::remap(Rs[i * 3 + j], value, mapx, mapy, cv::INTER_LINEAR);
			current_para.R.at<double>(i, j) = value[0];
			value.clear();
		}
	}
	return 0;
}

void GimPano::get_panorama()
{
	collection();
	int img_num = imgs.size();
	std::cout << img_num << std::endl;
	cv::Mat connection;
	connection = cv::Mat::zeros(img_num, img_num, CV_8U);
	for (size_t i = 0; i < imgs.size(); i++) {
		for (size_t j = 0; j < imgs.size(); j++) {
			if (i == j)
				continue;
			int row1 = i / Row;
			int col1 = i % Row;
			int row2 = j / Row;
			int col2 = j % Row;
			if (abs(row1 - row2) <= 1 && abs(col1 - col2) <= 1) {
				connection.at<uchar>(i, j) = 1;
				connection.at<uchar>(j, i) = 1;
			}
		}
	}

	match_ptr->init(imgs, connection);
	match_ptr->match();
	//match.debug();

	calib::CameraParamEstimator estimator;
	estimator.init(imgs, connection, match_ptr->getImageFeatures(), match_ptr->getMatchesInfo());
	estimator.estimate();

	calib::Compositor compositor;
	compositor.init(imgs, estimator.getCameraParams());
	compositor.composite(panorama);
	panorama.copyTo(current_panorama);
	save_para(compositor.cameras, compositor.corners, compositor.sizes);
}

void GimPano::save_point_connection_pulse_para(std::vector<std::pair<cv::Point, cv::Point>> point_and_pulse)
{
	//convert vector to quadrilateral
	std::vector<std::vector<cv::Point>> quad_point;
	std::vector<std::vector<cv::Point>> quad_pulse;
	for (int i = 0; i < Row - 1; i++)
	{
		for (int j = 0; j < Col - 1; j++)
		{
			std::vector<cv::Point> point;
			std::vector<cv::Point> pulse;
			point.push_back(point_and_pulse[j + i * Col].first);
			point.push_back(point_and_pulse[j + i * Col + 1].first);
			point.push_back(point_and_pulse[j + (i + 1) * Col].first);
			point.push_back(point_and_pulse[j + (i + 1) * Col + 1].first);
			quad_point.push_back(point);
			pulse.push_back(point_and_pulse[j + i * Col].second);
			pulse.push_back(point_and_pulse[j + i * Col + 1].second);
			pulse.push_back(point_and_pulse[j + (i + 1) * Col].second);
			pulse.push_back(point_and_pulse[j + (i + 1) * Col + 1].second);
			quad_pulse.push_back(pulse);
		}
	}

	std::ofstream file;
	file.open(cv::format("%s/point_connection_pulse.txt", GimUtil::datapath.c_str()), std::ios::out);

	for (int i = 0; i < quad_point.size(); i++)
	{
		for (int j = 0; j < 4; j++)
		{
			file << (quad_point[i])[j].x << '#';
			file << (quad_point[i])[j].y << '#';
		}
		file << std::endl;
	}
	for (int i = 0; i < quad_pulse.size(); i++)
	{
		for (int j = 0; j < 4; j++)
		{
			file << (quad_pulse[i])[j].x << '#';
			file << (quad_pulse[i])[j].y << '#';
		}
		file << std::endl;
	}
	file.close();
}

void GimPano::collection()
{
	cv::Mat ref, local;
	cv::Mat pulse_pixel(gimutil_ptr->video_height, gimutil_ptr->video_width, CV_8UC3, cv::Scalar::all(0));
	cv::Point set_init_pulse;
	std::vector<std::pair<cv::Point, cv::Point>> point_and_pulse;
	point_and_pulse.resize(Row * Col);
	for (int i = 0; i < Row; i++)    //固定从脉冲点0 0开始收集
	{
		if (i % 2 == 0)
		{
			for (int j = 0; j < Col; j++)
			{
				gimutil_ptr->send_pulse(cv::Point(dX * j, dY * i));
				GimUtil::move_delay();
				for (int i = 0; i < 1; i++)  
				{
					gimcamera_ptr->shoot(ref, local);
					cv::Mat fgmask;
					bg_model->apply(local, fgmask, -1);
					///////////这里没写完，存下fgmask之后再写
					GimUtil::sleep(20);
				}
				local.copyTo(imgs[j*Row + i]);
				cv::imwrite(cv::format("%s/local_%d_%d.png",
					GimUtil::datapath.c_str(), dX * j, dY * i), local);
				if (i == Row / 2 && j == Col / 2)
					set_init_pulse = cv::Point(dX * j, dY * i);

				//存下来之后再去找，这个为之后的pixel2pulse插值做准备
				cv::Point now_point;
				gimutil_ptr->find_position(ref, local, 
					now_point);
				cv::Rect roi(now_point, cv::Size(ref.cols * GimUtil::scale, ref.rows * GimUtil::scale));
				//仅供live watch   
				pulse_pixel.at<cv::Vec3b>(now_point.y,
					now_point.x)[0] = 0;
				pulse_pixel.at<cv::Vec3b>(now_point.y,
					now_point.x)[1] = 0;
				pulse_pixel.at<cv::Vec3b>(now_point.y,
					now_point.x)[2] = 255;
				//正经存
				point_and_pulse[j + Col * i] = std::pair<cv::Point, cv::Point>(now_point,
					gimutil_ptr->current_pulse);
			}
		}

		if (i % 2 == 1)
		{
			for (int j = Col - 1; j >= 0; j--)
			{
				gimutil_ptr->send_pulse(cv::Point(dX * j, dY * i));
				GimUtil::move_delay();
				for (int i = 0; i < 1; i++)   //选一张没前景的
				{
					gimcamera_ptr->shoot(ref, local);
					cv::Mat fgmask;
					bg_model->apply(local, fgmask, -1);
					///////////这里没写完，存下fgmask之后再写
					GimUtil::sleep(20);
				}
				local.copyTo(imgs[j*Row + i]);
				cv::imwrite(cv::format("%s/local_%d_%d.png",
					GimUtil::datapath.c_str(), dX * j, dY * i), local);
				if (i == Row / 2 && j == Col / 2)
					set_init_pulse = cv::Point(dX * j, dY * i);
				//存下来之后再去找
				cv::Point now_point;
				gimutil_ptr->find_position(ref, local,
					now_point);
				cv::Rect roi(now_point, cv::Size(ref.cols * GimUtil::scale, ref.rows * GimUtil::scale));
				
				//仅供live watch
				pulse_pixel.at<cv::Vec3b>(now_point.y,
					now_point.x)[0] = 0;
				pulse_pixel.at<cv::Vec3b>(now_point.y,
					now_point.x)[1] = 0;
				pulse_pixel.at<cv::Vec3b>(now_point.y,
					now_point.x)[2] = 255;
				//正经存
				point_and_pulse[j + Col * i] = std::pair<cv::Point, cv::Point>(now_point,
					gimutil_ptr->current_pulse);
			}
		}
	}
	cv::imwrite(cv::format("%s/pulse_pixel.png", GimUtil::datapath.c_str()), pulse_pixel);
	save_point_connection_pulse_para(point_and_pulse);
	//回归
	gimutil_ptr->send_pulse(set_init_pulse);
	GimUtil::move_delay();
	gimcamera_ptr->shoot(ref, local);
	gimutil_ptr->find_position(ref, local, gimutil_ptr->current_point);
}




int GimPano::warp(cv::Mat src)
{
	GimUtil::colorCorrectRGB(src, panorama);
	cv::Ptr<cv::detail::SphericalWarper> w = cv::makePtr<cv::detail::SphericalWarper>(false);
	std::shared_ptr<cv::detail::Blender> blender_ = std::make_shared<cv::detail::MultiBandBlender>(false);
	GetCurrentPara(cameras, gimutil_ptr->current_pulse, current_para);
	/////////���corner_current��size_current///////////////
	cv::Mat src_warped, mask, mask_warped;
	cv::Point corner_current;
	cv::Size size_current;
	w->setScale(35000);
	// calculate warping filed
	cv::Mat K, R;
	current_para.K().convertTo(K, CV_32F);
	current_para.R.convertTo(R, CV_32F);
	cv::Mat initmask(src.rows, src.cols, CV_8U);
	initmask.setTo(cv::Scalar::all(255));
	corner_current = w->warp(src, K, R, cv::INTER_LINEAR, cv::BORDER_CONSTANT, src_warped);
	w->warp(initmask, K, R, cv::INTER_NEAREST, cv::BORDER_CONSTANT, mask_warped);
	size_current = mask_warped.size();

	//calib::Compositor compositor;
	std::vector<int> index;
	findoverlap();   //���compositor.index

	calib::FeatureMatch match;
	calib::Imagefeature current_feature;
	std::vector<calib::Matchesinfo> current_matchesInfo(index.size());
	for (int i = 0; i < index.size(); i++)
	{
		match.current_feature_thread_(src, features[index[i]], current_feature, current_matchesInfo[i], i);    //������ƴ��ͼ�����Χͼ�������ƥ��
	}

	//calib::BundleAdjustment bundleAdjust;
	calib::BundleAdjustment bundleAdjust;
	calib::Compositor compositor;
	bundleAdjust.refine_BA(index, current_feature, features, current_matchesInfo, cameras, current_para);
	cv::Mat current_panorama_temp;
	panorama.copyTo(current_panorama_temp);
	compositor.single_composite(current_para, src, current_panorama_temp, corners, sizes);
	current_panorama_temp.copyTo(current_panorama);
	return 0;
}

void GimPano::read_panorama()
{
	panorama = cv::imread(cv::format("%s/result.jpg", GimUtil::datapath.c_str()));
	panorama.copyTo(current_panorama);
	for (int i = 0; i <= 280; i = i + 40) {
		for (int j = 0; j <= 240; j = j + 30) {
			imgs.push_back(cv::imread(cv::format("%s/local_%d_%d.png",
				GimUtil::datapath.c_str(), i, j)));
		}
	}
}