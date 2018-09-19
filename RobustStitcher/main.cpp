/**
* @brief main function of reference camera stitcher
* @author Shane Yuan
* @date May 24, 2017
*/

#include <stdio.h>
#include <cstdlib>
#include <opencv2/opencv.hpp>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "FeatureMatch.h"
#include "CameraParamEstimator.h"
#include "Compositor.h"

using namespace std;
using namespace cv;

#define Row 9
#define Col 8
float dX = 40.0, X_MIN = 0, dY = 30.0, Y_MIN = 0;

bool isOverlap(const cv::Rect &rc1, const cv::Rect &rc2)
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

int findoverlap(cv::Point corner_current, cv::Size size_current, vector<Point>& corners, vector<Size>& sizes, std::vector<int>& index)
{
	cv::Rect Rect_current(corner_current, size_current);
	for (int i = 0; i < Row*Col; i++)
	{
		cv::Rect temp(corners[i], sizes[i]);
		if (isOverlap(Rect_current, temp))
		{
			index.push_back(i);   //存放着相连图像的序号，用过后别忘了清空
		}
	}
	return 0;
}

int save_para(vector<calib::CameraParams>& cameras, vector<Point>& corners, vector<Size>& sizes)
{
	ofstream para;
	Mat K;
	para.open("E:/code/from-zero/Sln/para.txt", ios::out);
	if (!para)
		cout << "No have txt" << endl;
	for (int i = 0; i < cameras.size(); i++)
	{
		para << cameras[i].focal << " " << cameras[i].aspect << " "
			<< cameras[i].ppx << " " << cameras[i].ppy << " ";
		//可以考虑看下Mat_模板类，K()const函数被定义在camera.cpp里
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
		para << endl;
	}
	para.close();

	return 0;
}

int read_para(vector<calib::CameraParams> &cameras, vector<Point> &corners, vector<Size>&sizes)
{
	ifstream para;
	Mat K;
	para.open("E:/code/from-zero/Sln/para.txt");
	if (!para.is_open())
	{
		cout << "can not open txt" << endl;
		return -1;
	}
	string str;

	for (int i = 0; i < Row*Col; i++)   //这里没有自动计算图片个数！！！！！！
	{
		getline(para, str, ' ');
		cameras[i].focal = stof(str);
		getline(para, str, ' ');
		cameras[i].aspect = stof(str);
		getline(para, str, ' ');
		cameras[i].ppx = stof(str);
		getline(para, str, ' ');
		cameras[i].ppy = stof(str);
		cameras[i].R.create(3, 3, CV_64FC1);
		for (int j = 0; j < 3; j++)
		{
			for (int k = 0; k < 3; k++)
			{
				getline(para, str, ' ');
				cameras[i].R.at<double>(j, k) = stof(str);
			}
		}
		getline(para, str, ' ');
		corners[i].x = stoi(str);
		getline(para, str, ' ');
		corners[i].y = stoi(str);
		getline(para, str, ' ');
		sizes[i].width = stoi(str);
		getline(para, str, ' ');
		sizes[i].height = stoi(str);
		/*for (int j = 0; j < 3; j++)
		{
		getline(para, str, ' ');
		cameras[i].t.at<float>(j, 0) = stof(str);
		}*/
		getline(para, str);
	}
	para.close();
	return 0;
}

int GetCurrentPara(vector<calib::CameraParams>& cameras, cv::Point2f current_point, calib::CameraParams &current_para)
{
	///build a para matrix
	cv::Mat focals(Row, Col, CV_64FC1), ppxs(Row, Col, CV_64FC1), ppys(Row, Col, CV_64FC1);
	int index = 0;
	//////R and T
	vector<cv::Mat>Rs(9);   //每层对应一个r元素, 每层元素的个数对应图片个数
	for (int i = 0; i < 9; i++)
	{
		Rs[i].create(Row, Col, CV_64FC1);
	}
	//vector<cv::Mat>Ts(3);
	for (int i = 0; i < Col; i++)      //从上往下读数据
	{
		for (int j = 0; j < Row; j++)
		{
			focals.at<double>(j, i) = cameras[index].focal;      //第j行第i列
			ppxs.at<double>(j, i) = cameras[index].ppx;
			ppys.at<double>(j, i) = cameras[index].ppy;
			////每运行一组kl循环代表矩阵一个位置被填上 深度为9
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

	///////////////上面的最后还是要加到readpara函数里的
	vector<double>value;
	vector<float>mapx(1, (current_point.x - X_MIN)*1.0 / dX);    //默认坐标从00开始
	vector<float>mapy(1, (current_point.y - Y_MIN)*1.0 / dY);    //默认坐标从00开始
	remap(focals, value, mapx, mapy, INTER_LINEAR);    //可能需要clear一下，得实验返回值是push还是覆盖
	current_para.focal = value[0];
	value.clear();

	remap(ppxs, value, mapx, mapy, INTER_LINEAR);
	current_para.ppx = value[0];
	value.clear();

	remap(ppys, value, mapx, mapy, INTER_LINEAR);
	current_para.ppy = value[0];
	value.clear();
	current_para.R.create(3, 3, CV_64FC1);
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			remap(Rs[i * 3 + j], value, mapx, mapy, INTER_LINEAR);
			current_para.R.at<double>(i, j) = value[0];
			value.clear();
		}
	}
	return 0;
}

int warp(std::vector<cv::Mat>& imgs)
{
	vector<calib::CameraParams> cameras(Row*Col);
	vector<Point> corners(Row*Col);
	vector<Size> sizes(Row*Col);
	calib::CameraParams current_para;
	cv::Mat src = imread("E:/datasets/2018.2.9/bb/local_110_230_0.png");
	cv::resize(src, src, cv::Size(1000, 750));
	read_para(cameras, corners, sizes);
	GetCurrentPara(cameras, cv::Point2f(110.0, 230.0), current_para);
	/////////求出corner_current和size_current///////////////
	cv::Mat src_warped, mask, mask_warped;
	cv::Point corner_current;
	cv::Size size_current;
	cv::Ptr<cv::detail::SphericalWarper> w = cv::makePtr<cv::detail::SphericalWarper>(false);
	std::shared_ptr<cv::detail::Blender> blender_ = std::make_shared<cv::detail::MultiBandBlender>(false);
	w->setScale(16000);
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
	findoverlap(corner_current, size_current, corners, sizes, index);   //求出compositor.index
	std::cout << index.size() << std::endl;

	//calib::FeatureMatch match;
	calib::FeatureMatch match;
	std::vector<calib::Imagefeature> features(index.size());
	calib::Imagefeature current_feature;
	std::vector<calib::Matchesinfo> current_matchesInfo(index.size());
	for (int i = 0; i < index.size(); i++)
	{
		match.current_feature_thread_(src, imgs[index[i]], features[i], current_feature, current_matchesInfo[i],i);    //建立起拼接图像和周围图像的特征匹配
	}
	//calib::BundleAdjustment bundleAdjust;
	calib::BundleAdjustment bundleAdjust;
	bundleAdjust.refine_BA(index, current_feature, features, current_matchesInfo, cameras, current_para);    //得到当前current_camera的准确值
	
	
	for (int i = 0; i < index.size(); i++)
	{
		/*if (current_matchesInfo[i].confidence < 1.5)
			continue;*/
		if (current_matchesInfo[i].confidence < 2.9)
			continue;
		cv::Mat result;
		// make result image
		int width = src.cols * 2;
		int height = src.rows;
		result.create(height, width, CV_8UC3);
		cv::Rect rect(0, 0, src.cols, height);
		src.copyTo(result(rect));
		rect.x += src.cols;
		imgs[index[i]].copyTo(result(rect));
		// draw matching points
		cv::RNG rng(12345);
		int r = 3;
		for (size_t kind = 0; kind < current_matchesInfo[i].matches.size(); kind++) {
			if (current_matchesInfo[i].inliers_mask[kind]) {
				cv::Scalar color = cv::Scalar(rng.uniform(0, 255),
					rng.uniform(0, 255), rng.uniform(0, 255));
				const cv::DMatch& m = current_matchesInfo[i].matches[kind];
				cv::Point2f p1 = current_feature.keypt[m.queryIdx].pt;
				cv::Point2f p2 = features[i].keypt[m.trainIdx].pt;
				p2.x += src.cols;
				cv::circle(result, p1, r, color, -1, cv::LINE_8, 0);
				cv::circle(result, p2, r, color, -1, cv::LINE_8, 0);
				//cv::line(result, p1, p2, color, 5, cv::LINE_8, 0);
			}
		}
		cv::imwrite(cv::format("E:/code/from-zero/Sln/features/matching_points_%02d_%02d.jpg", -1, index[i]),
			result);
	}
	
	
	calib::Compositor compositor;
	compositor.single_composite(current_para, cameras, src, imgs);

	return 0;
}




int main(int argc, char* argv[]) {

	google::InitGoogleLogging(argv[0]);

    std::string datapath = "E:/datasets/2018.2.9/cc2";
    //std::string outpath = "out";
    std::vector<cv::Mat> imgs;
	for (int i = 0; i <= 280; i = i + 40) {
		for (int j = 0; j <= 240; j = j + 30) {
			imgs.push_back(cv::imread(cv::format("%s/local_%d_%d_0.png", 
				datapath.c_str(), i, j)));
		}
	}
	//for (int i = 1; i < 5; i++) {
	//	imgs.push_back(cv::imread(cv::format("E:/data/giga/360/%d.jpg", i)));
	//}
	int img_num = imgs.size();
	for (size_t i = 0; i < imgs.size(); i++) {
		cv::resize(imgs[i], imgs[i], cv::Size(1000, 750));
	}
	warp(imgs);
	////cv::Mat pano;
	////cv::Ptr<cv::Stitcher> stitcher = cv::Stitcher::create(cv::Stitcher::PANORAMA, false);
	////cv::Stitcher::Status status = stitcher->stitch(imgs, pano);
	////cv::imwrite("opencv_origin.jpg", pano);

	//cv::Mat connection;
	//connection = cv::Mat::zeros(img_num, img_num, CV_8U);
	////calib::Connection::addConnection(connection, 0, 1);
	////calib::Connection::addConnection(connection, 1, 2);
	////calib::Connection::addConnection(connection, 2, 3);
	////calib::Connection::addConnection(connection, 3, 4);
	////calib::Connection::addConnection(connection, 4, 5);
	//for (size_t i = 0; i < imgs.size(); i++) {
	//	for (size_t j = 0; j < imgs.size(); j++) {
	//		if (i == j)
	//			continue;
	//		int row1 = i / Row;
	//		int col1 = i % Row;
	//		int row2 = j / Row;
	//		int col2 = j % Row;
	//		if (abs(row1 - row2) <= 1 && abs(col1 - col2) <= 1) {
	//			connection.at<uchar>(i, j) = 1;
	//			connection.at<uchar>(j, i) = 1;
	//		}
	//	}
	//}

	//calib::FeatureMatch match;
	//match.init(imgs, connection);
	//match.match();
	////match.debug();

	//calib::CameraParamEstimator estimator;
	//estimator.init(imgs, connection, match.getImageFeatures(), match.getMatchesInfo());
	//estimator.estimate();

	//calib::Compositor compositor;
	//compositor.init(imgs, estimator.getCameraParams());
	//compositor.composite();
	//save_para(compositor.cameras, compositor.corners, compositor.sizes);
	//warp(imgs);
    return 0;    
}