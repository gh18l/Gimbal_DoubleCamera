/**
@brief FeatureMatch.h
C++ source file for feature matching
@author Shane Yuan
@date Jan 30, 2018
*/

#include <numeric>
#include "Common.hpp"
#include "FeatureMatch.h"
#include <iostream>

using namespace std;

#define DEBUG_FEATURE_MATCH
//#define DRAW_FEATURES
/**
@brief funcions of Connection class
*/
int calib::Connection::construct(cv::Mat & connection, size_t imgnum) {
	connection = cv::Mat::zeros(imgnum, imgnum, CV_8U);
	return 0;
}
int calib::Connection::addConnection(cv::Mat & connection, size_t ind1, size_t ind2) {
	connection.at<uchar>(ind1, ind2) = 1;
	connection.at<uchar>(ind2, ind1) = 1;
	return 0;
}
int calib::Connection::removeConnection(cv::Mat & connection, size_t ind1, size_t ind2) {
	connection.at<uchar>(ind1, ind2) = 0;
	connection.at<uchar>(ind2, ind1) = 0;
	return 0;
}
int calib::Connection::countConnections(cv::Mat & connection) {
	return cv::countNonZero(connection);
}
std::vector<std::pair<size_t, size_t>> calib::Connection::getConnections(cv::Mat & connection) {
	std::vector<std::pair<size_t, size_t>> connections;
	for (size_t i = 0; i < connection.rows - 1; i++) {
		for (size_t j = i + 1; j < connection.cols; j++) {
			if (connection.at<uchar>(i, j) != 0) {
				connections.push_back(std::pair<size_t, size_t>(i, j));
			}
		}
	}
	return connections;
}

/**
@brief funcions of Matchesinfo class
*/
calib::Matchesinfo::Matchesinfo() : src_img_idx(-1), dst_img_idx(-1), num_inliers(0), confidence(0) {}


/**
@brief funcions of FeatureMatch class
*/
calib::FeatureMatch::FeatureMatch() : maxThNum(4), matchConf(0.3f),
	featureNumThresh(6) 
{
}
calib::FeatureMatch::~FeatureMatch() {}

/**
@brief init feature matcher
@param std::vector<cv::Mat> imgs : input vector containing images
@param cv::Mat connection: input matrix denote the neighboring information
of the input images
@return int
*/
int calib::FeatureMatch::init(std::vector<cv::Mat> imgs, cv::Mat connection) {
	this->imgs = imgs;
	this->connection = connection;
	this->imgnum = imgs.size();
	thStatus.resize(imgnum);
	ths.resize(imgnum);
	features.resize(imgnum);
	for (size_t i = 0; i < imgnum; i++) {
		thStatus[i] = 0;
		features[i].imgsize = imgs[i].size();
		features[i].ind = i;
	}
	return 0;
}

/**
@breif compute features for one image
@param int ind: image index
@return int
*/
void calib::FeatureMatch::build_feature_thread_(int index) {
	cv::cuda::SURF_CUDA surf_d;
	cv::cuda::GpuMat des_d;
	cv::cuda::GpuMat keypt_d;
	cv::cuda::GpuMat img_d;
	// set surf parameter
	surf_d.keypointsRatio = 0.1f;
	surf_d.hessianThreshold = featureExtractorParam.hess_thresh;
	surf_d.extended = false;
	surf_d.nOctaves = featureExtractorParam.num_octaves;
	surf_d.nOctaveLayers = featureExtractorParam.num_layers;
	surf_d.upright = false;
	// upload image
	img_d.upload(imgs[index]);
	cv::cuda::cvtColor(img_d, img_d, CV_BGR2GRAY);
	// extract keypoints
	cv::cuda::GpuMat keypoints_;
	cv::cuda::GpuMat descriptors_;
	surf_d(img_d, cv::cuda::GpuMat(), keypoints_);
	// calculate descriptors
	surf_d.nOctaves = featureExtractorParam.num_octaves_descr;
	surf_d.nOctaveLayers = featureExtractorParam.num_layers_descr;
	surf_d.upright = true;
	surf_d(img_d, cv::cuda::GpuMat(), keypoints_, descriptors_, true);
	surf_d.downloadKeypoints(keypoints_, features[index].keypt);
	descriptors_.download(features[index].des);
	SysUtil::infoOutput(SysUtil::sprintf("Finish feature extractor on image %d .", index));
	thStatus[index] = 0;
}

/**
@brief compute features
@return int
*/
int calib::FeatureMatch::buildFeatures() {
	// start threads for feature extraction
	thStatus.resize(imgnum);
	ths.resize(imgnum);
	for (size_t i = 0; i < imgnum; i++) {
		thStatus[i] = 0;
	}
	int activeThNum = std::accumulate(thStatus.begin(), thStatus.end(), 0);
	int index = 0;
	for (;;) {
		if (activeThNum < maxThNum) {
			thStatus[index] = 1;
			ths[index] = std::thread(&calib::FeatureMatch::build_feature_thread_, this, index);
#ifdef DEBUG_FEATURE_MATCH
			SysUtil::infoOutput(SysUtil::sprintf("Apply feature extractor on image %d ...", index));
#endif
			index++;
			if (index >= imgnum) {
				break;
			}
		}
		else {
			SysUtil::sleep(200);
		}
		activeThNum = std::accumulate(thStatus.begin(), thStatus.end(), 0);
	}
	// exit thread
	for (size_t i = 0; i < imgnum; i++) {
		ths[i].join();
	}
	return 0;
}

/**
@breif apply knn feature matching between two image descriptors
@param int ind1: index of the first image
@param int ind2: index of the second image
@return int
*/
void calib::FeatureMatch::match_feature_thread_(int index, int ind1, int ind2, 
	Matchesinfo & matchesinfo) {
	cv::cuda::GpuMat des1, des2;
	// upload descriptors
	des1.upload(features[ind1].des);
	des2.upload(features[ind2].des);
	// init l1 descriptor matcher
	cv::Ptr<cv::cuda::DescriptorMatcher> matcher = 
		cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_L1);
	// init variables for matching
	MatchesSet matches;
	std::vector< std::vector<cv::DMatch> > pair_matches;
	// find 1->2 matches
	pair_matches.clear();
	matcher->knnMatch(des1, des2, pair_matches, 2);
	for (size_t i = 0; i < pair_matches.size(); ++i) {
		if (pair_matches[i].size() < 2)
			continue;
		const cv::DMatch& m0 = pair_matches[i][0];
		const cv::DMatch& m1 = pair_matches[i][1];
		if (m0.distance < (1.f - matchConf) * m1.distance) {
			matchesinfo.matches.push_back(m0);
			matches.insert(std::make_pair(m0.queryIdx, m0.trainIdx));
		}
	}
	// find 2->1 matches
	pair_matches.clear();
	matcher->knnMatch(des2, des1, pair_matches, 2);
	for (size_t i = 0; i < pair_matches.size(); ++i) {
		if (pair_matches[i].size() < 2)
			continue;
		const cv::DMatch& m0 = pair_matches[i][0];
		const cv::DMatch& m1 = pair_matches[i][1];
		if (m0.distance < (1.f - matchConf) * m1.distance)
			if (matches.find(std::make_pair(m0.trainIdx, m0.queryIdx)) == matches.end())
				matchesinfo.matches.push_back(cv::DMatch(m0.trainIdx, m0.queryIdx, m0.distance));
	}
	// reset thread status
	matchesinfo.src_img_idx = ind1;
	matchesinfo.dst_img_idx = ind2;

	// compute essential matrix (homography)
	// Check if it makes sense to find homography
	if (matchesinfo.matches.size() < featureNumThresh) {
		SysUtil::infoOutput(SysUtil::sprintf("Image pair"\
			" <%u, %u> does not have enough number of feature points ...", ind1, ind2));
		thStatus[index] = 0;
		return;
	}
	// Construct point-point correspondences for homography estimation
	cv::Mat src_points(1, static_cast<int>(matchesinfo.matches.size()), CV_32FC2);
	cv::Mat dst_points(1, static_cast<int>(matchesinfo.matches.size()), CV_32FC2);
	for (size_t i = 0; i < matchesinfo.matches.size(); ++i) {
		const cv::DMatch& m = matchesinfo.matches[i];
		cv::Point2f p = features[ind1].keypt[m.queryIdx].pt;
		p.x -= features[ind1].imgsize.width * 0.5f;
		p.y -= features[ind1].imgsize.height * 0.5f;
		src_points.at<cv::Point2f>(0, static_cast<int>(i)) = p;
		p = features[ind2].keypt[m.trainIdx].pt;
		p.x -= features[ind2].imgsize.width * 0.5f;
		p.y -= features[ind2].imgsize.height * 0.5f;
		dst_points.at<cv::Point2f>(0, static_cast<int>(i)) = p;
	}
	// Find pair-wise motion
	matchesinfo.H = findHomography(src_points, dst_points, matchesinfo.inliers_mask, cv::RANSAC);
	if (matchesinfo.H.empty() || std::abs(cv::determinant(matchesinfo.H))
		< std::numeric_limits<double>::epsilon()) {
		SysUtil::infoOutput(SysUtil::sprintf("Image pair"\
			" <%u, %u> findHomography failed ...", ind1, ind2));
		thStatus[index] = 0;
		return;
	}
	// Find number of inliers
	matchesinfo.num_inliers = 0;
	for (size_t i = 0; i < matchesinfo.inliers_mask.size(); ++i)
		if (matchesinfo.inliers_mask[i])
			matchesinfo.num_inliers++;
	// These coeffs are from paper M. Brown and D. Lowe. "Automatic Panoramic Image Stitching
	// using Invariant Features"
	matchesinfo.confidence = matchesinfo.num_inliers / (8 + 0.3 * matchesinfo.matches.size());

	//// Set zero confidence to remove matches between too close images, as they don't provide
	//// additional information anyway. The threshold was set experimentally.
	//matchesinfo.confidence = matchesinfo.confidence > 3. ? 0. : matchesinfo.confidence;

	// Check if we should try to refine motion
	if (matchesinfo.num_inliers < featureNumThresh) {
		// Construct point-point correspondences for inliers only
		src_points.create(1, matchesinfo.num_inliers, CV_32FC2);
		dst_points.create(1, matchesinfo.num_inliers, CV_32FC2);
		int inlier_idx = 0;
		for (size_t i = 0; i < matchesinfo.matches.size(); ++i) {
			if (!matchesinfo.inliers_mask[i])
				continue;
			const cv::DMatch& m = matchesinfo.matches[i];
			cv::Point2f p = features[ind1].keypt[m.queryIdx].pt;
			p.x -= features[ind1].imgsize.width * 0.5f;
			p.y -= features[ind1].imgsize.height * 0.5f;
			src_points.at<cv::Point2f>(0, inlier_idx) = p;
			p = features[ind2].keypt[m.trainIdx].pt;
			p.x -= features[ind2].imgsize.width * 0.5f;
			p.y -= features[ind2].imgsize.height * 0.5f;
			dst_points.at<cv::Point2f>(0, inlier_idx) = p;
			inlier_idx++;
		}
		// Rerun motion estimation on inliers only
		matchesinfo.H = findHomography(src_points, dst_points, cv::RANSAC);
	}

	SysUtil::infoOutput(SysUtil::sprintf("Finish feature matcher on image pair"\
		" <%u, %u> ...", ind1, ind2));
	thStatus[index] = 0;
}

/**
@brief match feature descriptors between images with connection
@return int
*/
int calib::FeatureMatch::buildMatchers() {
	// get connection pairs
	std::vector<std::pair<size_t, size_t>> pairs = Connection::getConnections(connection);
	size_t connectionNum = pairs.size();
	matchesInfo.resize(connectionNum);
	// init threads
	thStatus.resize(connectionNum);
	ths.resize(connectionNum);
	for (size_t i = 0; i < connectionNum; i++) {
		thStatus[i] = 0;
	}
	int activeThNum = std::accumulate(thStatus.begin(), thStatus.end(), 0);
	int index = 0;
	for (;;) {
		if (activeThNum < maxThNum) {
			thStatus[index] = 1;
			ths[index] = std::thread(&calib::FeatureMatch::match_feature_thread_, this, 
				index, pairs[index].first, pairs[index].second, std::ref(matchesInfo[index]));
#ifdef DEBUG_FEATURE_MATCH
			SysUtil::infoOutput(SysUtil::sprintf("Apply feature matcher on image pair"\
				" <%u, %u> ...", pairs[index].first, pairs[index].second));
#endif
			index++;
			if (index >= connectionNum) {
				break;
			}
		}
		else {
			SysUtil::sleep(200);
		}
		activeThNum = std::accumulate(thStatus.begin(), thStatus.end(), 0);
	}
	// exit thread
	for (size_t i = 0; i < connectionNum; i++) {
		ths[i].join();
	}
	return 0;
}

/**
@brief function for debugging
*/
int calib::FeatureMatch::match() {
	this->buildFeatures();
	this->buildMatchers();
	save_features(features);
	return 0;
}

/**
@brief get functions
*/
std::vector<calib::Matchesinfo> calib::FeatureMatch::getMatchesInfo() {
	return matchesInfo;
}
std::vector<calib::Imagefeature> calib::FeatureMatch::getImageFeatures() {
	return features;
}

/**
@brief functions for visual debugging
@param size_t ind1: input index of the first image
@param size_t ind2: input index of the second image
@return cv::Mat: output image with drawn matching points
*/
cv::Mat calib::FeatureMatch::visualizeMatchingPts(size_t ind1, size_t ind2) {
	cv::Mat result;
	// make result image
	int width = imgs[ind1].cols * 2;
	int height = imgs[ind1].rows;
	result.create(height, width, CV_8UC3);
	cv::Rect rect(0, 0, imgs[ind1].cols, height);
	imgs[ind1].copyTo(result(rect));
	rect.x += imgs[ind1].cols;  
	imgs[ind2].copyTo(result(rect));
	// find matches info index
	int infoInd = -1;
	std::vector<std::pair<size_t, size_t>> pairs = Connection::getConnections(connection);
	for (size_t i = 0; i < pairs.size(); i ++) {
		if ((pairs[i].first == ind1 && pairs[i].second == ind2) ||
			(pairs[i].first == ind2 && pairs[i].second == ind1)) {
			infoInd = i;
			break;
		}
	}
	// draw matching points
	cv::RNG rng(12345);
    int r = 3;
	if (infoInd != -1) {
		for (size_t kind = 0; kind < matchesInfo[infoInd].matches.size(); kind++) {
			if (matchesInfo[infoInd].inliers_mask[kind]) {
				cv::Scalar color = cv::Scalar(rng.uniform(0, 255), 
					rng.uniform(0, 255), rng.uniform(0, 255));
				const cv::DMatch& m = matchesInfo[infoInd].matches[kind];
				cv::Point2f p1 = features[ind1].keypt[m.queryIdx].pt;
				cv::Point2f p2 = features[ind2].keypt[m.trainIdx].pt;
				p2.x += imgs[ind1].cols;
				cv::circle(result, p1, r, color, -1, cv::LINE_8, 0);
				cv::circle(result, p2, r, color, -1, cv::LINE_8, 0);
				//cv::line(result, p1, p2, color, 5, cv::LINE_8, 0);
			}
		}
	}
	return result;
}

/**
@brief debug function
*/
int calib::FeatureMatch::debug() {
	// get connection pairs
	std::vector<std::pair<size_t, size_t>> pairs = Connection::getConnections(connection);
	size_t connectionNum = pairs.size();
	for (size_t i = 0; i < connectionNum; i ++) {
		cv::Mat result = calib::FeatureMatch::visualizeMatchingPts(pairs[i].first, pairs[i].second);
		cv::imwrite(cv::format("matching_points_%02d_%02d.jpg", pairs[i].first, pairs[i].second),
			result);
	}	
	return 0;
}

void calib::FeatureMatch::current_feature_thread_(cv::Mat& src, calib::Imagefeature& feature,
													calib::Imagefeature& current_feature, calib::Matchesinfo& matchesinfo,int i)
{
	clock_t start, finish;
	if (i == 0)
	{
		current_feature.imgsize = src.size();
		cv::cuda::SURF_CUDA surf_d1;
		cv::cuda::GpuMat img_d1;
		surf_d1.keypointsRatio = 0.1f;
		surf_d1.hessianThreshold = 5000.0;
		surf_d1.extended = false;
		surf_d1.nOctaves = featureExtractorParam.num_octaves;
		surf_d1.nOctaveLayers = featureExtractorParam.num_layers;
		surf_d1.upright = false;
		// upload image
		img_d1.upload(src);
		cv::cuda::cvtColor(img_d1, img_d1, CV_BGR2GRAY);
		// extract keypoints
		cv::cuda::GpuMat keypoints_1;
		cv::cuda::GpuMat descriptors_1;
		surf_d1(img_d1, cv::cuda::GpuMat(), keypoints_1);
		// calculate descriptors
		surf_d1.nOctaves = featureExtractorParam.num_octaves_descr;
		surf_d1.nOctaveLayers = featureExtractorParam.num_layers_descr;
		surf_d1.upright = true;
		surf_d1(img_d1, cv::cuda::GpuMat(), keypoints_1, descriptors_1, true);
		surf_d1.downloadKeypoints(keypoints_1, current_feature.keypt);
		descriptors_1.download(current_feature.des);
	}

	cv::cuda::GpuMat des1, des2;
	// upload descriptors
	des1.upload(current_feature.des);
	des2.upload(feature.des);     //表示每张图像的特征描述矩阵，每个特征有128维，一张图像n个特征
										  // init l1 descriptor matcher

	cv::Ptr<cv::cuda::DescriptorMatcher> matcher =
		cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_L1);
	// init variables for matching
	MatchesSet matches;
	std::vector< std::vector<cv::DMatch> > pair_matches;
	// find 1->2 matches
	pair_matches.clear();
	matcher->knnMatch(des1, des2, pair_matches, 2);   //pair_matches是两张图片的特征匹配对
	for (size_t i = 0; i < pair_matches.size(); ++i) {
		if (pair_matches[i].size() < 2)
			continue;
		const cv::DMatch& m0 = pair_matches[i][0];
		const cv::DMatch& m1 = pair_matches[i][1];    //分别表示置信度按序排列的一次特征匹配
		if (m0.distance < (1.f - matchConf) * m1.distance) {
			matchesinfo.matches.push_back(m0);     //信任本次特征匹配
			matches.insert(std::make_pair(m0.queryIdx, m0.trainIdx));
		}
	}
	// find 2->1 matches
	pair_matches.clear();
	matcher->knnMatch(des2, des1, pair_matches, 2);
	for (size_t i = 0; i < pair_matches.size(); ++i) {
		if (pair_matches[i].size() < 2)
			continue;
		const cv::DMatch& m0 = pair_matches[i][0];
		const cv::DMatch& m1 = pair_matches[i][1];
		if (m0.distance < (1.f - matchConf) * m1.distance)
			if (matches.find(std::make_pair(m0.trainIdx, m0.queryIdx)) == matches.end())
				matchesinfo.matches.push_back(cv::DMatch(m0.trainIdx, m0.queryIdx, m0.distance));
	}

	// compute essential matrix (homography)
	// Check if it makes sense to find homography
	if (matchesinfo.matches.size() < featureNumThresh) {
		SysUtil::infoOutput(SysUtil::sprintf("Image pair"\
			" <> does not have enough number of feature points ..."));
		return;
	}
	// Construct point-point correspondences for homography estimation
	cv::Mat src_points(1, static_cast<int>(matchesinfo.matches.size()), CV_32FC2);
	cv::Mat dst_points(1, static_cast<int>(matchesinfo.matches.size()), CV_32FC2);
	
	for (size_t i = 0; i < matchesinfo.matches.size(); ++i) {
		const cv::DMatch& m = matchesinfo.matches[i];
		cv::Point2f p = current_feature.keypt[m.queryIdx].pt;
		p.x -= current_feature.imgsize.width * 0.5f;
		p.y -= current_feature.imgsize.height * 0.5f;
		src_points.at<cv::Point2f>(0, static_cast<int>(i)) = p;
		p = feature.keypt[m.trainIdx].pt;
		p.x -= feature.imgsize.width * 0.5f;
		p.y -= feature.imgsize.height * 0.5f;
		dst_points.at<cv::Point2f>(0, static_cast<int>(i)) = p;
	}
	// Find pair-wise motion
	matchesinfo.H = findHomography(src_points, dst_points, matchesinfo.inliers_mask, cv::RANSAC);
	if (matchesinfo.H.empty() || std::abs(cv::determinant(matchesinfo.H))
		< std::numeric_limits<double>::epsilon()) {
		SysUtil::infoOutput(SysUtil::sprintf("Image pair"\
			" <> findHomography failed ..."));
		return;
	}
	// Find number of inliers
	matchesinfo.num_inliers = 0;
	for (size_t i = 0; i < matchesinfo.inliers_mask.size(); ++i)
		if (matchesinfo.inliers_mask[i])
			matchesinfo.num_inliers++;
	// These coeffs are from paper M. Brown and D. Lowe. "Automatic Panoramic Image Stitching
	// using Invariant Features"
	matchesinfo.confidence = matchesinfo.num_inliers / (8 + 0.3 * matchesinfo.matches.size());

	//// Set zero confidence to remove matches between too close images, as they don't provide
	//// additional information anyway. The threshold was set experimentally.
	//matchesinfo.confidence = matchesinfo.confidence > 3. ? 0. : matchesinfo.confidence;

	// Check if we should try to refine motion
	if (matchesinfo.num_inliers < featureNumThresh) {
		// Construct point-point correspondences for inliers only
		src_points.create(1, matchesinfo.num_inliers, CV_32FC2);
		dst_points.create(1, matchesinfo.num_inliers, CV_32FC2);
		int inlier_idx = 0;
		for (size_t i = 0; i < matchesinfo.matches.size(); ++i) {
			if (!matchesinfo.inliers_mask[i])
				continue;
			const cv::DMatch& m = matchesinfo.matches[i];
			cv::Point2f p = current_feature.keypt[m.queryIdx].pt;
			p.x -= current_feature.imgsize.width * 0.5f;
			p.y -= current_feature.imgsize.height * 0.5f;
			src_points.at<cv::Point2f>(0, inlier_idx) = p;
			p = feature.keypt[m.trainIdx].pt;
			p.x -= feature.imgsize.width * 0.5f;
			p.y -= feature.imgsize.height * 0.5f;
			dst_points.at<cv::Point2f>(0, inlier_idx) = p;
			inlier_idx++;
		}
		// Rerun motion estimation on inliers only
		matchesinfo.H = findHomography(src_points, dst_points, cv::RANSAC);
	}




	// reset thread status
	//matchesinfo.src_img_idx = ind1;
	//matchesinfo.dst_img_idx = ind2;

	// compute essential matrix (homography)
	// Check if it makes sense to find homography
	if (matchesinfo.matches.size() < featureNumThresh) {
		SysUtil::infoOutput(SysUtil::sprintf("Image pair"\
			" <> does not have enough number of feature points ..."));
		return;
	}
}

int calib::FeatureMatch::global2pano(cv::Mat global, cv::Mat pano)       //根据类中的当前点和当前脉冲使云台移动到目标像素位置
{
	calib::Imagefeature global_feature,pano_feature;
	cv::Size size = global.size();
	resize(pano, pano, size);
	global_feature.imgsize = global.size();
	cv::cuda::SURF_CUDA surf_d1;
	cv::cuda::GpuMat img_d1;
	surf_d1.keypointsRatio = 0.1f;
	surf_d1.hessianThreshold = featureExtractorParam.hess_thresh;
	surf_d1.extended = false;
	surf_d1.nOctaves = featureExtractorParam.num_octaves;
	surf_d1.nOctaveLayers = featureExtractorParam.num_layers;
	surf_d1.upright = false;
	surf_d1.hessianThreshold = featureExtractorParam.hess_thresh;
	// upload image
	img_d1.upload(global);
	cv::cuda::cvtColor(img_d1, img_d1, CV_BGR2GRAY);
	// extract keypoints
	cv::cuda::GpuMat keypoints_1;
	cv::cuda::GpuMat descriptors_1;
	surf_d1(img_d1, cv::cuda::GpuMat(), keypoints_1);
	// calculate descriptors
	surf_d1.nOctaves = featureExtractorParam.num_octaves_descr;
	surf_d1.nOctaveLayers = featureExtractorParam.num_layers_descr;
	surf_d1.upright = true;
	surf_d1(img_d1, cv::cuda::GpuMat(), keypoints_1, descriptors_1, true);
	surf_d1.downloadKeypoints(keypoints_1, global_feature.keypt);
	descriptors_1.download(global_feature.des);

	pano_feature.imgsize = pano.size();
	cv::cuda::SURF_CUDA surf_d2;
	cv::cuda::GpuMat img_d2;
	surf_d2.keypointsRatio = 0.1f;
	surf_d2.hessianThreshold = featureExtractorParam.hess_thresh;
	surf_d2.extended = false;
	surf_d2.nOctaves = featureExtractorParam.num_octaves;
	surf_d2.nOctaveLayers = featureExtractorParam.num_layers;
	surf_d2.upright = false;
	surf_d2.hessianThreshold = featureExtractorParam.hess_thresh;
	// upload image
	img_d2.upload(pano);
	cv::cuda::cvtColor(img_d2, img_d2, CV_BGR2GRAY);
	// extract keypoints
	cv::cuda::GpuMat keypoints_2;
	cv::cuda::GpuMat descriptors_2;
	surf_d2(img_d2, cv::cuda::GpuMat(), keypoints_2);
	// calculate descriptors
	surf_d2.nOctaves = featureExtractorParam.num_octaves_descr;
	surf_d2.nOctaveLayers = featureExtractorParam.num_layers_descr;
	surf_d2.upright = true;
	surf_d2(img_d2, cv::cuda::GpuMat(), keypoints_2, descriptors_2, true);
	surf_d2.downloadKeypoints(keypoints_2, pano_feature.keypt);
	descriptors_2.download(pano_feature.des);


	//////////////////////////////////////
	calib::Matchesinfo matchesinfo;
	cv::cuda::GpuMat des1, des2;
	// upload descriptors
	des1.upload(global_feature.des);
	des2.upload(pano_feature.des);     //表示每张图像的特征描述矩阵，每个特征有128维，一张图像n个特征
								  // init l1 descriptor matcher
	cv::Ptr<cv::cuda::DescriptorMatcher> matcher =
		cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_L1);
	// init variables for matching
	MatchesSet matches;
	std::vector< std::vector<cv::DMatch> > pair_matches;
	// find 1->2 matches
	pair_matches.clear();
	matcher->knnMatch(des1, des2, pair_matches, 2);   //pair_matches是两张图片的特征匹配对
	for (size_t i = 0; i < pair_matches.size(); ++i) {
		if (pair_matches[i].size() < 2)
			continue;
		const cv::DMatch& m0 = pair_matches[i][0];
		const cv::DMatch& m1 = pair_matches[i][1];    //分别表示置信度按序排列的一次特征匹配
		if (m0.distance < (1.f - matchConf) * m1.distance) {
			matchesinfo.matches.push_back(m0);     //信任本次特征匹配
			matches.insert(std::make_pair(m0.queryIdx, m0.trainIdx));
		}
	}
	// find 2->1 matches
	pair_matches.clear();
	matcher->knnMatch(des2, des1, pair_matches, 2);
	for (size_t i = 0; i < pair_matches.size(); ++i) {
		if (pair_matches[i].size() < 2)
			continue;
		const cv::DMatch& m0 = pair_matches[i][0];
		const cv::DMatch& m1 = pair_matches[i][1];
		if (m0.distance < (1.f - matchConf) * m1.distance)
			if (matches.find(std::make_pair(m0.trainIdx, m0.queryIdx)) == matches.end())
				matchesinfo.matches.push_back(cv::DMatch(m0.trainIdx, m0.queryIdx, m0.distance));
	}

	// compute essential matrix (homography)
	// Check if it makes sense to find homography
	// Construct point-point correspondences for homography estimation
	cv::Mat src_points(1, static_cast<int>(matchesinfo.matches.size()), CV_32FC2);
	cv::Mat dst_points(1, static_cast<int>(matchesinfo.matches.size()), CV_32FC2);
	for (size_t i = 0; i < matchesinfo.matches.size(); ++i) {
		const cv::DMatch& m = matchesinfo.matches[i];
		cv::Point2f p = global_feature.keypt[m.queryIdx].pt;
		p.x -= global_feature.imgsize.width * 0.5f;
		p.y -= global_feature.imgsize.height * 0.5f;
		src_points.at<cv::Point2f>(0, static_cast<int>(i)) = p;
		p = pano_feature.keypt[m.trainIdx].pt;
		p.x -= pano_feature.imgsize.width * 0.5f;
		p.y -= pano_feature.imgsize.height * 0.5f;
		dst_points.at<cv::Point2f>(0, static_cast<int>(i)) = p;
	}
	matchesinfo.H = findHomography(src_points, dst_points, matchesinfo.inliers_mask, cv::RANSAC,2);
	// Find number of inliers
	matchesinfo.num_inliers = 0;
	for (size_t i = 0; i < matchesinfo.inliers_mask.size(); ++i)
		if (matchesinfo.inliers_mask[i])
			matchesinfo.num_inliers++;

	matchesinfo.confidence = matchesinfo.num_inliers / (8 + 0.3 * matchesinfo.matches.size());


	// Check if we should try to refine motion
	if (matchesinfo.num_inliers < featureNumThresh) {
		// Construct point-point correspondences for inliers only
		src_points.create(1, matchesinfo.num_inliers, CV_32FC2);
		dst_points.create(1, matchesinfo.num_inliers, CV_32FC2);
		int inlier_idx = 0;
		for (size_t i = 0; i < matchesinfo.matches.size(); ++i) {
			if (!matchesinfo.inliers_mask[i])
				continue;
			const cv::DMatch& m = matchesinfo.matches[i];
			cv::Point2f p = global_feature.keypt[m.queryIdx].pt;
			p.x -= global_feature.imgsize.width * 0.5f;
			p.y -= global_feature.imgsize.height * 0.5f;
			src_points.at<cv::Point2f>(0, inlier_idx) = p;
			p = pano_feature.keypt[m.trainIdx].pt;
			p.x -= pano_feature.imgsize.width * 0.5f;
			p.y -= pano_feature.imgsize.height * 0.5f;
			dst_points.at<cv::Point2f>(0, inlier_idx) = p;
			inlier_idx++;
		}
		// Rerun motion estimation on inliers only
		matchesinfo.H = findHomography(src_points, dst_points, cv::RANSAC);
	}






	cv::Mat new_x(global.rows, global.cols, CV_32FC1);
	cv::Mat new_y(global.rows, global.cols, CV_32FC1);
	cv::Mat new_z(global.rows, global.cols, CV_32FC1);
	cv::Mat dst;
	matchesinfo.H.convertTo(matchesinfo.H, CV_32F);
	for (size_t i = 0; i < global.rows; i++)
	{
		for (size_t j = 0; j < global.cols; j++)
		{
			new_x.at<float>(i, j) = matchesinfo.H.at<float>(0, 0) * j + matchesinfo.H.at<float>(0, 1) * i
				+ matchesinfo.H.at<float>(0, 2);
			new_y.at<float>(i, j) = matchesinfo.H.at<float>(1, 0) * j + matchesinfo.H.at<float>(1, 1) * i
				+ matchesinfo.H.at<float>(1, 2);
			new_z.at<float>(i, j) = matchesinfo.H.at<float>(2, 0) * j + matchesinfo.H.at<float>(2, 1) * i
				+ matchesinfo.H.at<float>(2, 2);
			new_x.at<float>(i, j) = new_x.at<float>(i, j) / new_z.at<float>(i, j);
			new_y.at<float>(i, j) = new_y.at<float>(i, j) / new_z.at<float>(i, j);
		}
	}
	remap(global, dst, new_x, new_y, cv::INTER_CUBIC);
		

#ifdef DRAW_FEATURES
	cv::Mat result;
	// make result image
	int width = global.cols * 2;
	int height = global.rows;
	result.create(height, width, CV_8UC3);
	cv::Rect rect(0, 0, global.cols, height);
	global.copyTo(result(rect));
	rect.x += global.cols;
	pano.copyTo(result(rect));
	// draw matching points
	cv::RNG rng(12345);
	int r = 3;
	for (size_t kind = 0; kind < matchesinfo.matches.size(); kind++) {
		if (matchesinfo.inliers_mask[kind]) {
			cv::Scalar color = cv::Scalar(rng.uniform(0, 255),
				rng.uniform(0, 255), rng.uniform(0, 255));
			const cv::DMatch& m = matchesinfo.matches[kind];
			cv::Point2f p1 = global_feature.keypt[m.queryIdx].pt;
			cv::Point2f p2 = pano_feature.keypt[m.trainIdx].pt;
			p2.x += global.cols;
			cv::circle(result, p1, r, color, -1, cv::LINE_8, 0);
			cv::circle(result, p2, r, color, -1, cv::LINE_8, 0);
			//cv::line(result, p1, p2, color, 5, cv::LINE_8, 0);
		}
	}
	cv::imwrite("E:/code/gimble3.20/matching_points.jpg",result);
#endif
	return 0;
}


//int calib::FeatureMatch::save_features(std::vector<calib::Imagefeature>& feature)
//{
//	std::ofstream para;
//	para.open("E:/code/project/gimble3.23/featurepara.txt", std::ios::out);
//	if (!para)
//	{
//		std::cout << "No have txt" << std::endl;
//		return -1;
//	}
//	para << feature.size() << std::endl;
//	for (int k = 0; k < feature.size(); k++)
//	{
//		para << feature[k].imgsize.width << " " << feature[k].imgsize.height << " ";
//		para << feature[k].des.rows << " " << feature[k].des.cols << " ";
//		for (int i = 0; i < feature[k].des.rows; i++)
//		{
//			for (int j = 0; j < feature[k].des.cols; j++)
//			{
//				para << feature[k].des.at<float>(i, j) << " ";
//			}
//		}
//		para << feature[k].keypt.size() << " ";
//		for (int i = 0; i < feature[k].keypt.size(); i++)
//		{
//			para << feature[k].keypt[i].pt.x << " " << feature[k].keypt[i].pt.y << " ";
//		}
//		para << std::endl;
//	}
//
//	
//	para.close();
//
//	return 0;
//}

int calib::FeatureMatch::save_features(std::vector<calib::Imagefeature>& feature)
{
	int num = feature.size();
	std::ofstream para;
	para.open(cv::format("%s/featurepara.txt", GimUtil::datapath.c_str()), ios::binary | std::ios::out);
	if (!para)
	{
		std::cout << "No have txt" << std::endl;
		return -1;
	}
	para.write((char*)(&num), sizeof(int));
	//put data into the struct
	//featureParaList->keypt1->keypt2->...->keyptn->des
	for (int k = 0; k < feature.size(); k++)
	{
		calib::featureParaList featurePara;
		featurePara.width = feature[k].imgsize.width;
		featurePara.height = feature[k].imgsize.height;
		featurePara.des_row = feature[k].des.rows;
		featurePara.des_col = feature[k].des.cols;
		featurePara.keypt_size = feature[k].keypt.size();
		para.write((char*)(&featurePara), sizeof(calib::featureParaList));
		for (int i = 0; i < feature[k].keypt.size(); i++)
		{
			calib::keyptvalue keypt;
			keypt.ptx = feature[k].keypt[i].pt.x;
			keypt.pty = feature[k].keypt[i].pt.y;
			para.write((char*)(&keypt), sizeof(calib::keyptvalue));
		}
		para.write((char*)(feature[k].des.data), sizeof(float) * feature[k].des.cols * feature[k].des.rows);
		
	}

	para.close();

	return 0;
}

//int calib::FeatureMatch::read_features(std::vector<calib::Imagefeature>& feature)
//{
//	std::ifstream para;
//
//	para.open("E:/code/project/gimble3.23/featurepara.txt");
//	if (!para.is_open())
//	{
//		std::cout << "can't open txt" << std::endl;
//		return -1;
//	}
//	int num;
//	std::string str;
//	getline(para, str);
//	num = stoi(str);
//	for (int k = 0; k < num; k++)
//	{
//		calib::Imagefeature current_feature;
//		getline(para, str, ' ');
//		current_feature.imgsize.width = stoi(str);
//		getline(para, str, ' ');
//		current_feature.imgsize.height = stoi(str);
//
//		getline(para, str, ' ');
//		int rows = stoi(str);
//		getline(para, str, ' ');
//		int cols = stoi(str);
//
//		cv::Mat desMat(rows, cols, CV_32F);
//		for (int i = 0; i < rows; i++)
//		{
//			for (int j = 0; j < cols; j++)
//			{
//				getline(para, str, ' ');
//				desMat.at<float>(i, j) = stof(str);
//			}
//		}
//		desMat.copyTo(current_feature.des);
//
//		getline(para, str, ' ');
//		int num_keypt = stoi(str);
//		current_feature.keypt.resize(num_keypt);
//		for (int i = 0; i < num_keypt; i++)
//		{
//			getline(para, str, ' ');
//			current_feature.keypt[i].pt.x = stof(str);
//			getline(para, str, ' ');
//			current_feature.keypt[i].pt.y = stof(str);
//		}
//
//		feature.push_back(current_feature);
//
//		getline(para, str);
//	}
//
//	return 0;
//}

double calib::FeatureMatch::match_people(cv::Mat img1, cv::Mat img2)
{
	calib::Imagefeature img1_feature, img2_feature;
	cv::Size size1 = img1.size();
	cv::Size size2 = img2.size();
	img1_feature.imgsize = img1.size();
	img2_feature.imgsize = img2.size();
	cv::cuda::SURF_CUDA surf_d1;
	cv::cuda::GpuMat img_d1;
	surf_d1.keypointsRatio = 0.1f;
	surf_d1.hessianThreshold = 1.0f;
	surf_d1.extended = false;
	surf_d1.nOctaves = featureExtractorParam.num_octaves;
	surf_d1.nOctaveLayers = featureExtractorParam.num_layers;
	surf_d1.upright = false;
	// upload image
	img_d1.upload(img1);
	cv::cuda::cvtColor(img_d1, img_d1, CV_BGR2GRAY);
	// extract keypoints
	cv::cuda::GpuMat keypoints_1;
	cv::cuda::GpuMat descriptors_1;
	surf_d1(img_d1, cv::cuda::GpuMat(), keypoints_1);
	// calculate descriptors
	surf_d1.nOctaves = featureExtractorParam.num_octaves_descr;
	surf_d1.nOctaveLayers = featureExtractorParam.num_layers_descr;
	surf_d1.upright = true;
	surf_d1(img_d1, cv::cuda::GpuMat(), keypoints_1, descriptors_1, true);
	surf_d1.downloadKeypoints(keypoints_1, img1_feature.keypt);
	descriptors_1.download(img1_feature.des);

	cv::cuda::SURF_CUDA surf_d2;
	cv::cuda::GpuMat img_d2;
	surf_d2.keypointsRatio = 0.1f;
	surf_d2.hessianThreshold = 1.0f;
	surf_d2.extended = false;
	surf_d2.nOctaves = featureExtractorParam.num_octaves;
	surf_d2.nOctaveLayers = featureExtractorParam.num_layers;
	surf_d2.upright = false;
	// upload image
	img_d2.upload(img2);
	cv::cuda::cvtColor(img_d2, img_d2, CV_BGR2GRAY);
	// extract keypoints
	cv::cuda::GpuMat keypoints_2;
	cv::cuda::GpuMat descriptors_2;
	surf_d2(img_d2, cv::cuda::GpuMat(), keypoints_2);
	// calculate descriptors
	surf_d2.nOctaves = featureExtractorParam.num_octaves_descr;
	surf_d2.nOctaveLayers = featureExtractorParam.num_layers_descr;
	surf_d2.upright = true;
	surf_d2(img_d2, cv::cuda::GpuMat(), keypoints_2, descriptors_2, true);
	surf_d2.downloadKeypoints(keypoints_2, img2_feature.keypt);
	descriptors_2.download(img2_feature.des);


	//////////////////////////////////////
	calib::Matchesinfo matchesinfo;
	cv::cuda::GpuMat des1, des2;
	// upload descriptors
	des1.upload(img1_feature.des);
	des2.upload(img2_feature.des);     //表示每张图像的特征描述矩阵，每个特征有128维，一张图像n个特征
									   // init l1 descriptor matcher
	cv::Ptr<cv::cuda::DescriptorMatcher> matcher =
		cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_L1);
	// init variables for matching
	MatchesSet matches;
	std::vector< std::vector<cv::DMatch> > pair_matches;
	// find 1->2 matches
	pair_matches.clear();
	matcher->knnMatch(des1, des2, pair_matches, 2);   //pair_matches是两张图片的特征匹配对
	for (size_t i = 0; i < pair_matches.size(); ++i) {
		if (pair_matches[i].size() < 2)
			continue;
		const cv::DMatch& m0 = pair_matches[i][0];
		const cv::DMatch& m1 = pair_matches[i][1];    //分别表示置信度按序排列的一次特征匹配
		if (m0.distance < (1.f - 0.1) * m1.distance) {
			matchesinfo.matches.push_back(m0);     //信任本次特征匹配
			matches.insert(std::make_pair(m0.queryIdx, m0.trainIdx));
		}
	}
	// find 2->1 matches
	pair_matches.clear();
	matcher->knnMatch(des2, des1, pair_matches, 2);
	for (size_t i = 0; i < pair_matches.size(); ++i) {
		if (pair_matches[i].size() < 2)
			continue;
		const cv::DMatch& m0 = pair_matches[i][0];
		const cv::DMatch& m1 = pair_matches[i][1];
		if (m0.distance < (1.f - 0.1) * m1.distance)
			if (matches.find(std::make_pair(m0.trainIdx, m0.queryIdx)) == matches.end())
				matchesinfo.matches.push_back(cv::DMatch(m0.trainIdx, m0.queryIdx, m0.distance));
	}

	// compute essential matrix (homography)
	// Check if it makes sense to find homography
	// Construct point-point correspondences for homography estimation
	cv::Mat src_points(1, static_cast<int>(matchesinfo.matches.size()), CV_32FC2);
	cv::Mat dst_points(1, static_cast<int>(matchesinfo.matches.size()), CV_32FC2);
	for (size_t i = 0; i < matchesinfo.matches.size(); ++i) {
		const cv::DMatch& m = matchesinfo.matches[i];
		cv::Point2f p = img1_feature.keypt[m.queryIdx].pt;
		p.x -= img1_feature.imgsize.width * 0.5f;
		p.y -= img1_feature.imgsize.height * 0.5f;
		src_points.at<cv::Point2f>(0, static_cast<int>(i)) = p;
		p = img2_feature.keypt[m.trainIdx].pt;
		p.x -= img2_feature.imgsize.width * 0.5f;
		p.y -= img2_feature.imgsize.height * 0.5f;
		dst_points.at<cv::Point2f>(0, static_cast<int>(i)) = p;
	}
	if(src_points.cols > 0 && dst_points.cols > 0)
		matchesinfo.H = findHomography(src_points, dst_points, matchesinfo.inliers_mask, cv::RANSAC, 1);
	// Find number of inliers
	matchesinfo.num_inliers = 0;
	for (size_t i = 0; i < matchesinfo.inliers_mask.size(); ++i)
		if (matchesinfo.inliers_mask[i])
			matchesinfo.num_inliers++;

	matchesinfo.confidence = matchesinfo.num_inliers / (8 + 0.3 * matchesinfo.matches.size());


	// Check if we should try to refine motion
	if (matchesinfo.num_inliers < featureNumThresh) {
		// Construct point-point correspondences for inliers only
		src_points.create(1, matchesinfo.num_inliers, CV_32FC2);
		dst_points.create(1, matchesinfo.num_inliers, CV_32FC2);
		int inlier_idx = 0;
		for (size_t i = 0; i < matchesinfo.matches.size(); ++i) {
			if (!matchesinfo.inliers_mask[i])
				continue;
			const cv::DMatch& m = matchesinfo.matches[i];
			cv::Point2f p = img1_feature.keypt[m.queryIdx].pt;
			p.x -= img1_feature.imgsize.width * 0.5f;
			p.y -= img1_feature.imgsize.height * 0.5f;
			src_points.at<cv::Point2f>(0, inlier_idx) = p;
			p = img2_feature.keypt[m.trainIdx].pt;
			p.x -= img2_feature.imgsize.width * 0.5f;
			p.y -= img2_feature.imgsize.height * 0.5f;
			dst_points.at<cv::Point2f>(0, inlier_idx) = p;
			inlier_idx++;
		}
		// Rerun motion estimation on inliers only
		if (src_points.cols > 0 && dst_points.cols > 0)
			matchesinfo.H = findHomography(src_points, dst_points, cv::RANSAC);
	}

#ifdef DRAW_FEATURES
	cv::Mat result;
	// make result image
	int width = img1.cols + img2.cols;
	int height = std::max<int>(img1.rows, img2.rows);
	result.create(height, width, CV_8UC3);
	cv::Rect rect1(0, 0, img1.cols, img1.rows);
	img1.copyTo(result(rect1));
	cv::Rect rect2(img1.cols, 0, img2.cols, img2.rows);
	img2.copyTo(result(rect2));
	// draw matching points
	cv::RNG rng(12345);
	int r = 3;
	for (size_t kind = 0; kind < matchesinfo.matches.size(); kind++) {
		if (matchesinfo.inliers_mask[kind]) {
			cv::Scalar color = cv::Scalar(rng.uniform(0, 255),
				rng.uniform(0, 255), rng.uniform(0, 255));
			const cv::DMatch& m = matchesinfo.matches[kind];
			cv::Point2f p1 = img1_feature.keypt[m.queryIdx].pt;
			cv::Point2f p2 = img2_feature.keypt[m.trainIdx].pt;
			p2.x += img1.cols;
			cv::circle(result, p1, r, color, -1, cv::LINE_8, 0);
			cv::circle(result, p2, r, color, -1, cv::LINE_8, 0);
			//cv::line(result, p1, p2, color, 5, cv::LINE_8, 0);
		}
	}
	cv::imwrite("E:/code/project/gimble4.14/matching_points.png", result);
#endif
	return matchesinfo.confidence;
}