#include "GimCamera.hpp"

GimCamera::GimCamera() {}
GimCamera::~GimCamera() {}

void GimCamera::init()
{
	cameraPtr = cam::createCamera(cam::CameraModel::XIMEA_xiC);
	cameraPtr->init();
	// set camera setting
	cameraPtr->startCapture();
	cameraPtr->setFPS(-1, 20);
	cameraPtr->setAutoExposure(-1, cam::Status::on);
	cameraPtr->setAutoExposureLevel(-1, 30);
	cameraPtr->setAutoWhiteBalance(-1);
	cameraPtr->makeSetEffective();
	// set capturing setting
	cameraPtr->setCamBufferType(cam::GenCamBufferType::Raw);
	cameraPtr->setCaptureMode(cam::GenCamCaptureMode::Continous, 40);
	cameraPtr->setCapturePurpose(cam::GenCamCapturePurpose::Streaming);
	//cameraPtr->setVerbose(true);
	// get camera info
	cameraPtr->getCamInfos(camInfos);
	cameraPtr->startCaptureThreads();
	cameraPtr->makeSetEffective();
}

void GimCamera::shoot(cv::Mat &ref, cv::Mat &local)
{
	char name[200], temp[50];  //
	cv::Mat local_bayer, ref_bayer;
	cv::Mat watching;
	std::vector<cam::Imagedata> imgdatas(2);
	cameraPtr->captureFrame(imgdatas);
	cv::Mat(camInfos[0].height, camInfos[0].width,
		CV_8U, reinterpret_cast<void*>(imgdatas[0].data)).copyTo(ref_bayer);
	cv::Mat(camInfos[1].height, camInfos[1].width,
		CV_8U, reinterpret_cast<void*>(imgdatas[1].data)).copyTo(local_bayer);

	//////////////convert/////////////
	cv::cvtColor(local_bayer, local, CV_BayerRG2BGR);
	cv::cvtColor(ref_bayer, ref, CV_BayerRG2BGR);
	std::vector<cv::Mat> channels(3);
	cv::split(ref, channels);
	channels[0] = channels[0] * camInfos[0].blueGain;
	channels[1] = channels[1] * camInfos[0].greenGain;
	channels[2] = channels[2] * camInfos[0].redGain;
	merge(channels, ref);

	cv::split(local, channels);
	channels[0] = channels[0] * camInfos[1].blueGain;
	channels[1] = channels[1] * camInfos[1].greenGain;
	channels[2] = channels[2] * camInfos[1].redGain;
	cv::merge(channels, local);
	colorCorrectRGB(local, ref);
//#ifdef SAVEAVI
//	writer1 << ref;
//	writer2 << local;
//#endif
//#ifdef _SHOW
//	cv::Mat show1, show2;
//	cv::resize(local, show1, cv::Size(800, 600));
//	cv::resize(ref, show2, cv::Size(800, 600));
//	cv::imshow("local", show1);
//	cv::imshow("ref", show2);
//	cv::waitKey(30);
//#endif
}

void GimCamera::camera_close()
{
	cameraPtr->stopCaptureThreads();
	cameraPtr->release();
}

int GimCamera::colorCorrectRGB(cv::Mat & srcImg, cv::Mat dstImg)
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