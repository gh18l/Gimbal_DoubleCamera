#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#ifdef WIN32
#include <Windows.h>
#endif

#include <glad.h>
#include <glfw3.h>
#include <stdio.h>  
#include <stdlib.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <unordered_map>
#include <math.h>
#include "GimUtil.h"

class Display {
private:

	float yaw = -90.0f;	// yaw is initialized to -90.0 degrees since a yaw of 0.0 results in a direction vector pointing to the right so we initially rotate a bit to the left.
	float pitch = 0.0f;
	// timing
public:
	int display_init(cv::Mat img);
	GLFWwindow* window;

public:
	int display(cv::Mat img, std::vector<cv::Mat> NeedToShow);
	void GenArray(int x, int y, int w, int h);
public:
	Display();
	~Display();

};


#endif
