# The Double Camera System for Low-Cost Surveillance
## Overview
![image](https://github.com/li19960612/Gimbal_DoubleCamera/blob/master/demo/camera.PNG)
This project aims to achieve low-cost surveillance in large scenes with a dual camera system. The self-built double camera system consists of a fixed large FOV camera with low definition(***global camera***) and a rotatable small FOV camera with high definition(***local camera***). The global camera is responsible for obtaining the location of the key target as a guidence and the local camera aims to capture high-definition details.

## Hardware
The rotatable telephoto lens is equipped with a stable motor, which is controlled by the pulse signal sent by a STM32. The frequency of the pulse signal controls the speed of rotation, and the number of pulses signal controls the angle of rotation. The host and STM32 communicate through serial port, which can send desired number and frequency of pulses and receive the actual rotation angle from the feedback of rotary encoder of the motor.

## Pipeline
For convenience, our target is pedestrian only. The pedestrians are firstly detected in global frame with [YOLOv3](https://arxiv.org/abs/1804.02767), thanks to [@Alexey](https://github.com/AlexeyAB) for providing a [C++ version](https://github.com/AlexeyAB/darknet) that can be compiled on windows. The high-definition details are captured while the local camera rotate to target pedestrians so that the faces can be detected at sufficent resolution.

For the convenience of viewing, the local frame will be warped into the global background. There are two modes for the global background: 
1. The global frame will be directly selected when the scene is rich.
2. The panorama which is generated from the scan of local camera in advance will be chose when the scene is sparse.

In order to achieve long-term correspondence between the details and global pedestrian trajectories, pedestrians will be tracked with [KCF](https://arxiv.org/abs/1404.7584)(single target) or [DeepSort](https://arxiv.org/abs/1703.07402)(multiple target) after extracting details, these tracking algorithms can all run with cpu in real time. 

## Extra details
### Position Calibration
The position of the local frame in global frame need to be known before each rotation. The *TemplateMatch* with RGB and edge features are used to find the coarse position in global frame.

### Panorama Stitching
If the mode *2* is used for the global background, the panorama will be generated after sequential scan of local camera follow the steps: *feature extracting and matching*&rarr;*estimating camera parameters using BundleAdjustment*&rarr;*synthetic panorama with camera parameters*. The warping process from local frame to global frame is similar to the above.

### Color correction
Each camera is slightly different in the process of white balance, so that the global frame and local frame present different colors. A simple color correction is shown below:
$$Src^{'} = (Src-\overline{Src})\times\frac{\mathrm{Var}(Dst)}{\mathrm{Var}(Src)}+\overline{Dst}.$$

### Speed Improvement
Implement the following measures for real-time system:

1. Using ceres-solver to solve the BundleAdjustment Optimization.
2. Extracting and matching features in GPU.
3. Separate thread for each task.

## Installation
### STM32
You need import the folder of STM32 into the compiling tools [Keil-v5](https://www.keil.com/), and write the code with [J-link](https://www.segger.com/products/debug-probes/j-link/) into the STM32 microcontroller, which is only responsible for sending pulses and receiving feedback.

### Pre-compiled C++ library
In order to run the system, you need the installation of Opencv with CUDA and CUDA package(for basis), ceres-solver and eigen3(for fast BundleAdjustment), OpenGL(for display), XIMEA or PointGray driven(for enabling camera).

## Demo
A coarse but worked real-time demo is in **"demo/output.avi"**, there are some problems with online playback temporarily.

## Reference
The DoubleCamera system is a improved version with the [GigaPixel System](http://xiaoyunyuan.net/iccp_giga.html)
