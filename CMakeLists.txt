
cmake_minimum_required (VERSION 3.1)
set(name "Gimbal_DoubleCamera")
project ( ${name} )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# c++ version
set (CMAKE_CXX_STANDARD 11)

add_definitions(
    -DGOOGLE_GLOG_DLL_DECL=
	-DGLOG_NO_ABBREVIATED_SEVERITIES
)
# opencv 
if ( WIN32 )
    set(OpenCV_DIR "E:/libs/OpenCV/x64/vc14/lib")
    set(OpenCV_CONFIG_PATH "E:/libs/OpenCV")
    include_directories(${WIN_HEADER_PATH})
endif()
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Eigen3
if (WIN32)
	set(EIGEN_INCLUDE_DIR "E:/libs/eigen")
elseif ( UNIX )
		set(EIGEN_INCLUDE_DIR "/usr/local/include/eigen3")
endif()
include_directories(${EIGEN_INCLUDE_DIR})

# opengl
find_package(OpenGL REQUIRED)
# Ask the user to give the paths to GLFW, GLEW and GLM.
# On Linux this can be found automatically with pkg-config; 
# see http://stackoverflow.com/questions/6889264/cmake-flags-for-opengl-using-glew-and-glfw, for instance.
if ( WIN32 )
	set(GLFW_INCLUDE_PATH "E:/libs/OpenGL/include/GLFW")
	set(GLFW_LIBRARY "E:/libs/OpenGL/lib/glfw3.lib")
	set(GLEW_INCLUDE_PATH "E:/libs/OpenGL/include")
	set(GLEW_LIBRARY "E:/libs/OpenGL/lib/glew32.lib")
	set(GLEW_LIBRARY_DEBUG "E:/libs/OpenGL/lib/glew32d.lib")
	set(GLM_INCLUDE_PATH "E:/libs/OpenGL/include")
	set(GLAD_INCLUDE_PATH 
		"E:/libs/OpenGL/include/glad"
		"E:/libs/OpenGL/include")
	include_directories(${GLAD_INCLUDE_PATH}
						${GLFW_INCLUDE_PATH}
						${GLEW_INCLUDE_PATH}
						${GLM_INCLUDE_PATH})
	add_definitions(
		#-DTW_STATIC
		#-DTW_NO_LIB_PRAGMA
		#-DTW_NO_DIRECT3D
		-DGLEW_STATIC
		-DNANOGUI_GLAD    #nanogui
		#-D_CRT_SECURE_NO_WARNINGS
	)
	
#	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
endif()

# cuda
find_package(CUDA)
set(CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS}
#    -O3 
	-gencode=arch=compute_61,code=\"sm_61,compute_61\"
    )

# pointgrey camera sdk
set(PTGREY_SDK_INCLUDE_DIR "E:/libs/Spinnaker/include")
set(PTGREY_SDK_LIBRARY_DEBUG "E:/libs/Spinnaker/lib64/vs2015/Spinnakerd_v140.lib")
set(PTGREY_SDK_LIBRARY_RELEASE "E:/libs/Spinnaker/lib64/vs2015/Spinnaker_v140.lib")
include_directories(${PTGREY_SDK_INCLUDE_DIR})

# XIMEA camera
set(XIMEA_SDK_INCLUDE_DIR "C:/XIMEA/API")
set(XIMEA_SDK_LIBRARY_DEBUG "C:/XIMEA/API/x64/xiapi64.lib")
set(XIMEA_SDK_LIBRARY_RELEASE "C:/XIMEA/API/x64/xiapi64.lib")
include_directories(${XIMEA_SDK_INCLUDE_DIR})

#ceres solver
set(CERES_INCLUDE_DIR "E:/libs/ceres_solver/include")
set (CERES_LIBS
    debug "E:/libs/ceres_solver/lib/ceresd.lib"
    debug "E:/libs/ceres_solver/lib/libglog_staticd.lib"
    optimized "E:/libs/ceres_solver/lib/ceres.lib"
    optimized "E:/libs/ceres_solver/lib/libglog_static.lib"
)
include_directories(${CERES_INCLUDE_DIR}
					${CERES_INCLUDE_DIR}/ceres
					${CERES_INCLUDE_DIR}/glog)

set(NANOGUI_INCLUDE_DIR 
	"E:/libs/nanogui/include"
	"E:/libs/nanogui/include/coro"
	"E:/libs/nanogui/include/nanovg"
	)
set(NANOGUI_LIBS 
	debug "E:/libs/nanogui/lib/nanoguid.lib"
	optimized "E:/libs/nanogui/lib/nanogui.lib")
include_directories(${NANOGUI_INCLUDE_DIR})

					
set(MY_HEADER_FILES
    GenCameraDriver/Exceptions.h
    GenCameraDriver/helper_cuda.h
    GenCameraDriver/helper_string.h
    GenCameraDriver/NPPJpegCoder.h
    GenCameraDriver/GenCameraDriver.h
    GenCameraDriver/RealCameraDriver.h
    GenCameraDriver/PointGrey/PointGreyCamera.h
    GenCameraDriver/XIMEA/XIMEACamera.h
	GenCameraDriver/FileCamera/FileCamera.h
	
	GimUtil/GimUtil.h
	#serial
	Serial/Serial.h
	#cost
	GimExec/GimExec.h
	#common
	Common/Common.hpp
	#RobustStitcher
	RobustStitcher/FeatureMatch.h
	RobustStitcher/CameraParamEstimator.h
    RobustStitcher/Compositor.h
    RobustStitcher/BundleAdjustment.h
	#OpenGL
	OpenGL/shader_m.h
	OpenGL/glad.c
	OpenGL/Display.h
	
	GimPanorama.hpp
	define.h
	YOLOTracker.hpp
	GimCamera.hpp
)

set(MY_SOURCE_FILES 
    GenCameraDriver/NPPJpegCoder.cpp
    GenCameraDriver/GenCameraDriver.cpp
	GenCameraDriver/GenCameraDriverImpl.cpp
    GenCameraDriver/RealCameraDriver.cpp
	GenCameraDriver/PointGrey/PointGreyCamera.cpp
    GenCameraDriver/XIMEA/XIMEACamera.cpp
	GenCameraDriver/FileCamera/FileCamera.cpp
	GimUtil/GimUtil.cpp
	#serial
	Serial/Serial.cpp
	#cost
	GimExec/GimExec.cpp
	#RobustStitcher
	RobustStitcher/FeatureMatch.cpp
	RobustStitcher/CameraParamEstimator.cpp
    RobustStitcher/Compositor.cpp
    RobustStitcher/BundleAdjustment.cpp
	#OpenGL
	OpenGL/display.cpp

	GimCamera.cpp
	GimPanorama.cpp
    # main function
    main.cpp
	#example_icons.cpp
)

source_group("PointGrey" FILES
    PointGrey/PointGreyCamera.h
    PointGrey/PointGreyCamera.cpp 
)

source_group("XIMEA" FILES
    XIMEA/XIMEACamera.h
    XIMEA/XIMEACamera.cpp
)
source_group("RobustStitcher" FILES
    RobustStitcher/FeatureMatch.cpp
	RobustStitcher/CameraParamEstimator.cpp
    RobustStitcher/Compositor.cpp
    RobustStitcher/BundleAdjustment.cpp
	RobustStitcher/FeatureMatch.h
	RobustStitcher/CameraParamEstimator.h
    RobustStitcher/Compositor.h
    RobustStitcher/BundleAdjustment.h
)


set(YOLO_LIBS
    E:/libs/cuDNN/lib/x64/cudnn.lib
	E:/libs/YOLO/lib/pthreadVC2.lib
	debug E:/libs/YOLO/lib/Debug/yolo_libd.lib
	optimized E:/libs/YOLO/lib/Release/yolo_lib.lib
)

set(YOLOTracker_LIBS
	debug E:/libs/Tracker/lib/Debug/yolotracker_libd.lib
	optimized E:/libs/Tracker/lib/Release/yolotracker_lib.lib
)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/GenCameraDriver
	${PROJECT_SOURCE_DIR}/GenCameraDriver/XIMEA
	${PROJECT_SOURCE_DIR}/GenCameraDriver/PointGrey
	${PROJECT_SOURCE_DIR}/GenCameraDriver/FileCamera
	${PROJECT_SOURCE_DIR}/Serial
	${PROJECT_SOURCE_DIR}/XIMEA	
	${PROJECT_SOURCE_DIR}/PointGrey		
	${PROJECT_SOURCE_DIR}/GimExec
	${PROJECT_SOURCE_DIR}/Common
	${PROJECT_SOURCE_DIR}/GimUtil
	${PROJECT_SOURCE_DIR}/RobustStitcher
	${PROJECT_SOURCE_DIR}/OpenGL
	E:/libs/YOLO/include
    E:/libs/cuDNN/include
	E:/libs/Tracker/include
)

cuda_add_executable(${name}
    ${MY_HEADER_FILES}
    ${MY_SOURCE_FILES}
	YOLO/YOLODetector.hpp
	YOLOTracker.hpp
)

add_definitions(-DYOLO_DETECTOR_EXPORTS)
add_definitions(-DYOLODLL_EXPORTS)
add_definitions(-DYOLO_TRACKER_EXPORTS)
add_definitions(-DGPU)
add_definitions(-DCUDNN)
add_definitions(-DNOMINMAX)
#Tracker
add_definitions(-DUSE_OCV_UKF)
add_definitions(-DUSE_OCV_KCF)
add_definitions(-DUSE_OCV_BGFG)

if(NOT CMAKE_DEBUG_POSTFIX)
  set(CMAKE_DEBUG_POSTFIX d)
endif()

target_link_libraries(${name}
    ${OpenCV_LIBS}
    ${CUDA_npp_LIBRARY}
    ${CUDA_nppc_LIBRARY}
	${CUDA_LIBRARIES}
	${CUDA_CUBLAS_LIBRARIES}
	${CUDA_curand_LIBRARY}
	${CERES_LIBS}
	${GLFW_LIBRARY}
    ${PTGREY_SDK_LIBRARY_DEBUG} 
	${XIMEA_SDK_LIBRARY_DEBUG} 
	${YOLO_LIBS}
	${YOLOTracker_LIBS}
    ${PTGREY_SDK_LIBRARY_RELEASE} 
	${XIMEA_SDK_LIBRARY_RELEASE} 
	${NANOGUI_LIBS}
)