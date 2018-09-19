# CMakeLists file of Multiscale gigapixel video
#
# Author: Shane Yuan
# Date: May 23, 2017
#

set(projname "RobustStitcher")

# ceres solver for bundle adjustment
add_definitions(
    -DGOOGLE_GLOG_DLL_DECL=
)
include_directories(
    E:/libs/ceres_solver/include
    E:/libs/ceres_solver/include/ceres
    E:/libs/ceres_solver/include/glog
)
set (CERES_LIBS
    debug "E:/libs/ceres_solver/lib/ceresd.lib"
    debug "E:/libs/ceres_solver/lib/libglog_staticd.lib"
    optimized "E:/libs/ceres_solver/lib/ceres.lib"
    optimized "E:/libs/ceres_solver/lib/libglog_static.lib"
)

set(MY_HEADER_FILE
    ${PROJECT_SOURCE_DIR}/Common/Common.hpp
    FeatureMatch.h
	CameraParamEstimator.h
    Compositor.h
    BundleAdjustment.h
)

set(MY_SOURCE_FILE 
    main.cpp
    FeatureMatch.cpp
	CameraParamEstimator.cpp
	Compositor.cpp
    BundleAdjustment.cpp
)

cuda_add_executable( ${projname} 
    ${MY_HEADER_FILE} 
    ${MY_SOURCE_FILE}
)

target_link_libraries( ${projname} 
    ${OpenCV_LIBS}
    ${CERES_LIBS} 
)
