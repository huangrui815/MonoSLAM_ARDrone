cmake_minimum_required(VERSION 2.8)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#OPENCV and wxWidgets support
find_package( OpenCV REQUIRED )
SET(wxWidgets_USE_LIBS base core gl net)
find_package( wxWidgets )
include( "${wxWidgets_USE_FILE}" )
include_directories(
/home/rui/rosbuild_ws/myROS/libvisualslam/src
/home/rui/rosbuild_ws/myROS/monoslam_ros_test/src
/home/rui/src/pba/src
)

# ------------------- add dynamic reconfigure api ------------------------------------
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#set movisslam header and source files
set(MOVISSLAM_APP_SOURCE_FILES
	#src/app/ARDrone.cpp
	#src/app/ARDroneMain.cpp
	#src/app/ARVideo.cpp
	#src/app/MiniCamMain.cpp
	src/app/APP_MoSLAMThread.cpp
	src/app/APP_MyApp.cpp
	src/app/APP_OfflineMain.cpp
	src/app/APP_SynObj.cpp
	src/app/APP_USBCamMain.cpp
	src/app/APP_EstimationNode.cpp
	src/app/APP_ARDroneROSMain.cpp
	src/app/SL_Bundler.cpp
	src/app/SL_CameraTracker.cpp
	src/app/SL_GlobParam.cpp
	src/app/SL_MapMaker.cpp
	src/app/SL_MoSLAM.cpp
	src/app/SL_MoSLAMRobustBA.cpp
	src/app/SL_PointRegistration.cpp
	src/app/SL_Relocalization.cpp
	src/app/SL_PBA.cpp
)
set(MOVISSLAM_APP_HEADER_FILES
	#src/app/ARDrone.h
	#src/app/ARVideo.h
	src/app/APP_MoSLAMThread.h
	src/app/APP_MyApp.h
	src/app/APP_SynObj.h
	src/app/APP_EstimationNode.h
	#src/app/NavData.h
	src/app/SL_Bundler.h
	src/app/SL_CameraTracker.h
	src/app/SL_GlobParam.h
	src/app/SL_MapMaker.h
	src/app/SL_MoSLAM.h
	src/app/SL_MoSLAMRobustBA.h
	src/app/SL_PointRegistration.h
	src/app/SL_Relocalization.h
	src/app/SL_PBA.h
)

set(MOVISSLAM_GUI_SOURCE_FILES
	src/gui/GLHelper.cpp
	src/gui/GLImagePane.cpp
	src/gui/GLImageWnd.cpp
	src/gui/GLScenePane.cpp
	src/gui/GLScenePaneHelper.cpp
	src/gui/GLSceneWnd.cpp
	src/gui/GLTrackballPane.cpp
	src/gui/MainDialog.cpp
	src/gui/trackball.cpp
)

set (MOVISSLAM_GUI_HEADER_FILES
	src/gui/GLHelper.h
	src/gui/GLImagePane.h
	src/gui/GLImageWnd.h
	src/gui/GLScenePane.h
	src/gui/GLScenePaneHelper.h
	src/gui/GLSceneWnd.h
	src/gui/GLTrackballPane.h
	src/gui/MainDialog.h
	src/gui/trackball.h
)

set(MOVISSLAM_SLAM_SOURCE_FILES
	src/slam/SL_CameraPose.cpp
	src/slam/SL_FeatureMatching.cpp
	src/slam/SL_FeaturePoint.cpp
	src/slam/SL_GlobalPoseEstimation.cpp
	src/slam/SL_IntraCamPose.cpp
	src/slam/SL_KalmanFilter.cpp
	src/slam/SL_KeyFrame.cpp
	src/slam/SL_MapPoint.cpp
	src/slam/SL_MoSLAMHelper.cpp
	src/slam/SL_NCCBlock.cpp
	src/slam/SL_SLAMHelper.cpp
	src/slam/SL_SolvePnP.cpp
)

set(MOVISSLAM_SLAM_HEADER_FILES
	src/slam/SL_CameraPose.h
	src/slam/SL_Define.h
	src/slam/SL_FeatureMatching.h
	src/slam/SL_FeaturePoint.h
	src/slam/SL_GlobalPoseEstimation.h
	src/slam/SL_IntraCamPose.h
	src/slam/SL_KalmanFilter.h
	src/slam/SL_KeyFrame.h
	src/slam/SL_MapPoint.h
	src/slam/SL_MoSLAMHelper.h
	src/slam/SL_NCCBlock.h
	src/slam/SL_SLAMHelper.h
	src/slam/SL_SolvePnP.h
)

set(MOVISSLAM_TRACKING_FILES
	src/tracking/SL_BaseKLTTracker.h
	src/tracking/SL_FeatureTracker.h
	src/tracking/SL_GPUKLTTracker.h
#	src/tracking/SL_OpenCVKLTTracker.h
	src/tracking/SL_Track2D.h
	src/tracking/SL_FeatureTracker.cpp
	src/tracking/SL_GPUKLTTracker.cpp
#	src/tracking/SL_OpenCVKLTTracker.cpp
	src/tracking/SL_Track2D.cpp
	src/tracking/CVKLTTracker.h
	src/tracking/CVKLTTracker.cpp
)

set(MOVISSLAM_CGKLT_FILES
	src/tracking/CGKLT/v3d_exception.h
	src/tracking/CGKLT/v3d_gpubase.h
	src/tracking/CGKLT/v3d_gpuflow.h
	src/tracking/CGKLT/v3d_gpuklt.h
	src/tracking/CGKLT/v3d_gpupyramid.h
	src/tracking/CGKLT/v3d_gpuundistort.h
	src/tracking/CGKLT/v3d_image.h
	src/tracking/CGKLT/v3d_linear.h
	src/tracking/CGKLT/v3d_linearbase.h
	src/tracking/CGKLT/v3d_serialization.h
	src/tracking/CGKLT/v3d_timer.h
	src/tracking/CGKLT/v3d_utilities.h
	src/tracking/CGKLT/v3d_gpubase.cpp
	src/tracking/CGKLT/v3d_gpuflow.cpp
	src/tracking/CGKLT/v3d_gpuklt.cpp
	src/tracking/CGKLT/v3d_gpupyramid.cpp
	src/tracking/CGKLT/v3d_gpuundistort.cpp
	src/tracking/CGKLT/v3d_image.cpp
)

set(MOVISSLAM_CONTROL_FILES
    src/stateestimation/Drone_Filter.h
    src/stateestimation/Drone_Filter.cpp
    src/stateestimation/Drone_FilterHelper.h
    src/stateestimation/Drone_FilterHelper.cpp
    src/stateestimation/Drone_Param.h
    src/stateestimation/Drone_ScaleEstimator.h
    src/stateestimation/Drone_ScaleEstimator.cpp
    src/stateestimation/Drone_MapViewPane.cpp
    src/stateestimation/Drone_MapViewWnd.cpp
    src/stateestimation/Drone_ScaleFilter.h
    src/stateestimation/Drone_ScaleFilter.cpp
    
    src/ControllerNode/PIDController.h
    src/ControllerNode/PIDController.cpp
    src/ControllerNode/PositionController.h
    src/ControllerNode/PositionController.cpp
    src/ControllerNode/PathPlanner.h
    src/ControllerNode/PathPlanner.cpp
    )
    
set(MOVISSLAM_VIDEO_READER_FILES
    src/videoReader/VR_AVIReader.h
    src/videoReader/VR_AVIReader.cpp
    src/videoReader/VR_DroneCamReader.h
    src/videoReader/VR_DroneCamReader.cpp
    src/videoReader/VR_USBCamReader.h
    src/videoReader/VR_USBCamReader.cpp
)

set(MoSLAM_LIB
	/home/rui/rosbuild_ws/myROS/libvisualslam/Debug/libVisualSLAM.a
	/home/rui/src/pba/bin/libpba_no_gpu.a
	lapack
	blas
	swscale
	avformat
	avutil
	avcodec
	Cg	
	GL
	GLEW
	CgGL
	glut 
	GLU
)

add_definitions(-DV3DLIB_GPGPU_ENABLE_CG -DV3DLIB_ENABLE_GPGPU )

rosbuild_add_executable(monoslam_ros_test ${MOVISSLAM_APP_SOURCE_FILES}
									  ${MOVISSLAM_GUI_SOURCE_FILES}
									  ${MOVISSLAM_SLAM_SOURCE_FILES}
									  ${MOVISSLAM_APP_HEADER_FILES}
									  ${MOVISSLAM_GUI_HEADER_FILES}
									  ${MOVISSLAM_SLAM_HEADER_FILES}
									  ${MOVISSLAM_TRACKING_FILES}
									  ${MOVISSLAM_CGKLT_FILES}
									  ${MOVISSLAM_VIDEO_READER_FILES}
									  ${MOVISSLAM_CONTROL_FILES}
)

target_link_libraries(monoslam_ros_test ${MoSLAM_LIB} ${OpenCV_LIBS} ${wxWidgets_LIBRARIES})

#boost
#rosbuild_add_boost_directories()
#rosbuild_link_boost(movisslam_ros thread)


# ------------------------- autopilot & KI -----------------------------------------
# set header ans source files
set(AUTOPILOT_SOURCE_FILES         
	src/autopilot/AUTO_Main.cpp  
	src/autopilot/AUTO_ControlNode.cpp
	src/autopilot/AUTO_DroneController.cpp
	src/autopilot/KI/KILand.cpp
	src/autopilot/KI/KIAutoInit.cpp
	src/autopilot/KI/KIMyAutoInit.cpp
	src/autopilot/KI/KIFlyTo.cpp
	src/autopilot/KI/KIScaleInit.cpp
)
set(AUTOPILOT_HEADER_FILES        
	src/autopilot/AUTO_ControlNode.h
	src/autopilot/AUTO_DroneController.h
	src/autopilot/KI/KILand.h
	src/autopilot/KI/KIAutoInit.h
	src/autopilot/KI/KIMyAutoInit.h
	src/autopilot/KI/KIFlyTo.h
	src/autopilot/KI/KIScaleInit.h
	src/autopilot/KI/KIProcedure.h
)

# build!
rosbuild_add_executable(drone_autopilot ${AUTOPILOT_SOURCE_FILES} ${AUTOPILOT_HEADER_FILES})
# ---------------------------- Messages & Services --------------------------------------------------
#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()