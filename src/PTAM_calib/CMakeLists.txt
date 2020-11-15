cmake_minimum_required(VERSION 3.10)
project(PTAM)

set(PTAM_CALIB_SRC CalibCornerPatch.cc CalibImage.cc ../PTAM_lib/ATANCamera.cc)

add_executable(CameraCalibrator CameraCalibrator.cc ../VideoSource.cpp ${PTAM_CALIB_SRC} ../GLWindowMenu.cc ../GLWindow2.cc )
target_link_libraries(CameraCalibrator GVars3 cvd GL lapack ${OpenCV_LIBS} opencv_videoio)

add_executable(VideoCameraCalibrator VideoCameraCalibrator.cpp ../VideoSource.cpp ${PTAM_CALIB_SRC} ../GLWindowMenu.cc ../GLWindow2.cc )
target_link_libraries(VideoCameraCalibrator GVars3 cvd GL lapack ${OpenCV_LIBS} opencv_videoio)