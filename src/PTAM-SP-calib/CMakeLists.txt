cmake_minimum_required(VERSION 3.10)
project(PTAM-SP)

set(PTAM_SP_CALIB_SRC CalibCornerPatch.cpp CalibImage.cpp ../PTAM-SP-lib/ATANCamera.cpp)

add_executable(CameraCalibrator CameraCalibrator.cpp ../VideoSource.cpp ${PTAM_SP_CALIB_SRC} ../GLWindowMenu.cpp ../GLWindow2.cpp)
target_link_libraries(CameraCalibrator GVars3 cvd GL lapack ${OpenCV_LIBS} opencv_videoio)

add_executable(VideoCameraCalibrator VideoCameraCalibrator.cpp ../VideoSource.cpp ${PTAM_SP_CALIB_SRC} ../GLWindowMenu.cpp ../GLWindow2.cpp)
target_link_libraries(VideoCameraCalibrator GVars3 cvd GL lapack ${OpenCV_LIBS} opencv_videoio)