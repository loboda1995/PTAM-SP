cmake_minimum_required(VERSION 3.10)
project(PTAM)

add_executable(HeadlessPTAM headless.cpp HeadlessSystem.cpp ../VideoSource.cpp ${PTAM_LIB_SRC})
target_include_directories(HeadlessPTAM PUBLIC include src/PTAM_headless /usr/local/include/eigen3 ${DBoW2_INCLUDE_DIRS})
target_link_libraries(HeadlessPTAM GVars3 cvd lapack ${OpenCV_LIBS} opencv_videoio ${DBoW2_LIBS})
