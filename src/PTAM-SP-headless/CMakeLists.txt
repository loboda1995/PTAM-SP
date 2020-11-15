cmake_minimum_required(VERSION 3.10)
project(PTAM-SP)

add_executable(HeadlessPTAM-SP headless.cpp HeadlessSystem.cpp ../VideoSource.cpp ${PTAM_SP_LIB_SRC})
target_include_directories(HeadlessPTAM-SP PUBLIC include src/PTAM_headless /usr/local/include/eigen3 ${DBoW2_INCLUDE_DIRS})
target_link_libraries(HeadlessPTAM-SP GVars3 cvd lapack ${OpenCV_LIBS} opencv_videoio ${DBoW2_LIBS})
