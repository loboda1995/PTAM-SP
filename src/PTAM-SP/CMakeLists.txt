cmake_minimum_required(VERSION 3.10)
project(PTAM-SP)

add_executable(PTAM-SP main.cpp MapViewer.cpp ../System.cpp ../VideoSource.cpp ../GLWindowMenu.cpp ../GLWindow2.cpp ${PTAM_SP_LIB_SRC})
target_include_directories(PTAM-SP PUBLIC . /usr/local/include/eigen3 ${DBoW2_INCLUDE_DIRS})
target_link_libraries(PTAM-SP GVars3 cvd GL lapack ${OpenCV_LIBS} opencv_videoio ${DBoW2_LIBS})
