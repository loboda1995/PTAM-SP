cmake_minimum_required(VERSION 3.10)
project(PTAM)

add_executable(PTAM main.cc MapViewer.cc ../System.cc ../VideoSource.cpp ../GLWindowMenu.cc ../GLWindow2.cc ${PTAM_LIB_SRC})
target_include_directories(PTAM PUBLIC . /usr/local/include/eigen3 ${DBoW2_INCLUDE_DIRS})
target_link_libraries(PTAM GVars3 cvd GL lapack ${OpenCV_LIBS} opencv_videoio ${DBoW2_LIBS})
