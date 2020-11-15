cmake_minimum_required(VERSION 3.10)
project(PTAM)

add_executable(ModelInstaller model_installer.cpp ../VideoSource.cpp ${PTAM_LIB_SRC})
target_include_directories(ModelInstaller PUBLIC /usr/local/include/eigen3 ${DBoW2_INCLUDE_DIRS})
target_link_libraries(ModelInstaller GVars3 cvd lapack ${OpenCV_LIBS} opencv_videoio ${DBoW2_LIBS})