cmake_minimum_required(VERSION 3.10)
project(PTAM)

set(CMAKE_CXX_STANDARD 17)

find_package( OpenCV 4 REQUIRED )
find_package( DBoW2 QUIET PATHS /home/luka/Sources/DBoW2/install)
find_package( glfw3 3.3 REQUIRED )
find_package( OpenGL REQUIRED )

find_package( CUDA REQUIRED )
find_package( OpenMP REQUIRED )
find_package( OpenMVS REQUIRED )

include_directories(include include/PTAM_lib include/PTAM_calib)

set(PTAM_LIB_SRC
        ${CMAKE_SOURCE_DIR}/src/PTAM_lib/ATANCamera.cc
        ${CMAKE_SOURCE_DIR}/src/PTAM_lib/Bundle.cc
        ${CMAKE_SOURCE_DIR}/src/PTAM_lib/KeyFrame.cc
        ${CMAKE_SOURCE_DIR}/src/PTAM_lib/Map.cc
        ${CMAKE_SOURCE_DIR}/src/PTAM_lib/MapMaker.cc
        ${CMAKE_SOURCE_DIR}/src/PTAM_lib/MapPoint.cc
        ${CMAKE_SOURCE_DIR}/src/PTAM_lib/PatchFinder.cc
        ${CMAKE_SOURCE_DIR}/src/PTAM_lib/ShiTomasi.cc
        ${CMAKE_SOURCE_DIR}/src/PTAM_lib/SmallBlurryImage.cc
        ${CMAKE_SOURCE_DIR}/src/PTAM_lib/Tracker.cc)

SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-Wno-enum-compare -march=core2 -msse3")
add_definitions(-DCVD_HAVE_XMMINTRIN=1)

# Original PTAM program with GUI
add_subdirectory(src/PTAM)
# Calibration program
add_subdirectory(src/PTAM_calib)
# Headless implementation for testing
add_subdirectory(src/PTAM_headless)
# Program used to install models used by PTAM-SP
add_subdirectory(src/PTAM_model_installer)
# Demo AR application
add_subdirectory(src/AR_demo)
