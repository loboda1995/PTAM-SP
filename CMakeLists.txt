cmake_minimum_required(VERSION 3.10)
project(PTAM-SP)

set(CMAKE_CXX_STANDARD 17)

find_package( OpenCV 4 REQUIRED )
find_package( DBoW2 QUIET PATHS /home/luka/Sources/DBoW2/install)
find_package( OpenGL REQUIRED )

include_directories(include include/PTAM_lib include/PTAM_calib)

set(PTAM_SP_LIB_SRC
        ${CMAKE_SOURCE_DIR}/src/PTAM-SP-lib/ATANCamera.cpp
        ${CMAKE_SOURCE_DIR}/src/PTAM-SP-lib/Bundle.cpp
        ${CMAKE_SOURCE_DIR}/src/PTAM-SP-lib/KeyFrame.cpp
        ${CMAKE_SOURCE_DIR}/src/PTAM-SP-lib/Map.cpp
        ${CMAKE_SOURCE_DIR}/src/PTAM-SP-lib/MapMaker.cpp
        ${CMAKE_SOURCE_DIR}/src/PTAM-SP-lib/MapPoint.cpp
        ${CMAKE_SOURCE_DIR}/src/PTAM-SP-lib/PatchFinder.cpp
        ${CMAKE_SOURCE_DIR}/src/PTAM-SP-lib/ShiTomasi.cpp
        ${CMAKE_SOURCE_DIR}/src/PTAM-SP-lib/SmallBlurryImage.cpp
        ${CMAKE_SOURCE_DIR}/src/PTAM-SP-lib/Tracker.cpp)

SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-Wno-enum-compare -march=core2 -msse3")
add_definitions(-DCVD_HAVE_XMMINTRIN=1)

# Original PTAM program with GUI
add_subdirectory(src/PTAM-SP)
# Calibration program
add_subdirectory(src/PTAM-SP-calib)
# Headless implementation for testing
add_subdirectory(src/PTAM-SP-headless)
# Program used to install models used by PTAM-SP
add_subdirectory(src/PTAM-SP-model-installer)
