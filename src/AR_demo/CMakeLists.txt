cmake_minimum_required(VERSION 3.10)
project(PTAM)

set(IMGUI_SOURCE_ROOT ${CMAKE_SOURCE_DIR}/include/imgui)
set(IMGUI_SOURCES
        ${IMGUI_SOURCE_ROOT}/imconfig.h
        ${IMGUI_SOURCE_ROOT}/imgui_demo.cpp
        ${IMGUI_SOURCE_ROOT}/imgui_draw.cpp
        ${IMGUI_SOURCE_ROOT}/imgui_impl_glfw.cpp
        ${IMGUI_SOURCE_ROOT}/imgui_impl_glfw.h
        ${IMGUI_SOURCE_ROOT}/imgui_impl_opengl3.cpp
        ${IMGUI_SOURCE_ROOT}/imgui_impl_opengl3.h
        ${IMGUI_SOURCE_ROOT}/imgui_internal.h
        ${IMGUI_SOURCE_ROOT}/imgui_widgets.cpp
        ${IMGUI_SOURCE_ROOT}/imgui.cpp
        ${IMGUI_SOURCE_ROOT}/imgui.h
        ${IMGUI_SOURCE_ROOT}/imstb_rectpack.h
        ${IMGUI_SOURCE_ROOT}/imstb_textedit.h
        ${IMGUI_SOURCE_ROOT}/imstb_truetype.h)

add_executable(ARDemo ar_demo_main.cpp ARDriver.cpp ARElements.cpp ARElements.cpp ARElements.h LetterCubes.cpp LetterCubes.h ModelWireframe.cpp ModelWireframe.h VideoDisplay.cpp VideoDisplay.h
        ../glad.c ../VideoSource.cpp
        ${PTAM_LIB_SRC} ${IMGUI_SOURCES}
)
target_include_directories(ARDemo PUBLIC . /usr/local/include/eigen3 ${DBoW2_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})
target_link_libraries(ARDemo glfw GVars3 cvd lapack ${OPENGL_gl_LIBRARY} ${DBoW2_LIBS} ${OpenCV_LIBS} opencv_videoio OpenMP::OpenMP_CXX ${OpenMVS_LIBRARIES})