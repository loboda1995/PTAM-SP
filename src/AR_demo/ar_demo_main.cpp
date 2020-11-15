#include <glad/glad.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include "ATANCamera.h"
#include "Tracker.h"
#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"
#include "ARDriver.h"
#include "LetterCubes.h"
#include "ModelWireframe.h"
#include "VideoDisplay.h"

#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <gvars3/instances.h>
#include <GL/glext.h>
#include <opencv2/opencv.hpp>

static unsigned int m_backgroundTextureId;
static Tracker::TrackingState mOldState = Tracker::LOADING_SCENE;
static std::chrono::time_point<std::chrono::steady_clock, std::chrono::duration<float>> lastSuccReloc;
static std::chrono::time_point<std::chrono::steady_clock, std::chrono::duration<float>> lastKeyframeAdd;
static bool showAROnly = false, showStats = false;

static void error_callback(int error, const char* description)
{
    fprintf(stderr, "Error: %s\n", description);
}

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    if (action == GLFW_PRESS && mods == GLFW_MOD_CONTROL) {
        if (key == GLFW_KEY_A)
            showAROnly = !showAROnly;
        if (key == GLFW_KEY_S)
            showStats = !showStats;
    }
}

static GLfloat* convertMatrixType(const cv::Mat& m)
{
    typedef double precision;

    cv::Size s = m.size();
    GLfloat* mGL = new GLfloat[s.width*s.height];

    for(int ix = 0; ix < s.width; ix++)
    {
        for(int iy = 0; iy < s.height; iy++)
        {
            mGL[ix*s.height + iy] = m.at<precision>(iy, ix);
        }
    }

    return mGL;
}

static void drawBackground(cv::Mat &img, int border = 0) {
    const GLfloat img_width = img.cols;
    const GLfloat img_height = img.rows;

    const GLfloat bgTextureVertices[] = { 0, 0, img_width, 0, 0, img_height, img_width, img_height };
    const GLfloat bgTextureCoords[]   = { 1, 0, 1, 1, 0, 0, 0, 1 };
    const GLfloat proj[]              = { 0, -2.f/img_width, 0, 0, -2.f/img_height, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1 };

    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glBindTexture(GL_TEXTURE_2D, m_backgroundTextureId);

    // Upload new texture data:
    if (img.channels() == 3)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img_width, img_height, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, img.data);
    else if(img.channels() == 4)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img_width, img_height, 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, img.data);
    else if (img.channels()==1)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img_width, img_height, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, img.data);


    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(proj);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, m_backgroundTextureId);

    // Update attribute values.
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);

    glVertexPointer(2, GL_FLOAT, 0, bgTextureVertices);
    glTexCoordPointer(2, GL_FLOAT, 0, bgTextureCoords);

    glColor4f(1,1,1,1);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glDisable(GL_TEXTURE_2D);

    if (border > 0) {
        glLineWidth(15);
        if (border == 1) {
            glColor3d(1, 0, 0);
        } else if (border == 2) {
            glColor3d(0, 1, 1);
        } else {
            glColor3d(0, 1, 0);
        }
        glEnable(GL_LINE_SMOOTH);
        glBegin(GL_LINE_LOOP);
        glVertex2d(0,0);
        glVertex2d(img_width, 0);
        glVertex2d(img_width, img_height);
        glVertex2d(0,img_height);
        glEnd();

        glLineWidth(1);
        glDisable(GL_LINE_SMOOTH);
    }
}

int main(void)
{
    std::string deviceFolder = "/home/luka/mag_data_demo/devices/device_Huawei_P10";
    std::string filename = "/home/luka/mag_data/recordings/statues/test_01/video_nomarker.avi";


    // PTAM driver
    cv::Mat imBW, imRGB;
    GVars3::GUI.LoadFile(deviceFolder + "/PTAM_calib.cfg");
    VideoSource mVideoSource;
    mVideoSource.Open(filename);
    auto mARDriver = new ARDriver(deviceFolder, mVideoSource.Size());


    // GLFW Window
    GLFWwindow* window;
    glfwSetErrorCallback(error_callback);
    if (!glfwInit())
        exit(EXIT_FAILURE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    const GLfloat img_width = mVideoSource.Size()[0]*0.5;
    const GLfloat img_height = mVideoSource.Size()[1]*0.5;
    window = glfwCreateWindow(img_width, img_height, "AR Demo", NULL, NULL);
    if (!window) {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }
    glfwSetKeyCallback(window, key_callback);
    glfwMakeContextCurrent(window);
    gladLoadGLLoader((GLADloadproc) glfwGetProcAddress);
    glfwSwapInterval(1);
    glfwHideWindow(window);


    // AR examples
    auto lettersExamle = LetterCubes("PTAM", 0.8f, 3000, 250);
    std::map<std::string, std::string> modelToSceneName;
    modelToSceneName["desk_PC"] = "Desk PC";
    modelToSceneName["garage"] = "Garage";
    modelToSceneName["statues"] = "Statues";
    auto wireframe = ModelWireframe("/home/luka/mag_data_demo/installers/statues/3d_model_selected_in_ref.mvs");
    std::vector<cv::Point3f> videoPos;
    videoPos.emplace_back(-0.23, 0.435, 0.395);
    videoPos.emplace_back(cv::Point3f(0.163, 0.435, 0.395));
    videoPos.emplace_back(cv::Point3f(-0.23, 0.42, 0.095));
    videoPos.emplace_back(cv::Point3f(0.163, 0.42, 0.095));
    auto videoDisplay = VideoDisplay(filename, videoPos);


    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 460");
    ImGui::StyleColorsDark();
    ImFont* pFontLarge = io.Fonts->AddFontFromFileTTF("resources/Ruda-Bold.ttf", 30);
    ImFont* pFontSmall = io.Fonts->AddFontFromFileTTF("resources/Ruda-Bold.ttf", 15);


    // Background texture
    glGenTextures(1, &m_backgroundTextureId);
    glBindTexture(GL_TEXTURE_2D, m_backgroundTextureId);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);


    // Prepare projection matrix
    float zNear = 0.01f;  // Near clipping distance
    float zFar  = 100.0f;  // Far clipping distance
    cv::Mat projection(4,4, CV_64FC1);
    mARDriver->GetCameraProjectionMat(zNear, zFar, projection, 0.5);
    auto cameraProj = convertMatrixType(projection);

    bool done = false;
    glfwShowWindow(window);
    while (!glfwWindowShouldClose(window) && !done)
    {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // PTAM tracking
        if (!mVideoSource.GetAndFillFrameBWandRGB(imBW, imRGB)) {
            done = true;
        } else {
            auto track_quality = mARDriver->TrackFrame(imBW);

            // Determine status changes
            if ((mOldState == Tracker::RELOCATING || mOldState == Tracker::LOADING_SCENE) && (track_quality == Tracker::TRACKING_GOOD || track_quality == Tracker::TRACKING_DODGY)) {
                lastSuccReloc = std::chrono::steady_clock::now();
            }
            if (track_quality == Tracker::KEYFRAME_ADDED) {
                lastKeyframeAdd = std::chrono::steady_clock::now();
            }
            mOldState = track_quality;


            // Draw background
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            if (showAROnly) {
                drawBackground(imRGB);
            } else if (track_quality == Tracker::TRACKING_BAD) {
                drawBackground(imRGB, 1);
            } else if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - lastSuccReloc).count() < 500){
                drawBackground(imRGB, 2);
            } else if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - lastKeyframeAdd).count() < 250){
                drawBackground(imRGB, 3);
            } else {
                drawBackground(imRGB);
            }

            glMatrixMode(GL_PROJECTION);
            glLoadMatrixf(cameraProj);
            glMatrixMode(GL_MODELVIEW);
            cv::Mat glViewMatrix = cv::Mat::zeros(4, 4, CV_64F);
            mARDriver->GetCameraPosGLMat(glViewMatrix);
            glLoadMatrixd(&glViewMatrix.at<double>(0, 0));


            auto current_scene = mARDriver->GetCurrentScene();
            if (modelToSceneName.find(current_scene) != modelToSceneName.end())
                current_scene = modelToSceneName[current_scene];

            // Draw AR elements
            if (track_quality == Tracker::TRACKING_GOOD || track_quality == Tracker::TRACKING_DODGY) {
                if (current_scene == "Garage")
                    lettersExamle.Show();
                if (current_scene == "Statues")
                    wireframe.Show();
                if (current_scene == "Desk PC")
                    videoDisplay.Show();
            }

            // GUI toggled with Ctrl+A
            if (!showAROnly) {
                bool tmp;
                ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 0.3f));
                ImGui::Begin("scene_title", &tmp, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize  | ImGuiWindowFlags_AlwaysAutoResize);
                ImGui::PushFont(pFontLarge);

                if (current_scene.empty() || track_quality == Tracker::LOADING_SCENE) {
                    ImGui::TextColored(ImVec4(0, 1, 1, 1), "Recognising Scene");
                } else if (track_quality == Tracker::TRACKING_DODGY) {
                    ImGui::TextColored(ImVec4(1, 1, 0, 1), current_scene.c_str());
                } else if (track_quality == Tracker::TRACKING_BAD) {
                    ImGui::TextColored(ImVec4(1, 0, 0, 1), current_scene.c_str());
                } else if (track_quality == Tracker::RELOCATING) {
                    ImGui::TextColored(ImVec4(0, 1, 1, 1), (current_scene+" (relocating)").c_str());
                } else {
                    ImGui::TextColored(ImVec4(0, 1, 0, 1), current_scene.c_str());
                }
                ImGui::PopFont();
                ImGui::PopStyleColor();
                ImGui::SetWindowPos(ImVec2(img_width/2 - ImGui::GetWindowWidth()/2, 10));
                ImGui::End();
                // PTAM stats toggled with Ctrl+S
                if (showStats) {
                    bool tmp;
                    ImGui::SetNextWindowPos(ImVec2(10, 10));
                    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 0.3f));
                    ImGui::Begin("stats", &tmp, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize  | ImGuiWindowFlags_AlwaysAutoResize);
                    ImGui::PushFont(pFontSmall);
                    ImGui::Text("Avg. track time: %.2f ms", mARDriver->GetAvgTrack());
                    ImGui::Text("KeyFrames: %d", mARDriver->GetNKeyframes());
                    ImGui::Text("Map poins: %d", mARDriver->GetNPoints());
                    ImGui::Text("Successful relocalizations: %d", mARDriver->GetNRelocs());

                    ImGui::PopStyleColor();
                    ImGui::PopFont();
                    ImGui::End();
                }

            }
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    exit(EXIT_SUCCESS);
}