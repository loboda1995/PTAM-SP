// Copyright 2008 Isis Innovation Limited
#include "System.h"
#include "OpenGL.h"
#include <gvars3/instances.h>
#include <opencv2/imgproc.hpp>
#include <LevelHelpers.h>
#include "ATANCamera.h"
#include "MapMaker.h"
#include "TrackerData.h"

using namespace CVD;
using namespace GVars3;


System::System(const std::string &deviceFolder, const std::string &testFolder, MapMaker::OperationMode mode, const std::string &model) {

    std::string filename = testFolder + "/video_nomarker.avi";
    mVideoSource.Open(filename);

    mGLWindow = new GLWindow2(mVideoSource.Size(), "PTAM");
    GUI.RegisterCommand("exit", GUICommandCallBack, this);
    GUI.RegisterCommand("quit", GUICommandCallBack, this);
    if (mVideoSource.IsFromRecording())
        GUI.RegisterCommand("ResetRecording", GUICommandCallBack, this);

    mimFrameBW.resize(mVideoSource.Size());
    mimFrameRGB.resize(mVideoSource.Size());
    // First, check if the camera is calibrated.
    // If not, we need to run the calibration widget.
    Vector<NUMTRACKERCAMPARAMETERS> vTest;

    vTest = GV3::get<Vector<NUMTRACKERCAMPARAMETERS> >("Camera.Parameters", ATANCamera::mvDefaultParams, HIDDEN);
    mpCamera = new ATANCamera("Camera");
    mpCamera->SetImageSize(mVideoSource.Size());

    mpMap = new Map;
    mpMapMaker = new MapMaker(*mpMap, *mpCamera, stats, deviceFolder, mode);
    mpTracker = new Tracker(mVideoSource.Size(), *mpCamera, *mpMap, *mpMapMaker, mVideoSource, stats, testFolder+"/PTAM_track.txt");
    mpMapViewer = new MapViewer(*mpMap, *mGLWindow);

    if (mode == MapMaker::MM_MODE_SINGLE_MODEL) {
        SE3<> tmp;
        mpMapMaker->LoadModelFromFolder(model, mVideoSource.Size(), tmp);
    } else if (mode == MapMaker::MM_MODE_FROM_INSTALLER) {
        SE3<> tmp;
        mpMapMaker->LoadMapFromInstaller(model, mVideoSource.Size(), tmp);
    }

    GUI.ParseLine("GLWindow.AddMenu Menu Menu");
    GUI.ParseLine("Menu.ShowMenu Root");
    GUI.ParseLine("Menu.AddMenuButton Root Reset Reset Root");
    if (mVideoSource.IsFromRecording())
        GUI.ParseLine("Menu.AddMenuButton Root \"Restart Rec\" ResetRecording Root");
    GUI.ParseLine("DrawMap=0");
    GUI.ParseLine("Menu.AddMenuToggle Root \"View Map\" DrawMap Root");


    mbDone = false;
};

void System::PrintStats() {
    std::cout << std::endl << std::endl;
    std::cout << "#############   T R A C K I N G   S T A T S   #############" << std::endl;
    std::cout << "Avg. tracking time: \t\t\t" << stats.GetAvgTrackingTime() << " ms" << std::endl;
    std::cout << "Models loaded: \t\t\t\t\t" << stats.GetLoadedModels() << std::endl;
    std::cout << "Successful relocalizations: \t" << stats.GetSuccessfulRelocs() << std::endl;
    std::cout << "Number of key frames: \t\t\t" << stats.GetNumOfStartKeyFrames() << "(start)  -  " << stats.GetNumOfEndKeyFrames() << "(end)" << std::endl;
    std::cout << "Number of points: \t\t\t\t" << stats.GetNumOfStartPoints() << "(start)  -  " << stats.GetNumOfEndPoints() << "(end)" << std::endl;
    std::cout << "###########################################################" << std::endl;
}

void System::Run() {
    int lastFrameN = -1;
    cv::Mat imBW, imRGB;

    while (!mbDone) {
        if (!mVideoSource.GetAndFillFrameBWandRGB(imBW, imRGB)) {
            mbDone = true;
            break;
        }

        if (mbRestartRecording) {
            mbRestartRecording = false;
            mVideoSource.RestartVideoFile();
        }

        if (lastFrameN == mVideoSource.GetFrameN()) {
            continue;
        }
        lastFrameN = mVideoSource.GetFrameN();


        mGLWindow->SetupViewport();
        mGLWindow->SetupVideoOrtho();
        mGLWindow->SetupVideoRasterPosAndZoom();

        cv::Mat cvd_image(imRGB.rows, imRGB.cols,CV_8UC3, mimFrameRGB.data());
        cvtColor(imRGB, cvd_image, cv::COLOR_BGR2RGB);
        glDrawPixels(mimFrameRGB);


        auto start = std::chrono::high_resolution_clock::now();
        mpTracker->TrackFrame(imBW);
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        stats.AddTrackTime(duration.count());

        static gvar3<int> gvnDrawMap("DrawMap", 0, HIDDEN | SILENT);
        bool bDrawMap = mpMap->IsGood() && *gvnDrawMap;
        if (bDrawMap) {
            mpMapViewer->DrawMap(mpTracker->GetCurrentPose());
        }


        std::string sCaption;
        if (bDrawMap)
            sCaption = mpMapViewer->GetMessageForUser();
        else
            sCaption = mpTracker->GetMessageForUser();
        mGLWindow->DrawCaption(sCaption);
        mGLWindow->DrawMenus();
        mGLWindow->swap_buffers();
        mGLWindow->HandlePendingEvents();
    }
    stats.SetEndStats(mpMap->vpKeyFrames.size(), mpMap->vpPoints.size());
    PrintStats();
}

void System::GUICommandCallBack(void *ptr, std::string sCommand, std::string sParams) {
    if (sCommand == "quit" || sCommand == "exit")
        static_cast<System *>(ptr)->mbDone = true;
    if (sCommand == "ResetRecording")
        static_cast<System *>(ptr)->mbRestartRecording = true;
}








