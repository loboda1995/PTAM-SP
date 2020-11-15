// Copyright 2008 Isis Innovation Limited
#include "HeadlessSystem.h"
#include <gvars3/instances.h>
#include "ATANCamera.h"
#include "MapMaker.h"

using namespace CVD;
using namespace GVars3;


HeadlessSystem::HeadlessSystem(const std::string &deviceFolder, const std::string &testFolder, const std::string &pathFile, MapMaker::OperationMode mode, const std::string &model) {

    std::string filename = testFolder+"/video_nomarker.avi";
    mVideoSource.Open(filename);


    // First, check if the camera is calibrated.
    // If not, we need to run the calibration widget.
    Vector<NUMTRACKERCAMPARAMETERS> vTest = GV3::get<Vector<NUMTRACKERCAMPARAMETERS> >("Camera.Parameters", ATANCamera::mvDefaultParams, HIDDEN);
    mpCamera = new ATANCamera("Camera");
    if (vTest == ATANCamera::mvDefaultParams) {
        std::cout << std::endl;
        std::cout << "! Camera.Parameters is not set, need to run the CameraCalibrator tool" << std::endl;
        std::cout << "  and/or put the Camera.Parameters= line into the appropriate .cfg file." << std::endl;
        exit(1);
    }

    mpMap = new Map;
    mpMapMaker = new MapMaker(*mpMap, *mpCamera, stats, deviceFolder, mode);
    mpTracker = new Tracker(mVideoSource.Size(), *mpCamera, *mpMap, *mpMapMaker, mVideoSource, stats, pathFile);
    if (mode == MapMaker::MM_MODE_SINGLE_MODEL) {
        SE3<> tmp;
        mpMapMaker->LoadModelFromFolder(model, mVideoSource.Size(), tmp);
    } else if (mode == MapMaker::MM_MODE_FROM_INSTALLER) {
        SE3<> tmp;
        mpMapMaker->LoadMapFromInstaller(model, mVideoSource.Size(), tmp);
    }
    mbDone = false;
};

void HeadlessSystem::PrintStats() {
    std::cout << std::endl << std::endl;
    std::cout << "#############   T R A C K I N G   S T A T S   #############" << std::endl;
    std::cout << "AVG_TRACK_TIME=" << stats.GetAvgTrackingTime() << std::endl;
    std::cout << "N_LOADED_MODELS=" << stats.GetLoadedModels() << std::endl;
    std::cout << "N_SUCCESS_RELOC=" << stats.GetSuccessfulRelocs() << std::endl;
    std::cout << "N_KEYFRAMES_START=" << stats.GetNumOfStartKeyFrames() << std::endl;
    std::cout << "N_POINTS_START=" << stats.GetNumOfStartPoints() << std::endl;
    std::cout << "N_KEYFRAMES_END=" << stats.GetNumOfEndKeyFrames() << std::endl;
    std::cout << "N_POINTS_END=" << stats.GetNumOfEndPoints() << std::endl;
    std::cout << "###########################################################" << std::endl;
}

void HeadlessSystem::Run() {
    static bool bSkipFrames = true;

    cv::Mat imBW, imRGB;
    lastFrameN = -1;
    while (!mbDone) {
        if (!mVideoSource.GetAndFillFrameBWandRGB(imBW, imRGB)) {
            mbDone = true;
            std::cout << "VIDEO ENDED" << std::endl;
            break;
        }
        if (!bSkipFrames || lastFrameN != mVideoSource.GetFrameN()){
            lastFrameN = mVideoSource.GetFrameN();
            auto quality = mpTracker->GetTrackingQuality();
            bool timing = quality == Tracker::TRACKING_GOOD || quality == Tracker::TRACKING_DODGY || quality == Tracker::TRACKING_BAD;
            auto start = std::chrono::high_resolution_clock::now();
            mpTracker->TrackFrame(imBW);
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
            if(timing)
                stats.AddTrackTime(duration.count());
        }
    }

    stats.SetEndStats(mpMap->vpKeyFrames.size(), mpMap->vpPoints.size());
    PrintStats();
}









