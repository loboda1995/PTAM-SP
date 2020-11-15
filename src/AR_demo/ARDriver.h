//
// Created by luka on 16. 09. 20.
//

#ifndef PTAM_ARDEMOAPP_H
#define PTAM_ARDEMOAPP_H


#include <string>
#include <cvd/image.h>
#include "Map.h"
#include "MapMaker.h"
#include "Tracker.h"
#include "TrackingStats.h"

class ARDriver {
public:
    ARDriver(const std::string &deviceFolder, const CVD::ImageRef videoSize);
    ~ARDriver() {
        delete mpTracker;
        delete mpMapMaker;
        delete mpMap;
        delete mpCamera;
    }

    inline std::string GetCurrentScene() { return mpMapMaker->currentModelName; }
    Tracker::TrackingState TrackFrame(cv::Mat &imBw);
    inline float GetAvgTrack() { return stats.GetAvgTrackingTime(); };
    inline int GetNKeyframes() { return mpMap->vpKeyFrames.size(); }
    inline int GetNPoints() { return mpMap->vpPoints.size(); }
    inline int GetNRelocs() { return stats.GetSuccessfulRelocs(); }

    void GetCameraProjectionMat(double zNear, double zFar, cv::Mat &projectionMatrix, double scale);
    void GetCameraPoseMat(cv::Mat &viewMatrix);
    void GetCameraPosGLMat(cv::Mat &viewMatrix);

private:
    Map *mpMap;
    MapMaker *mpMapMaker;
    Tracker *mpTracker;
    ATANCamera *mpCamera;
    TrackingStats stats;

    cv::Mat cvToGlMat;
};


#endif //PTAM_ARDEMOAPP_H
