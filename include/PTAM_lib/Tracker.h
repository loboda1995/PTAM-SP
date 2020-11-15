//-*- C++ -*-
// Copyright 2008 Isis Innovation Limited
// 
// This header declares the Tracker class.
// The Tracker is one of main components of the system,
// and is responsible for determining the pose of a camera
// from a video feed. It uses the Map to track, and communicates 
// with the MapMaker (which runs in a different thread)
// to help construct this map.
//
// Initially there is no map, so the Tracker also has a mode to 
// do simple patch tracking across a stereo pair. This is handled 
// by the TrackForInitialMap() method and associated sub-methods. 
// Once there is a map, TrackMap() is used.
//
// Externally, the tracker should be used by calling TrackFrame()
// with every new input video frame. This then calls either 
// TrackForInitialMap() or TrackMap() as appropriate.
//

#ifndef __TRACKER_H
#define __TRACKER_H

#include "MapMaker.h"
#include "ATANCamera.h"
#include "VideoSource.h"

#include <sstream>
#include <vector>
#include <list>
#include <fstream>
#include <opencv2/imgproc.hpp>


class TrackerData;

class Tracker {
public:
    Tracker(CVD::ImageRef irVideoSize, const ATANCamera &c, Map &m, MapMaker &mm, VideoSource &v, TrackingStats &stats, const std::string &pathFile);

    ~Tracker();

    // TrackFrame is the main working part of the tracker: call this every frame.
    void TrackFrame(cv::Mat &imFrame);

    inline SE3<> GetCurrentPose() { return mse3CamFromWorld; }

    // Gets messages to be printed on-screen for the user.
    std::string GetMessageForUser();

    enum TrackingState{
        TRACKING_GOOD, TRACKING_DODGY, TRACKING_BAD, LOADING_SCENE, RELOCATING, KEYFRAME_ADDED
    };

    TrackingState GetTrackingQuality() {
        if (mMapMaker.WasKeyframeAdded())
            return KEYFRAME_ADDED;
        if (mMapMaker.currentModelName.empty())
            return LOADING_SCENE;
        if (mTrackingQuality == GOOD)
            return TRACKING_GOOD;
        if (mTrackingQuality == DODGY)
            return TRACKING_DODGY;
        if (mTrackingQuality == BAD && mnLostFrames < 3)
            return TRACKING_BAD;
        return RELOCATING;
    }

    void GetCameraPose(cv::Vec3d &r, cv::Vec3d &t);
    static bool MatrixHasNaN(SE3<> mat);

protected:
    std::ofstream locationFile;
    VideoSource &mVideoSource;

    KeyFrame mCurrentKF;            // The current working frame as a keyframe struct

    // The major components to which the tracker needs access:
    Map &mMap;                      // The map, consisting of points and keyframes
    MapMaker &mMapMaker;            // The class which maintains the map
    ATANCamera mCamera;             // Projection model

    CVD::ImageRef mirSize;          // Image size of whole image

    TrackingStats &stats;

    void Reset();                   // Restart from scratch. Also tells the mapmaker to reset itself.

    // Methods for tracking the map once it has been made:
    void TrackMap();                // Called by TrackFrame if there is a map.
    void AssessTrackingQuality();   // Heuristics to choose between good, poor, bad.
    void ApplyMotionModel();        // Decaying velocity motion model applied prior to TrackMap
    void UpdateMotionModel();       // Motion model is updated after TrackMap
    int SearchForPoints(std::vector<TrackerData *> &vTD,
                        int nRange,
                        int nFineIts);  // Finds points in the image
    Vector<6> CalcPoseUpdate(std::vector<TrackerData *> vTD,
                             double dOverrideSigma = 0.0,
                             bool bMarkOutliers = false); // Updates pose from found points.
    SE3<> mse3CamFromWorld;           // Camera pose: this is what the tracker updates every frame.
    SE3<> mse3StartPos;               // What the camera pose was at the start of the frame.
    Vector<6> mv6CameraVelocity;    // Motion model
    double mdVelocityMagnitude;     // Used to decide on coarse tracking
    double mdMSDScaledVelocityMagnitude; // Velocity magnitude scaled by relative scene depth.
    bool mbDidCoarse;               // Did tracking use the coarse tracking stage?


    // Interface with map maker:
    int mnFrame;                    // Frames processed since last reset
    int mnLastKeyFrameDropped;      // Counter of last keyframe inserted.
    void AddNewKeyFrame();          // Gives the current frame to the mapmaker to use as a keyframe

    // Tracking quality control:
    int manMeasAttempted[LEVELS];
    int manMeasFound[LEVELS];
    enum {
        BAD, DODGY, GOOD
    } mTrackingQuality;
    int mnLostFrames;

    // Relocalisation functions:
    bool AttemptRecovery();         // Called by TrackFrame if tracking is lost.
    bool mbJustRecoveredSoUseCoarse;// Always use coarse tracking after recovery!

    // Frame-to-frame motion init:
    SmallBlurryImage *mpSBILastFrame;
    SmallBlurryImage *mpSBIThisFrame;

    void CalcSBIRotation();

    Vector<6> mv6SBIRot;
    bool mbUseSBIInit;

    // User interaction for initial tracking:
    std::ostringstream mMessageForUser;

    // GUI interface:
    void GUICommandHandler(std::string sCommand, std::string sParams);

    static void GUICommandCallBack(void *ptr, std::string sCommand, std::string sParams);

    struct Command {
        std::string sCommand;
        std::string sParams;
    };
    std::vector<Command> mvQueuedCommands;
};

#endif






