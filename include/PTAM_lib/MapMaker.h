// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

//
// This header declares the MapMaker class
// MapMaker makes and maintains the Map struct
// Starting with stereo initialisation from a bunch of matches
// over keyframe insertion, continual bundle adjustment and 
// data-association refinement.
// MapMaker runs in its own thread, although some functions
// (notably stereo init) are called by the tracker and run in the 
// tracker's thread.

#ifndef __MAPMAKER_H
#define __MAPMAKER_H
#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/thread.h>
#include <cvd/colourspace_convert.h>

#include <queue>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <Eigen/Core>

#include "Map.h"
#include "KeyFrame.h"
#include "ATANCamera.h"
#include "DBoW2.h"
#include "TrackingStats.h"

// Each MapPoint has an associated MapMakerData class
// Where the mapmaker can store extra information
 
struct MapMakerData
{
  std::set<KeyFrame*> sMeasurementKFs;   // Which keyframes has this map point got measurements in?
  std::set<KeyFrame*> sNeverRetryKFs;    // Which keyframes have measurements failed enough so I should never retry?
  inline int GoodMeasCount()            
  {  return sMeasurementKFs.size(); }
};

struct KeypointResize
{
    double offsetX = 0;
    double offsetY = 0;
    double scaleX = 0;
    double scaleY = 0;

    KeypointResize(double ox, double oy, double sx, double sy): offsetX(ox), offsetY(oy), scaleX(sx), scaleY(sy) {}


    CVD::ImageRef getAdjustedKeypoint(const PTAMImagePoint &imp) {
        return CVD::ImageRef(offsetX + scaleX * imp.x, offsetY + scaleY * imp.y);
    }
};

// MapMaker dervives from CVD::Thread, so everything in void run() is its own thread.
class MapMaker : protected CVD::Thread {
public:
    enum Mode {
        MM_MODE_MAP, MM_MODE_RELOC
    };

    enum OperationMode {
        MM_MODE_INSTALL, MM_MODE_FULL_AUTO, MM_MODE_FROM_INSTALLER, MM_MODE_SINGLE_MODEL
    };

    MapMaker(Map &m, const ATANCamera &cam, TrackingStats &stats, const std::string &deviceFolder, OperationMode mode);

    ~MapMaker();

    void AddKeyFrame(KeyFrame &k);   // Add a key-frame to the map. Called by the tracker.
    void RequestReset();   // Request that the we reset. Called by the tracker.
    bool ResetDone();      // Returns true if the has been done.
    int QueueSize() { return mvpKeyFrameQueue.size(); } // How many KFs in the queue waiting to be added?
    bool NeedNewKeyFrame(KeyFrame &kCurrent);            // Is it a good camera pose to add another KeyFrame?
    bool IsDistanceToNearestKeyFrameExcessive(KeyFrame &kCurrent);  // Is the camera far away from the nearest KeyFrame (i.e. maybe lost?)
    bool IsDistanceToRelocKeyFrameExcessive(SE3<> &camPose, KeyFrame &kTarget);

    int nRelocFeatureCount;
    int nBestRelocKF;
    std::string currentModelName;

    void SetMode(Mode m);

    void AddRelocImage(KeyFrame &k);
    bool NewRelocPoseReady();
    int BestRelocKeyFrame();
    SE3<> LastRelocPose();
    double GetOneCM() { return mdOneCM; };

    bool LoadModelFromFolder(const std::string &folder, CVD::ImageRef imSize, SE3<> &se3TrackerPose);
    bool LoadMapFromInstaller(const std::string &rootFolder, CVD::ImageRef imSize, SE3<> &se3TrackerPose);

    inline bool WasKeyframeAdded() {
        bool b = mKeyframeAdded;
        mKeyframeAdded = false;
        return b;
    }

protected:

    Map &mMap;               // The map
    ATANCamera mCamera;      // Same as the tracker's camera: N.B. not a reference variable!
    virtual void run();      // The MapMaker thread code lives here

    void ApplyGlobalTransformationToMap(const Eigen::Matrix<float, 4, 4> &trans);

    // Map expansion functions:
    void AddKeyFrameFromTopOfQueue();
    void ThinCandidates(KeyFrame &k, int nLevel);
    void AddSomeMapPoints(int nLevel);
    int AddSomeMapPoints(int nLevel, int kfID, int limit = 0);
    bool AddPointEpipolar(KeyFrame &kSrc, KeyFrame &kTarget, int nLevel, int nCandidate);
    bool RemoveKeyFrame(KeyFrame *kf);

    // Returns point in ref frame B
    Vector<3> ReprojectPoint(SE3<> se3AfromB, const Vector<2> &v2A, const Vector<2> &v2B);

    // Bundle adjustment functions:
    void BundleAdjust(std::set<KeyFrame *>, std::set<KeyFrame *>, std::set<MapPoint *>, bool, bool = false);
    void BundleAdjustAll(bool = false);
    void BundleAdjustRecent();
    void BundleAdjustKeyframe(int i);

    // Data association functions:
    int ReFindInSingleKeyFrame(KeyFrame &k);
    void ReFindFromFailureQueue();
    void ReFindNewlyMade();
    bool ReFind_Common(KeyFrame &k, MapPoint &p);

    // General Maintenance/Utility:
    void Reset();
    void HandleBadPoints();
    double DistToNearestKeyFrame(KeyFrame &kCurrent);
    static double KeyFrameLinearDist(KeyFrame &k1, KeyFrame &k2);
    KeyFrame *ClosestKeyFrame(KeyFrame &k);
    std::vector<KeyFrame *> NClosestKeyFrames(KeyFrame &k, unsigned int N);
    static std::vector<KeyFrame *> NClosestKeyFramesInList(KeyFrame &k, unsigned int N, std::vector<KeyFrame *> &list);
    void RefreshSceneDepth(KeyFrame *pKF);
    KeypointResize ConvertAndResizeWithAspectRatio(const cv::Mat &input, CVD::Image<CVD::byte> &imBW);
    Eigen::Matrix<float, 4, 4> GetTransformFromModelToWorld(PTAMInstallerFile &e);
    SE3<> GetCameraPosePNP(const PTAMInstallerFile &exprt, int frameN, const ImageRefKD &keypointsKD,
                           KeypointResize &resizer, Eigen::Matrix<float, 4, 4> modelToWorld, cv::Vec3d &prevR,
                           cv::Vec3d &prevT);

    // GUI Interface:
    void GUICommandHandler(std::string sCommand, std::string sParams);
    static void GUICommandCallBack(void *ptr, std::string sCommand, std::string sParams);
    struct Command {
        std::string sCommand;
        std::string sParams;
    };
    std::vector<Command> mvQueuedCommands;


    // Member variables:
    std::vector<KeyFrame *> mvpKeyFrameQueue;  // Queue of keyframes from the tracker waiting to be processed
    std::vector<std::pair<KeyFrame *, MapPoint *> > mvFailureQueue; // Queue of failed observations to re-find
    std::queue<MapPoint *> mqNewQueue;   // Queue of newly-made map points to re-find in other KeyFrames
    bool mKeyframeAdded = false;

    double mdOneCM; // Distance that represents 1 cm in reference world
    double mdWiggleScale;  // Metric distance between the first two KeyFrames (copied from GVar)
    // This sets the scale of the map
    GVars3::gvar3<double> mgvdWiggleScale;   // GVar for above
    double mdWiggleScaleDepthNormalized;  // The above normalized against scene depth,
    // this controls keyframe separation

    bool mbBundleConverged_Full;    // Has global bundle adjustment converged?
    bool mbBundleConverged_Recent;  // Has local bundle adjustment converged?

    // Thread interaction signalling stuff
    bool mbResetRequested;   // A reset has been requested
    bool mbResetDone;        // The reset was done.
    bool mbBundleAbortRequested;      // We should stop bundle adjustment
    bool mbBundleRunning;             // Bundle adjustment is running
    bool mbBundleRunningIsRecent;     //    ... and it's a local bundle adjustment.

    double minKFDistance = 10;
    double insertKeypointRadius = 10;


    OperationMode operationMode = MM_MODE_FULL_AUTO;
    Mode currentMode = MM_MODE_MAP;
    std::string deviceFolder;
    std::string currentModel;
    TrackingStats &stats;

    // Relocalization
    void BuildRelocIndex();
    void ProcessReloc();

    cv::FlannBasedMatcher keyframeImageMatcher;
    std::mutex relocImageMutex;
    cv::Mat lastRelocImage;
    SE3<> lastRelocPose;
    bool newRelocImage = false;
    bool newRelocPose = false;
    OrbDatabase relocDBoW;
    std::vector<std::string> relocFeatureToModel;
    std::vector<std::string> last10RelocModels;
};
#endif


















