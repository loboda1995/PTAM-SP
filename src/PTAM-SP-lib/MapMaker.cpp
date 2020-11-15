// Copyright 2008 Isis Innovation Limited
#include "MapMaker.h"
#include "MapPoint.h"
#include "Bundle.h"
#include "PatchFinder.h"
#include "SmallMatrixOpts.h"
#include "TrackerData.h"

#include <cvd/vector_image_ref.h>

#include <TooN/SVD.h>

#include <gvars3/instances.h>
#include <fstream>
#include <algorithm>
#include <opencv2/imgproc.hpp>

#include "PTAMInstallerFile.h"
#include <cereal/archives/binary.hpp>
#include <opencv2/opencv.hpp>
#include <ShiTomasi.h>

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

using namespace CVD;
using namespace GVars3;

// Constructor sets up internal reference variable to Map.
// Most of the intialisation is done by Reset()..
MapMaker::MapMaker(Map &m, const ATANCamera &cam, TrackingStats &stats, const std::string &deviceFolder, OperationMode mode)
        : mMap(m), mCamera(cam), deviceFolder(deviceFolder), operationMode(mode), stats(stats) {
    if (operationMode == MM_MODE_FULL_AUTO) {
        relocDBoW.load(deviceFolder + "/reloc_db.yml.gz");
        cv::FileStorage fs(deviceFolder + "/reloc_db_meta.yml", cv::FileStorage::READ);
        fs["feature_to_model"] >> relocFeatureToModel;
    }
    static gvar3<int>minRelocKeypoints("MapMaker.MinRelocKeypoints", 500, SILENT);
    nRelocFeatureCount = *minRelocKeypoints;
    cv::Ptr<cv::flann::IndexParams> indexParams = cv::makePtr<cv::flann::LshIndexParams>(6, 12, 1);
    cv::Ptr<cv::flann::SearchParams> searchParams = cv::makePtr<cv::flann::SearchParams>(50);
    keyframeImageMatcher = cv::FlannBasedMatcher(indexParams, searchParams);

    mbResetRequested = false;
    Reset();
    if (operationMode != MM_MODE_INSTALL) {
        start(); // This CVD::thread func starts the map-maker thread with function run()
        GUI.RegisterCommand("SaveMap", GUICommandCallBack, this);
    }
    GV3::Register(mgvdWiggleScale, "MapMaker.WiggleScale", 0.10, SILENT); // Default to 10cm between keyframes
};

void MapMaker::Reset() {
    // This is only called from within the mapmaker thread...
    mMap.Reset();
    mvFailureQueue.clear();
    while (!mqNewQueue.empty()) mqNewQueue.pop();
    for (const auto &kf : mMap.vpKeyFrames)
        delete kf;
    for (const auto &kf : mvpKeyFrameQueue)
        delete kf;
    mMap.vpKeyFrames.clear();
    mvpKeyFrameQueue.clear();
    mbBundleRunning = false;
    mbBundleConverged_Full = true;
    mbBundleConverged_Recent = true;
    mbResetDone = true;
    mbResetRequested = false;
    mbBundleAbortRequested = false;
    keyframeImageMatcher.clear();
}

// CHECK_RESET is a handy macro which makes the mapmaker thread stop
// what it's doing and reset, if required.
#define CHECK_RESET if(mbResetRequested) {Reset(); continue;};
#define CHECK_RELOC if(currentMode == MM_MODE_RELOC) {continue;};

void MapMaker::run() {

#ifdef WIN32
    // For some reason, I get tracker thread starvation on Win32 when
    // adding key-frames. Perhaps this will help:
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_LOWEST);
#endif
    while (!shouldStop())  // ShouldStop is a CVD::Thread func which return true if the thread is told to exit.
    {
        CHECK_RESET;
        // sleep(5); // Sleep not really necessary, especially if mapmaker is busy
        CHECK_RESET;

        // Handle any GUI commands encountered..
        while (!mvQueuedCommands.empty()) {
            GUICommandHandler(mvQueuedCommands.begin()->sCommand, mvQueuedCommands.begin()->sParams);
            mvQueuedCommands.erase(mvQueuedCommands.begin());
        }

        if (mMap.IsGood()) {
            int cost = 0;
            for (const auto &kf : mMap.vpKeyFrames) {
                if (kf->state < KeyFrame::LITE) {
                    cv::Mat img = cv::imread(kf->imagePath, cv::IMREAD_GRAYSCALE);
                    CVD::Image<CVD::byte> imBW(CVD::ImageRef(img.cols, img.rows));
                    cv::Mat tmp(img.rows, img.cols, CV_8UC1, imBW.data());
                    img.copyTo(tmp);
                    kf->MakeKeyFrame_Lite(imBW);
                    cost += 2;
                }
                if (cost > 20)
                    break;
            }
            if (cost == 0) {
                for (const auto &kf : mMap.vpKeyFrames) {
                    if (kf->state < KeyFrame::REST) {
                        kf->MakeKeyFrame_Rest();
                        cost++;
                    }
                    if (cost > 20)
                        break;
                }
            }
        }

        if (currentMode == MM_MODE_RELOC) {
            if (newRelocImage) {
                ProcessReloc();
                continue;
            }
        } else {
            if (!mMap.IsGood())  // Nothing to do if there is no map yet!
                continue;

            // From here on, mapmaker does various map-maintenance jobs in a certain priority
            // Hierarchy. For example, if there's a new key-frame to be added (QueueSize() is >0)
            // then that takes high priority.

            CHECK_RESET;
            CHECK_RELOC;
            // Should we run local bundle adjustment?
            if (!mbBundleConverged_Recent && QueueSize() == 0) {
                BundleAdjustRecent();
            }

            CHECK_RESET;
            CHECK_RELOC;
            // Are there any newly-made map points which need more measurements from older key-frames?
            if (mbBundleConverged_Recent && QueueSize() == 0) {
                ReFindNewlyMade();
            }

            CHECK_RESET;
            CHECK_RELOC;
            // Run global bundle adjustment?
            if (mbBundleConverged_Recent && !mbBundleConverged_Full && QueueSize() == 0) {
                BundleAdjustAll();
            }

            CHECK_RESET;
            CHECK_RELOC;
            // Very low priorty: re-find measurements marked as outliers
            if (mbBundleConverged_Recent && mbBundleConverged_Full && rand() % 20 == 0 && QueueSize() == 0) {
                ReFindFromFailureQueue();
            }

            CHECK_RESET;
            CHECK_RELOC;
            HandleBadPoints();

            CHECK_RESET;
            CHECK_RELOC;
            // Any new key-frames to be added?
            if (QueueSize() > 0) {
                AddKeyFrameFromTopOfQueue(); // Integrate into map data struct, and process
            }
        }
    }
}


// Tracker calls this to demand a reset
void MapMaker::RequestReset() {
    mbResetDone = false;
    mbResetRequested = true;
}

bool MapMaker::ResetDone() {
    return mbResetDone;
}

// HandleBadPoints() Does some heuristic checks on all points in the map to see if 
// they should be flagged as bad, based on tracker feedback.
void MapMaker::HandleBadPoints() {
    // Did the tracker see this point as an outlier more often than as an inlier?
    for (unsigned int i = 0; i < mMap.vpPoints.size(); i++) {
        MapPoint &p = *mMap.vpPoints[i];
        if (!p.bFromModel && p.nMEstimatorOutlierCount > 20 && p.nMEstimatorOutlierCount > p.nMEstimatorInlierCount)
            p.bBad = true;
    }

    // All points marked as bad will be erased - erase all records of them
    // from keyframes in which they might have been measured.
    for (unsigned int i = 0; i < mMap.vpPoints.size(); i++) {
        if (mMap.vpPoints[i]->bBad) {
            MapPoint *p = mMap.vpPoints[i];
            for (unsigned int j = 0; j < mMap.vpKeyFrames.size(); j++) {
                KeyFrame &k = *mMap.vpKeyFrames[j];
                if (k.mMeasurements.count(p))
                    k.mMeasurements.erase(p);
            }
        }
    }

    // Move bad points to the trash list.
    mMap.MoveBadPointsToTrash();
}

MapMaker::~MapMaker() {
    if (operationMode == MM_MODE_INSTALL)
        return;
    mbBundleAbortRequested = true;
    stop(); // makes shouldStop() return true
    std::cout << "Waiting for mapmaker to die.." << std::endl;
    join();
    std::cout << " .. mapmaker has died." << std::endl;
}


// Finds 3d coords of point in reference frame B from two z=1 plane projections
Vector<3> MapMaker::ReprojectPoint(SE3<> se3AfromB, const Vector<2> &v2A, const Vector<2> &v2B) {
    Matrix<3, 4> PDash;
    PDash.slice<0, 0, 3, 3>() = se3AfromB.get_rotation().get_matrix();
    PDash.slice<0, 3, 3, 1>() = se3AfromB.get_translation().as_col();

    Matrix<4> A;
    A[0][0] = -1.0;
    A[0][1] = 0.0;
    A[0][2] = v2B[0];
    A[0][3] = 0.0;
    A[1][0] = 0.0;
    A[1][1] = -1.0;
    A[1][2] = v2B[1];
    A[1][3] = 0.0;
    A[2] = v2A[0] * PDash[2] - PDash[0];
    A[3] = v2A[1] * PDash[2] - PDash[1];

    SVD<4, 4> svd(A);
    Vector<4> v4Smallest = svd.get_VT()[3];
    if (v4Smallest[3] == 0.0)
        v4Smallest[3] = 0.00001;
    return project(v4Smallest);
}

Eigen::Matrix<float, 4, 4> MapMaker::GetTransformFromModelToWorld(PTAMInstallerFile &e) {
    Eigen::MatrixXf model;
    model.resize(3, 4);
    Eigen::MatrixXf ref;
    ref.resize(3, 4);
    for (int i = 0; i < 4; i++) {
        model.col(i) = e.calModelPoints[i].getEigen4f().head<3>();
        ref.col(i) = e.calWorldPoints[i].getEigen4f().head<3>();
    }
    return Eigen::umeyama(model, ref);
}

bool MapMaker::RemoveKeyFrame(KeyFrame *kf) {
    for (const auto &m : kf->mMeasurements) {
        auto p = m.first;
        p->pMMData->sMeasurementKFs.erase(kf);
        if (p->pMMData->sMeasurementKFs.empty()) {
            // If no more measurements, remove this point from map
            delete p;
            mMap.vpPoints.erase(std::remove(mMap.vpPoints.begin(), mMap.vpPoints.end(), p),
                                mMap.vpPoints.end());
        } else if (p->pPatchSourceKF == kf) {
            // If source KeyFrame is this keyframe, replace it with the closes next KeyFrame
            std::vector<KeyFrame *> candidateKFs;
            for (const auto &c : p->pMMData->sMeasurementKFs) {
                candidateKFs.push_back(c);
            }
            auto best = NClosestKeyFramesInList(*kf, 1, candidateKFs);
            if (best.size() > 0) {
                p->pPatchSourceKF = best[0];
                best[0]->mMeasurements[p].Source = Measurement::SRC_ROOT;
                auto newCenter = best[0]->mMeasurements[p].v2RootPos;
                p->irCenter = CVD::ImageRef(newCenter[0], newCenter[1]);
                p->v3Center_NC = unproject(mCamera.UnProject(p->irCenter));
                p->v3OneDownFromCenter_NC = unproject(mCamera.UnProject(p->irCenter + CVD::ImageRef(0, 1)));
                p->v3OneRightFromCenter_NC = unproject(mCamera.UnProject(p->irCenter + CVD::ImageRef(1, 0)));
                normalize(p->v3Center_NC);
                normalize(p->v3OneDownFromCenter_NC);
                normalize(p->v3OneRightFromCenter_NC);
                p->RefreshPixelVectors();
            } else {
                std::cout << "Failed to find replacement source KeyFrame" << std::endl;
                return false;
            }
        }
    }
    delete kf;
    mMap.vpKeyFrames.erase(std::remove(mMap.vpKeyFrames.begin(), mMap.vpKeyFrames.end(), kf),mMap.vpKeyFrames.end());
    return true;
}

KeypointResize MapMaker::ConvertAndResizeWithAspectRatio(const cv::Mat &input, CVD::Image<CVD::byte> &imBW) {
    cv::Mat output;
    double dstW = imBW.size().x;
    double dstH = imBW.size().y;
    double h1 = dstW * (input.rows/(double)input.cols);
    double w2 = dstH * (input.cols/(double)input.rows);
    int top = 0;
    int down = 0;
    int left = 0;
    int right = 0;
    double scaleX, scaleY;
    if( h1 <= dstH) {
        // only horizontal borders
        top = (dstH - h1) / 2;
        down = top;
        scaleX = dstW / (double)input.cols;
        scaleY = h1 / (double)input.rows;
        cv::resize(input, output, cv::Size(dstW, h1));
    } else {
        // only vertical borders
        left = (dstW - w2) / 2;
        right = left;
        scaleX = w2/(double)input.cols;
        scaleY = dstH/(double)input.rows;
        cv::resize(input, output, cv::Size(w2, dstH));
    }
    cv::copyMakeBorder(output, output, top, down, left, right, cv::BORDER_CONSTANT, 0 );

    CVD::BasicImage<CVD::byte> y = CVD::BasicImage<CVD::byte>(output.data, imBW.size());
    convert_image(y, imBW);
    return KeypointResize(left, top, scaleX, scaleY);
}

SE3<> MapMaker::GetCameraPosePNP(const PTAMInstallerFile &exprt, int frameN, const ImageRefKD &keypointsKD, KeypointResize &resizer, Eigen::Matrix<float, 4, 4> modelToWorld, cv::Vec3d &prevR, cv::Vec3d &prevT) {
    std::vector<cv::Point3f> worldPos;
    std::vector<cv::Point2f> imgPos;
    for (int pid = 0; pid < exprt.points.size(); pid++) {
        auto point = exprt.points[pid];

        int idx = -1;
        for (int i = 0; i < point.imagePoints.size(); i++) {
            const auto &imageP = point.imagePoints[i];
            if (imageP.level == 0 && imageP.frame == frameN) {
                idx = i;
                break;
            }
        }
        if (idx < 0)
            continue;


        auto tmp = resizer.getAdjustedKeypoint(point.imagePoints[idx]);
        double query_pt[2] = { static_cast<double>(tmp.x), static_cast<double>(tmp.y) };
        std::vector<size_t> ret_index(1);
        std::vector<double> out_dist_sqr(1);
        int nRes = keypointsKD.knnSearch(&query_pt[0], 1, &ret_index[0], &out_dist_sqr[0]);
        if (nRes == 0 || out_dist_sqr[0] >= (2*insertKeypointRadius)*(2*insertKeypointRadius))
            continue;

        auto w = point.getEigen4f();
        w = modelToWorld * w;
        worldPos.push_back(cv::Point3f(w.x(), w.y(), w.z()));
        imgPos.push_back(cv::Point2f(tmp.x, tmp.y));
    }


    cv::Vec3d R = prevR;
    cv::Vec3d t = prevT;
    cv::solvePnPRansac(worldPos, imgPos, mCamera.GetCameraMatrix(), cv::noArray(), R, t, true, 2000, insertKeypointRadius, 0.99);

    prevR = R;
    prevT = t;


    Vector<3, double> finalT;
    finalT[0] = t(0);
    finalT[1] = t(1);
    finalT[2] = t(2);
    Vector<3, double> finalR;
    finalR[0] = R(0);
    finalR[1] = R(1);
    finalR[2] = R(2);

    return SE3<>(SO3<>(finalR), finalT);
}

bool MapMaker::LoadModelFromFolder(const std::string &folder, CVD::ImageRef imSize, SE3<> &se3TrackerPose) {
    Reset();
    mCamera.SetImageSize(imSize);

    if (!mMap.LoadModelFromFile(mCamera, deviceFolder + "/" + folder, currentModelName, mdOneCM))
        return false;

    mdWiggleScale = *mgvdWiggleScale;
    minKFDistance = 10.0;
    insertKeypointRadius = 7.0 * ((mCamera.GetImageSize()[0] + mCamera.GetImageSize()[1]) / 2.0) / 1000;
    if (insertKeypointRadius < 5)
        insertKeypointRadius = 5;
    mdWiggleScale = mdOneCM * 10.0;


    // Prepare data for relocalization
    for (const auto &kf : mMap.vpKeyFrames) {
        keyframeImageMatcher.add(kf->relocFrameDescriptor);
    }
    keyframeImageMatcher.train();

    double meanDepth = 0;
    for (auto & kf : mMap.vpKeyFrames) {
        RefreshSceneDepth(kf);
        meanDepth += kf->dSceneDepthMean;
    }
    mdWiggleScaleDepthNormalized = mdWiggleScale / (meanDepth / mMap.vpKeyFrames.size());

    mMap.bGood = true;
    se3TrackerPose = mMap.vpKeyFrames[0]->se3CfromW;
    stats.AddLoadedModel(mMap.vpKeyFrames.size(), mMap.vpPoints.size());
    return true;
}

bool MapMaker::LoadMapFromInstaller(const std::string &rootFolder, CVD::ImageRef imSize, SE3<> &se3TrackerPose) {
    mdWiggleScale = *mgvdWiggleScale; // Cache this for the new map.
    mCamera.SetImageSize(imSize);
    minKFDistance = 10.0;
    insertKeypointRadius = 10.0 * ((mCamera.GetImageSize()[0] + mCamera.GetImageSize()[1]) / 2.0) / 1000;

    // Load model
    std::ifstream ptamFile(rootFolder + "/installer", std::ios::binary);
    PTAMInstallerFile ptamExport;
    cereal::BinaryInputArchive iarchive(ptamFile);
    iarchive(ptamExport);

    // Get transformation from model to reference world
    Eigen::Matrix<float, 4, 4> transformFromModelToWorld = GetTransformFromModelToWorld(ptamExport);

    // Get scale
    if (ptamExport.scalePoint1 >= 0 && ptamExport.scalePoint2 >= 0) {
        Eigen::Vector4f s1 = ptamExport.calWorldPoints[ptamExport.scalePoint1].getEigen4f();
        Eigen::Vector4f s2 = ptamExport.calWorldPoints[ptamExport.scalePoint2].getEigen4f();
        auto d = (s1-s2).norm();
        mdOneCM = d / ptamExport.scaleDistance;
        mdWiggleScale = mdOneCM * 10.0;
    } else {
        return false;
    }

    std::srand(21);
    SE3<> prevSE3;
    std::vector<KeypointResize> kpResize;
    // Prepare KD tree for fast knn search of LEVEL 0 keypoints
    std::vector<ImageRefKD*> levelKeypointKD;

    cv::Vec3d prevR, prevT;
    for (int frameN = 0; frameN < ptamExport.frameNames.size(); frameN++) {
        // Read keyframe image
        cv::Mat image = imread(rootFolder + "/" + ptamExport.frameNames[frameN], cv::IMREAD_GRAYSCALE);
        if (!image.data) {
            std::cout << "Could not open or find the image" << std::endl;
            return false;
        }
        Image<CVD::byte> imBW(CVD::ImageRef(mCamera.GetImageSize()[0], mCamera.GetImageSize()[1]));
        auto resizer = ConvertAndResizeWithAspectRatio(image, imBW);
        kpResize.push_back(resizer);

        // Prepare keyframe structure
        auto *kf = new KeyFrame();
        kf->MakeKeyFrame_Lite(imBW);
        kf->MakeKeyFrame_Rest();

        auto *pc = new PointCloud();
        pc->pts.resize(kf->aLevels[0].vMaxCorners.size());
        for (int i = 0; i < kf->aLevels[0].vMaxCorners.size(); i++) {
            pc->pts[i] = PointCloud::Point(kf->aLevels[0].vMaxCorners[i].x, kf->aLevels[0].vMaxCorners[i].y);
        }
        auto kd = new ImageRefKD(2, *pc, nanoflann::KDTreeSingleIndexAdaptorParams(20));
        kd->buildIndex();
        levelKeypointKD.push_back(kd);


        kf->bFixed = frameN == 0;
        kf->se3CfromW = GetCameraPosePNP(ptamExport, frameN, *kd, resizer, transformFromModelToWorld, prevR, prevT);
        mMap.vpKeyFrames.push_back(kf);

        if (frameN == 1) {
            mMap.vpKeyFrames[0]->se3CfromW = GetCameraPosePNP(ptamExport, 0, *levelKeypointKD[0], kpResize[0], transformFromModelToWorld, prevR, prevT);
        }
    }


    std::vector<MapPoint*> modelCalPoints;
    std::vector<Eigen::Vector4f> modelRefPoints;
    // Transform imported model and insert it
    std::random_shuffle(ptamExport.points.begin(), ptamExport.points.end());
    for (int i = 0; i < ptamExport.points.size(); i++) {
        if (mMap.vpPoints.size() >= 75 * ptamExport.frameNames.size())
            break;
        PTAMPoint point = ptamExport.points[i];

        // Only insert points that are around keypoint locations in 75% of keyframe images
        double good = 0;
        int candidates = 0;
        int first = -1;
        for (int j = 0; j < point.imagePoints.size(); j++) {
            auto p = point.imagePoints[j];
            if (p.level > 2)
                continue;
            candidates++;
            if (first < 0)
                first = j;

            auto tmp = kpResize[p.frame].getAdjustedKeypoint(p);
            const double query_pt[2] = {static_cast<double>(tmp.x), static_cast<double>(tmp.y)};
            std::vector<size_t> ret_index(1);
            std::vector<double> out_dist_sqr(1);
            int nRes = levelKeypointKD[p.frame]->knnSearch(&query_pt[0], 1, &ret_index[0], &out_dist_sqr[0]);
            if (nRes == 0 || out_dist_sqr[0] < insertKeypointRadius * insertKeypointRadius) {
                good++;
            }
        }
        if (candidates < 2 || good / candidates < 0.75) {
            continue;
        }

        // Create new point
        Eigen::Vector4f posWorld = transformFromModelToWorld * point.getEigen4f();
        auto *p = new MapPoint();
        p->v3WorldPos[0] = posWorld.x();
        p->v3WorldPos[1] = posWorld.y();
        p->v3WorldPos[2] = posWorld.z();
        // Check if any already inserted point is too close
        if (!mMap.AddPoint(p, mdOneCM)) {
            delete p;
            continue;
        }

        p->bFromModel = true;
        auto center = kpResize[point.imagePoints[first].frame].getAdjustedKeypoint(point.imagePoints[first]);
        p->SetSourcePatch(mCamera, mMap.vpKeyFrames[point.imagePoints[first].frame], 0, center, LevelZeroPos(center, 0));

        // Add to calibration points
        modelCalPoints.push_back(p);
        modelRefPoints.push_back(posWorld);

        // Construct measurements and insert into relevant DBs
        for (int j = 0; j < point.imagePoints.size(); j++) {
            if (point.imagePoints[j].level > 2)
                continue;
            Measurement m;
            m.nLevel = 0;
            m.Source = j == first ? Measurement::SRC_ROOT : Measurement::SRC_TRAIL;
            m.v2RootPos = vec(kpResize[point.imagePoints[j].frame].getAdjustedKeypoint(point.imagePoints[j]));
            m.bSubPix = true;
            mMap.vpKeyFrames[point.imagePoints[j].frame]->mMeasurements[p] = m;
            p->pMMData->sMeasurementKFs.insert(mMap.vpKeyFrames[point.imagePoints[j].frame]);
        }
    }
    for (const auto &kd : levelKeypointKD) {
        delete kd;
    }

    // Thin keyframes - Order KFs by number of measurements
    std::vector<std::pair<KeyFrame*, int>> measuresPerKF;
    for (int i = 0; i < mMap.vpKeyFrames.size(); i++) {
        measuresPerKF.push_back(std::pair<KeyFrame*,int>(mMap.vpKeyFrames[i], mMap.vpKeyFrames[i]->mMeasurements.size()));
    }
    std::sort(measuresPerKF.begin(), measuresPerKF.end(), [](auto &left, auto &right) {
        return left.second < right.second;
    });
    // Start by removing KFs with few measurements
    for (const auto &mpkf : measuresPerKF) {
        auto kf = mpkf.first;
        if (kf->bFixed)
            continue;
        auto neighbours = MapMaker::NClosestKeyFramesInList(*kf, 2, mMap.vpKeyFrames);

        if (neighbours.size() == 2) {
            auto d1 = KeyFrameLinearDist(*kf, *neighbours[0]);
            auto d2 = KeyFrameLinearDist(*kf, *neighbours[1]);
            if (d1 < minKFDistance * mdOneCM && d2 < minKFDistance * mdOneCM) {
                if (!RemoveKeyFrame(kf)) {
                    std::cout << "MapMaker: failed to remove keyframe" << std::endl;
                    return false;
                }
            }
        }
    }

    // Make first bundle adjust
    mbBundleConverged_Full = false;
    mbBundleConverged_Recent = false;
    while (!mbBundleConverged_Full) {
        BundleAdjustAll(false);
        if (mbResetRequested)
            return false;
    }

    // Estimate the feature depth distribution and add more points
    int newPointsPerKF = mMap.vpPoints.size() / mMap.vpKeyFrames.size();
    std::vector<int> kfIDs;
    for (int i = 0; i < mMap.vpKeyFrames.size(); i++)
        kfIDs.push_back(i);
    for (int i : kfIDs) {
        RefreshSceneDepth(mMap.vpKeyFrames[i]);
        AddSomeMapPoints(3, i, newPointsPerKF);
        AddSomeMapPoints(2, i, newPointsPerKF/3);
        AddSomeMapPoints(1, i, newPointsPerKF/2);
        ReFindNewlyMade();
        BundleAdjustKeyframe(i);
    }

    mbBundleConverged_Full = false;
    mbBundleConverged_Recent = false;
    while (!mbBundleConverged_Full) {
        BundleAdjustAll(false);
        if (mbResetRequested)
            return false;
    }

    // Reverse transformation caused by bundle adjust and transform to reference coordinates
    {
        Eigen::MatrixXf target;
        target.resize(3, modelCalPoints.size());
        Eigen::MatrixXf now;
        now.resize(3, modelCalPoints.size());
        for (int i = 0; i < modelCalPoints.size(); i++) {
            auto m = modelCalPoints[i]->v3WorldPos;
            now.col(i) = Eigen::Vector3f(m[0], m[1], m[2]);
            target.col(i) = modelRefPoints[i].head<3>();
        }
        ApplyGlobalTransformationToMap(Eigen::umeyama(now, target));
    }

    // Prepare data for relocalization
    BuildRelocIndex();

    double meanDepth = 0;
    for (auto & kf : mMap.vpKeyFrames) {
        RefreshSceneDepth(kf);
        meanDepth += kf->dSceneDepthMean;
    }
    mdWiggleScaleDepthNormalized = mdWiggleScale / (meanDepth / mMap.vpKeyFrames.size());

    mMap.bGood = true;
    se3TrackerPose = mMap.vpKeyFrames[0]->se3CfromW;
    stats.AddLoadedModel(mMap.vpKeyFrames.size(), mMap.vpPoints.size());
    return true;
}

// ThinCandidates() Thins out a key-frame's candidate list.
// Candidates are those salient corners where the mapmaker will attempt
// to make a new map point by epipolar search. We don't want to make new points
// where there are already existing map points, this routine erases such candidates.
// Operates on a single level of a keyframe.
void MapMaker::ThinCandidates(KeyFrame &k, int nLevel) {
    std::vector<Candidate> &vCSrc = k.aLevels[nLevel].vCandidates;
    std::vector<Candidate> vCGood;
    std::vector<CVD::ImageRef> irBusyLevelPos;
    // Make a list of `busy' image locations, which already have features at the same level
    // or at one level higher.
    for (meas_it it = k.mMeasurements.begin(); it != k.mMeasurements.end(); it++) {
        if (!(it->second.nLevel == nLevel || it->second.nLevel == nLevel + 1))
            continue;
        irBusyLevelPos.push_back(ir_rounded(it->second.v2RootPos / LevelScale(nLevel)));
    }

    // Only keep those candidates further than 20 pixels away from busy positions.
    unsigned int nMinMagSquared = (insertKeypointRadius*2) / LevelScale(nLevel);
    nMinMagSquared = nMinMagSquared * nMinMagSquared;
    for (unsigned int i = 0; i < vCSrc.size(); i++) {
        CVD::ImageRef irC = vCSrc[i].irLevelPos;
        bool bGood = true;
        for (unsigned int j = 0; j < irBusyLevelPos.size(); j++) {
            CVD::ImageRef irB = irBusyLevelPos[j];
            if ((irB - irC).mag_squared() < nMinMagSquared) {
                bGood = false;
                break;
            }
        }
        if (bGood)
            vCGood.push_back(vCSrc[i]);
    }
    vCSrc = vCGood;
}

// Adds map points by epipolar search to the last-added key-frame, at a single
// specified pyramid level. Does epipolar search in the target keyframe as closest by
// the ClosestKeyFrame function.
void MapMaker::AddSomeMapPoints(int nLevel) {
    AddSomeMapPoints(nLevel, mMap.vpKeyFrames.size() - 1, 0);
};

int MapMaker::AddSomeMapPoints(int nLevel, int kfID, int limit) {
    KeyFrame &kSrc = *(mMap.vpKeyFrames[kfID]); // The new keyframe
    KeyFrame &kTarget = *(ClosestKeyFrame(kSrc));
    Level &l = kSrc.aLevels[nLevel];

    ThinCandidates(kSrc, nLevel);

    unsigned int nMinMagSquared = (insertKeypointRadius*2) / LevelScale(nLevel);
    nMinMagSquared = nMinMagSquared * nMinMagSquared;
    std::vector<CVD::ImageRef> used;
    int c = 0;
    for (unsigned int i = 0; i < l.vCandidates.size() && (limit == 0 || c < limit); i++) {
        bool good = true;
        for (const auto &p : used) {
            if ((p - l.vCandidates[i].irLevelPos).mag_squared() < nMinMagSquared) {
                good = false;
                break;
            }
        }
        if (good && AddPointEpipolar(kSrc, kTarget, nLevel, i)) {
            used.push_back(l.vCandidates[i].irLevelPos);
            c++;
        }
    }
    return c;
};

void MapMaker::ApplyGlobalTransformationToMap(const Eigen::Matrix<float, 4, 4>& trans) {
    for (const auto & vpKeyFrame : mMap.vpKeyFrames) {
        auto old = vpKeyFrame->se3CfromW;
        auto oldR = old.get_rotation().get_matrix();
        auto oldT = old.get_translation();
        Eigen::Vector3f T(oldT[0], oldT[1], oldT[2]);
        Eigen::Matrix3f R;
        R.row(0) = Eigen::Vector3f(oldR(0, 0), oldR(0, 1), oldR(0, 2));
        R.row(1) = Eigen::Vector3f(oldR(1, 0), oldR(1, 1), oldR(1, 2));
        R.row(2) = Eigen::Vector3f(oldR(2, 0), oldR(2, 1), oldR(2, 2));
        Eigen::Matrix<float, 4, 4> oldTrans;
        oldTrans.block<3,3>(0,0) = R;
        oldTrans.block<3,1>(0,3) = T;

        auto newTrans = oldTrans * trans.inverse();

        R = newTrans.block<3,3>(0,0);
        T = newTrans.block<3,1>(0,3);
        Vector<3,float> finalT;
        finalT[0] = T.x();
        finalT[1] = T.y();
        finalT[2] = T.z();
        auto newAngleAxis = Eigen::AngleAxisf(R);
        auto x = newAngleAxis.axis() * newAngleAxis.angle();
        Vector<3,float> finalR;
        finalR[0] = x.x();
        finalR[1] = x.y();
        finalR[2] = x.z();

        vpKeyFrame->se3CfromW = SE3<>(SO3<>(finalR), finalT);
    }

    for (const auto &mp : mMap.vpPoints) {
        Eigen::Vector4f p(mp->v3WorldPos[0], mp->v3WorldPos[1], mp->v3WorldPos[2], 1);
        p = trans * p;
        auto tmp = p.head<3>();
        mp->v3WorldPos[0] = tmp.x();
        mp->v3WorldPos[1] = tmp.y();
        mp->v3WorldPos[2] = tmp.z();
        mp->RefreshPixelVectors();
    }
}

// The tracker entry point for adding a new keyframe;
// the tracker thread doesn't want to hang about, so
// just dumps it on the top of the mapmaker's queue to
// be dealt with later, and return.
void MapMaker::AddKeyFrame(KeyFrame &k) {
    KeyFrame *pK = new KeyFrame;
    *pK = k;
    mvpKeyFrameQueue.push_back(pK);
    if (mbBundleRunning)   // Tell the mapmaker to stop doing low-priority stuff and concentrate on this KF first.
        mbBundleAbortRequested = true;
}

// Mapmaker's code to handle incoming key-frames.
void MapMaker::AddKeyFrameFromTopOfQueue() {
    if (mvpKeyFrameQueue.size() == 0)
        return;

    KeyFrame *pK = mvpKeyFrameQueue[0];
    mvpKeyFrameQueue.erase(mvpKeyFrameQueue.begin());
    pK->MakeKeyFrame_Rest();
    mMap.vpKeyFrames.push_back(pK);
    // Any measurements? Update the relevant point's measurement counter status map
    for (meas_it it = pK->mMeasurements.begin(); it != pK->mMeasurements.end(); it++) {
        it->first->pMMData->sMeasurementKFs.insert(pK);
        it->second.Source = Measurement::SRC_TRACKER;
    }

    // And maybe we missed some - this now adds to the map itself, too.
    ReFindInSingleKeyFrame(*pK);

    AddSomeMapPoints(3);       // .. and add more map points by epipolar search.
    AddSomeMapPoints(0);
    AddSomeMapPoints(1);
    AddSomeMapPoints(2);

    mbBundleConverged_Full = false;
    mbBundleConverged_Recent = false;

    pK->MakeKeyFrame_Reloc(nRelocFeatureCount, insertKeypointRadius);
    keyframeImageMatcher.add(pK->relocFrameDescriptor);
    keyframeImageMatcher.train();

    std::vector<cv::Mat> feature;
    feature.resize(pK->relocFrameDescriptor.rows);
    for(int i = 0; i < pK->relocFrameDescriptor.rows; ++i)
        feature[i] = pK->relocFrameDescriptor.row(i);
    if (operationMode == MM_MODE_FULL_AUTO) {
        relocDBoW.add(feature);
        relocFeatureToModel.push_back(currentModel);
    }
    ReFindNewlyMade();

    mKeyframeAdded = true;
}

// Tries to make a new map point out of a single candidate point
// by searching for that point in another keyframe, and triangulating
// if a match is found.
bool MapMaker::AddPointEpipolar(KeyFrame &kSrc,
                                KeyFrame &kTarget,
                                int nLevel,
                                int nCandidate) {
    static Image<Vector<2> > imUnProj;
    static bool bMadeCache = false;
    if (!bMadeCache) {
        imUnProj.resize(kSrc.aLevels[0].im.size());
        CVD::ImageRef ir;
        do imUnProj[ir] = mCamera.UnProject(ir);
        while (ir.next(imUnProj.size()));
        bMadeCache = true;
    }

    int nLevelScale = LevelScale(nLevel);
    Candidate &candidate = kSrc.aLevels[nLevel].vCandidates[nCandidate];
    CVD::ImageRef irLevelPos = candidate.irLevelPos;
    Vector<2> v2RootPos = LevelZeroPos(irLevelPos, nLevel);

    Vector<3> v3Ray_SC = unproject(mCamera.UnProject(v2RootPos));
    normalize(v3Ray_SC);
    Vector<3> v3LineDirn_TC = kTarget.se3CfromW.get_rotation() * (kSrc.se3CfromW.get_rotation().inverse() * v3Ray_SC);

    // Restrict epipolar search to a relatively narrow depth range
    // to increase reliability
    double dMean = kSrc.dSceneDepthMean;
    double dSigma = kSrc.dSceneDepthSigma;
    double dStartDepth = std::max(mdWiggleScale, dMean - dSigma);
    double dEndDepth = std::min(40 * mdWiggleScale, dMean + dSigma);

    Vector<3> v3CamCenter_TC = kTarget.se3CfromW * kSrc.se3CfromW.inverse().get_translation(); // The camera end
    Vector<3> v3RayStart_TC =
            v3CamCenter_TC + dStartDepth * v3LineDirn_TC;                               // the far-away end
    Vector<3> v3RayEnd_TC =
            v3CamCenter_TC + dEndDepth * v3LineDirn_TC;                               // the far-away end


    // it's highly unlikely that we'll manage to get anything out if we're facing backwards wrt the other camera's view-ray
    if (v3RayEnd_TC[2] <= v3RayStart_TC[2]) {
        return false;
    }

    if (v3RayEnd_TC[2] <= 0.0) return false;
    if (v3RayStart_TC[2] <= 0.0)
        v3RayStart_TC += v3LineDirn_TC * (0.001 - v3RayStart_TC[2] / v3LineDirn_TC[2]);

    Vector<2> v2A = project(v3RayStart_TC);
    Vector<2> v2B = project(v3RayEnd_TC);
    Vector<2> v2AlongProjectedLine = v2A - v2B;

    if (v2AlongProjectedLine * v2AlongProjectedLine < 0.00000001) {
        return false;
    }
    normalize(v2AlongProjectedLine);
    Vector<2> v2Normal;
    v2Normal[0] = v2AlongProjectedLine[1];
    v2Normal[1] = -v2AlongProjectedLine[0];

    double dNormDist = v2A * v2Normal;
    if (fabs(dNormDist) > mCamera.LargestRadiusInImage())
        return false;

    double dMinLen = std::min(v2AlongProjectedLine * v2A, v2AlongProjectedLine * v2B) - 0.05;
    double dMaxLen = std::max(v2AlongProjectedLine * v2A, v2AlongProjectedLine * v2B) + 0.05;
    if (dMinLen < -2.0) dMinLen = -2.0;
    if (dMaxLen < -2.0) dMaxLen = -2.0;
    if (dMinLen > 2.0) dMinLen = 2.0;
    if (dMaxLen > 2.0) dMaxLen = 2.0;

    // Find current-frame corners which might match this
    PatchFinder Finder;
    Finder.MakeTemplateCoarseNoWarp(kSrc, nLevel, irLevelPos);
    if (Finder.TemplateBad()) return false;

    std::vector<Vector<2>> &vv2Corners = kTarget.aLevels[nLevel].vImplaneCorners;
    std::vector<CVD::ImageRef> &vIR = kTarget.aLevels[nLevel].vCorners;
    if (!kTarget.aLevels[nLevel].bImplaneCornersCached) {
        // over all corners in target img..
        for (unsigned int i = 0; i < vIR.size(); i++) {
            auto pos = ir(LevelZeroPos(vIR[i], nLevel));
            if (pos.x >= 0 && pos.x < imUnProj.size()[0] && pos.y >= 0 && pos.y < imUnProj.size()[1]) {
                vv2Corners.push_back(imUnProj[pos]);
            }
        }
        kTarget.aLevels[nLevel].bImplaneCornersCached = true;
    }

    int nBest = -1;
    int nBestZMSSD = Finder.mnMaxSSD + 1;
    double dMaxDistDiff = mCamera.OnePixelDist() * (4.0 + 1.0 * nLevelScale);
    double dMaxDistSq = dMaxDistDiff * dMaxDistDiff;

    for (unsigned int i = 0; i < vv2Corners.size(); i++)   // over all corners in target img..
    {
        Vector<2> v2Im = vv2Corners[i];
        double dDistDiff = dNormDist - v2Im * v2Normal;
        if (dDistDiff * dDistDiff > dMaxDistSq) continue; // skip if not along epi line
        if (v2Im * v2AlongProjectedLine < dMinLen) continue; // skip if not far enough along line
        if (v2Im * v2AlongProjectedLine > dMaxLen) continue; // or too far
        int nZMSSD = Finder.ZMSSDAtPoint(kTarget.aLevels[nLevel].im, vIR[i]);
        if (nZMSSD < nBestZMSSD) {
            nBest = i;
            nBestZMSSD = nZMSSD;
        }
    }

    if (nBest == -1) return false;   // Nothing found.

    //  Found a likely candidate along epipolar ray
    Finder.MakeSubPixTemplate();
    Finder.SetSubPixPos(LevelZeroPos(vIR[nBest], nLevel));
    bool bSubPixConverges = Finder.IterateSubPixToConvergence(kTarget, 10);
    if (!bSubPixConverges)
        return false;

    // Now triangulate the 3d point...
    Vector<3> v3New;
    v3New = kTarget.se3CfromW.inverse() *
            ReprojectPoint(kSrc.se3CfromW * kTarget.se3CfromW.inverse(),
                           mCamera.UnProject(v2RootPos),
                           mCamera.UnProject(Finder.GetSubPixPos()));

    MapPoint *pNew = new MapPoint;
    pNew->v3WorldPos = v3New;
    mMap.vpPoints.push_back(pNew);

    pNew->pMMData = new MapMakerData();
    pNew->SetSourcePatch(mCamera, &kSrc, nLevel, irLevelPos, v2RootPos);

    mqNewQueue.push(pNew);
    Measurement m;
    m.v2RootPos = v2RootPos;
    m.nLevel = nLevel;
    m.bSubPix = true;
    kSrc.mMeasurements[pNew] = m;

    m.Source = Measurement::SRC_EPIPOLAR;
    m.v2RootPos = Finder.GetSubPixPos();
    kTarget.mMeasurements[pNew] = m;
    pNew->pMMData->sMeasurementKFs.insert(&kSrc);
    pNew->pMMData->sMeasurementKFs.insert(&kTarget);
    return true;
}

double MapMaker::KeyFrameLinearDist(KeyFrame &k1, KeyFrame &k2) {
    Vector<3> v3KF1_CamPos = k1.se3CfromW.inverse().get_translation();
    Vector<3> v3KF2_CamPos = k2.se3CfromW.inverse().get_translation();
    Vector<3> v3Diff = v3KF2_CamPos - v3KF1_CamPos;
    double dDist = sqrt(v3Diff * v3Diff);
    return dDist;
}

std::vector<KeyFrame *> MapMaker::NClosestKeyFramesInList(KeyFrame &k, unsigned int N, std::vector<KeyFrame*> &list) {
    std::vector<std::pair<double, KeyFrame *>> vKFandScores;
    for (unsigned int i = 0; i < list.size(); i++) {
        if (list[i] == &k)
            continue;
        double dDist = KeyFrameLinearDist(k, *list[i]);
        vKFandScores.push_back(std::make_pair(dDist, list[i]));
    }
    if (N > vKFandScores.size())
        N = vKFandScores.size();
    partial_sort(vKFandScores.begin(), vKFandScores.begin() + N, vKFandScores.end());

    std::vector<KeyFrame *> vResult;
    for (unsigned int i = 0; i < N; i++)
        vResult.push_back(vKFandScores[i].second);
    return vResult;
}

std::vector<KeyFrame *> MapMaker::NClosestKeyFrames(KeyFrame &k, unsigned int N) {
    return NClosestKeyFramesInList(k, N, mMap.vpKeyFrames);
}

KeyFrame *MapMaker::ClosestKeyFrame(KeyFrame &k) {
    double dClosestDist = 9999999999.9;
    int nClosest = -1;
    for (unsigned int i = 0; i < mMap.vpKeyFrames.size(); i++) {
        if (mMap.vpKeyFrames[i] == &k)
            continue;
        double dDist = MapMaker::KeyFrameLinearDist(k, *mMap.vpKeyFrames[i]);
        if (dDist < dClosestDist) {
            dClosestDist = dDist;
            nClosest = i;
        }
    }
    assert(nClosest != -1);
    return mMap.vpKeyFrames[nClosest];
}

double MapMaker::DistToNearestKeyFrame(KeyFrame &kCurrent) {
    KeyFrame *pClosest = ClosestKeyFrame(kCurrent);
    double dDist = MapMaker::KeyFrameLinearDist(kCurrent, *pClosest);
    return dDist;
}

bool MapMaker::NeedNewKeyFrame(KeyFrame &kCurrent) {
    KeyFrame *pClosest = ClosestKeyFrame(kCurrent);
    double dDist = MapMaker::KeyFrameLinearDist(kCurrent, *pClosest);
    dDist *= (1.0 / kCurrent.dSceneDepthMean);
    if (dDist > GV2.GetDouble("MapMaker.MaxKFDistWiggleMult", 2, SILENT) * mdWiggleScaleDepthNormalized)
        return true;
    return false;
}

// Perform bundle adjustment on all keyframes, all map points
void MapMaker::BundleAdjustAll(bool ignoreOut) {
    // construct the sets of kfs/points to be adjusted:
    // in this case, all of them
    std::set<KeyFrame *> sAdj;
    std::set<KeyFrame *> sFixed;
    for (unsigned int i = 0; i < mMap.vpKeyFrames.size(); i++)
        if (mMap.vpKeyFrames[i]->bFixed)
            sFixed.insert(mMap.vpKeyFrames[i]);
        else
            sAdj.insert(mMap.vpKeyFrames[i]);

    std::set<MapPoint *> sMapPoints;
    for (unsigned int i = 0; i < mMap.vpPoints.size(); i++)
        sMapPoints.insert(mMap.vpPoints[i]);

    BundleAdjust(sAdj, sFixed, sMapPoints, false, ignoreOut);
}

void MapMaker::BundleAdjustKeyframe(int idx) {
    std::set<KeyFrame *> sAdjustSet;
    KeyFrame *pkfSelected = mMap.vpKeyFrames[idx];
    sAdjustSet.insert(pkfSelected);
    std::vector<KeyFrame *> vClosest = NClosestKeyFrames(*pkfSelected, 4);
    for (int i = 0; i < 4; i++)
        if (!vClosest[i]->bFixed)
            sAdjustSet.insert(vClosest[i]);

    // Now we find the set of features which they contain.
    std::set<MapPoint *> sMapPoints;
    for (std::set<KeyFrame *>::iterator iter = sAdjustSet.begin(); iter != sAdjustSet.end(); iter++) {
        std::map<MapPoint *, Measurement> &mKFMeas = (*iter)->mMeasurements;
        for (meas_it jiter = mKFMeas.begin(); jiter != mKFMeas.end(); jiter++) {
            sMapPoints.insert(jiter->first);
        }
    };

    // Finally, add all keyframes which measure above points as fixed keyframes
    std::set<KeyFrame *> sFixedSet;
    for (std::vector<KeyFrame *>::iterator it = mMap.vpKeyFrames.begin(); it != mMap.vpKeyFrames.end(); it++) {
        if (sAdjustSet.count(*it))
            continue;
        bool bInclude = false;
        for (meas_it jiter = (*it)->mMeasurements.begin(); jiter != (*it)->mMeasurements.end(); jiter++) {
            if (sMapPoints.count(jiter->first)) {
                bInclude = true;
                break;
            }
        }
        if (bInclude)
            sFixedSet.insert(*it);
    }

    BundleAdjust(sAdjustSet, sFixedSet, sMapPoints, true);
}

// Peform a local bundle adjustment which only adjusts
// recently added key-frames
void MapMaker::BundleAdjustRecent() {
    BundleAdjustKeyframe(mMap.vpKeyFrames.size()-1);
}

// Common bundle adjustment code. This creates a bundle-adjust instance, populates it, and runs it.
void
MapMaker::BundleAdjust(std::set<KeyFrame *> sAdjustSet, std::set<KeyFrame *> sFixedSet, std::set<MapPoint *> sMapPoints,
                       bool bRecent, bool bIgnoreOut) {
    Bundle b(mCamera);   // Our bundle adjuster
    mbBundleRunning = true;
    mbBundleRunningIsRecent = bRecent;

    // The bundle adjuster does different accounting of keyframes and map points;
    // Translation maps are stored:
    std::map<MapPoint *, int> mPoint_BundleID;
    std::map<int, MapPoint *> mBundleID_Point;
    std::map<KeyFrame *, int> mView_BundleID;
    std::map<int, KeyFrame *> mBundleID_View;

    // Add the keyframes' poses to the bundle adjuster. Two parts: first nonfixed, then fixed.
    for (std::set<KeyFrame *>::iterator it = sAdjustSet.begin(); it != sAdjustSet.end(); it++) {
        int nBundleID = b.AddCamera((*it)->se3CfromW, (*it)->bFixed);
        mView_BundleID[*it] = nBundleID;
        mBundleID_View[nBundleID] = *it;
    }
    for (std::set<KeyFrame *>::iterator it = sFixedSet.begin(); it != sFixedSet.end(); it++) {
        int nBundleID = b.AddCamera((*it)->se3CfromW, true);
        mView_BundleID[*it] = nBundleID;
        mBundleID_View[nBundleID] = *it;
    }

    // Add the points' 3D position
    for (std::set<MapPoint *>::iterator it = sMapPoints.begin(); it != sMapPoints.end(); it++) {
        int nBundleID = b.AddPoint((*it)->v3WorldPos);
        mPoint_BundleID[*it] = nBundleID;
        mBundleID_Point[nBundleID] = *it;
    }

    // Add the relevant point-in-keyframe measurements
    for (unsigned int i = 0; i < mMap.vpKeyFrames.size(); i++) {
        if (mView_BundleID.count(mMap.vpKeyFrames[i]) == 0)
            continue;

        int nKF_BundleID = mView_BundleID[mMap.vpKeyFrames[i]];
        for (meas_it it = mMap.vpKeyFrames[i]->mMeasurements.begin(); it != mMap.vpKeyFrames[i]->mMeasurements.end(); it++) {
            if (mPoint_BundleID.count(it->first) == 0)
                continue;
            int nPoint_BundleID = mPoint_BundleID[it->first];
            b.AddMeas(nKF_BundleID, nPoint_BundleID, it->second.v2RootPos,
                      LevelScale(it->second.nLevel) * LevelScale(it->second.nLevel));
        }
    }

    // Run the bundle adjuster. This returns the number of successful iterations
    int nAccepted = b.Compute(&mbBundleAbortRequested);

    if (nAccepted < 0) {
        // Crap: - LM Ran into a serious problem!
        // This is probably because the initial stereo was messed up.
        // Get rid of this map and start again!
        std::cout << "!! MapMaker: Cholesky failure in bundle adjust. " << std::endl
                  << "   The map is probably corrupt: Ditching the map. " << std::endl;
        mbResetRequested = true;
        return;
    }

    // Bundle adjustment did some updates, apply these to the map
    if (nAccepted > 0) {

        for (std::map<MapPoint *, int>::iterator itr = mPoint_BundleID.begin(); itr != mPoint_BundleID.end(); itr++)
            itr->first->v3WorldPos = b.GetPoint(itr->second);

        for (std::map<KeyFrame *, int>::iterator itr = mView_BundleID.begin(); itr != mView_BundleID.end(); itr++)
            itr->first->se3CfromW = b.GetCamera(itr->second);
        if (bRecent)
            mbBundleConverged_Recent = false;
        mbBundleConverged_Full = false;
    };

    if (b.Converged()) {
        mbBundleConverged_Recent = true;
        if (!bRecent)
            mbBundleConverged_Full = true;
    }

    mbBundleRunning = false;
    mbBundleAbortRequested = false;

    if (bIgnoreOut)
        return;
    // Handle outlier measurements:
    std::vector<std::pair<int, int>> vOutliers_PC_pair = b.GetOutlierMeasurements();
    for (unsigned int i = 0; i < vOutliers_PC_pair.size(); i++) {
        MapPoint *pp = mBundleID_Point[vOutliers_PC_pair[i].first];
        KeyFrame *pk = mBundleID_View[vOutliers_PC_pair[i].second];
        Measurement &m = pk->mMeasurements[pp];
        // Is the original source kf considered an outlier? That's bad.
        if (!pp->bFromModel && (pp->pMMData->GoodMeasCount() <= 2 || m.Source == Measurement::SRC_ROOT)) {
            pp->bBad = true;
        } else {
            // Do we retry it? Depends where it came from!!
            if (m.Source == Measurement::SRC_TRACKER || m.Source == Measurement::SRC_EPIPOLAR)
                mvFailureQueue.push_back(std::pair<KeyFrame *, MapPoint *>(pk, pp));
            else
                pp->pMMData->sNeverRetryKFs.insert(pk);
            pk->mMeasurements.erase(pp);
            pp->pMMData->sMeasurementKFs.erase(pk);
        }
    }
}

// Mapmaker's try-to-find-a-point-in-a-keyframe code. This is used to update
// data association if a bad measurement was detected, or if a point
// was never searched for in a keyframe in the first place. This operates
// much like the tracker! So most of the code looks just like in
// TrackerData.h.
bool MapMaker::ReFind_Common(KeyFrame &k, MapPoint &p) {
    // abort if either a measurement is already in the map, or we've
    // decided that this point-kf combo is beyond redemption
    if (p.pMMData->sMeasurementKFs.count(&k)
        || p.pMMData->sNeverRetryKFs.count(&k))
        return false;

    static PatchFinder Finder;
    Vector<3> v3Cam = k.se3CfromW * p.v3WorldPos;
    if (v3Cam[2] < 0.001) {
        p.pMMData->sNeverRetryKFs.insert(&k);
        return false;
    }
    Vector<2> v2ImPlane = project(v3Cam);
    if (v2ImPlane * v2ImPlane > mCamera.LargestRadiusInImage() * mCamera.LargestRadiusInImage()) {
        p.pMMData->sNeverRetryKFs.insert(&k);
        return false;
    }

    Vector<2> v2Image = mCamera.Project(v2ImPlane);
    if (mCamera.Invalid()) {
        p.pMMData->sNeverRetryKFs.insert(&k);
        return false;
    }

    CVD::ImageRef irImageSize = k.aLevels[0].im.size();
    if (v2Image[0] < 0 || v2Image[1] < 0 || v2Image[0] > irImageSize[0] || v2Image[1] > irImageSize[1]) {
        p.pMMData->sNeverRetryKFs.insert(&k);
        return false;
    }

    Matrix<2> m2CamDerivs = mCamera.GetProjectionDerivs();
    Finder.MakeTemplateCoarse(p, k.se3CfromW, m2CamDerivs);

    if (Finder.TemplateBad()) {
        p.pMMData->sNeverRetryKFs.insert(&k);
        return false;
    }

    bool bFound = Finder.FindPatchCoarse(ir(v2Image), k, 4);  // Very tight search radius!
    if (!bFound) {
        p.pMMData->sNeverRetryKFs.insert(&k);
        return false;
    }

    // If we found something, generate a measurement struct and put it in the map
    Measurement m;
    m.nLevel = Finder.GetLevel();
    m.Source = Measurement::SRC_REFIND;

    if (Finder.GetLevel() > 0) {
        Finder.MakeSubPixTemplate();
        Finder.IterateSubPixToConvergence(k, 8);
        m.v2RootPos = Finder.GetSubPixPos();
        m.bSubPix = true;
    } else {
        m.v2RootPos = Finder.GetCoarsePosAsVector();
        m.bSubPix = false;
    };

    if (k.mMeasurements.count(&p)) {
        assert(0); // This should never happen, we checked for this at the start.
    }
    k.mMeasurements[&p] = m;
    p.pMMData->sMeasurementKFs.insert(&k);
    return true;
}

// A general data-association update for a single keyframe
// Do this on a new key-frame when it's passed in by the tracker
int MapMaker::ReFindInSingleKeyFrame(KeyFrame &k) {
    std::vector<MapPoint *> vToFind;
    for (unsigned int i = 0; i < mMap.vpPoints.size(); i++)
        vToFind.push_back(mMap.vpPoints[i]);

    int nFoundNow = 0;
    for (unsigned int i = 0; i < vToFind.size(); i++)
        if (ReFind_Common(k, *vToFind[i]))
            nFoundNow++;

    return nFoundNow;
};

// When new map points are generated, they're only created from a stereo pair
// this tries to make additional measurements in other KFs which they might
// be in.
void MapMaker::ReFindNewlyMade() {
    if (mqNewQueue.empty())
        return;
    int nFound = 0;
    int nBad = 0;
    while (!mqNewQueue.empty() && mvpKeyFrameQueue.size() == 0) {
        MapPoint *pNew = mqNewQueue.front();
        mqNewQueue.pop();
        if (pNew->bBad) {
            nBad++;
            continue;
        }
        for (unsigned int i = 0; i < mMap.vpKeyFrames.size(); i++) {
            if (ReFind_Common(*mMap.vpKeyFrames[i], *pNew))
                nFound++;
        }
    }
};

// Dud measurements get a second chance.
void MapMaker::ReFindFromFailureQueue() {
    if (mvFailureQueue.size() == 0)
        return;
    sort(mvFailureQueue.begin(), mvFailureQueue.end());
    std::vector<std::pair<KeyFrame *, MapPoint *> >::iterator
            it;
    int nFound = 0;
    for (it = mvFailureQueue.begin(); it != mvFailureQueue.end(); it++) {
        if (ReFind_Common(*it->first, *it->second))
            nFound++;
    }

    mvFailureQueue.erase(mvFailureQueue.begin(), it);
};

// Is the tracker's camera pose in cloud-cuckoo land?
bool MapMaker::IsDistanceToNearestKeyFrameExcessive(KeyFrame &kCurrent) {
    return DistToNearestKeyFrame(kCurrent) > mdWiggleScale * 10.0;
}

bool MapMaker::IsDistanceToRelocKeyFrameExcessive(SE3<> &camPose, KeyFrame &kTarget) {
    Vector<3> v3KF1_CamPos = camPose.inverse().get_translation();
    Vector<3> v3KF2_CamPos = kTarget.se3CfromW.inverse().get_translation();
    Vector<3> v3Diff = v3KF2_CamPos - v3KF1_CamPos;
    double dDist = sqrt(v3Diff * v3Diff);
    return dDist > GV2.GetDouble("MapMaker.MaxRelocKFDistance", 3.0, SILENT) * minKFDistance * mdOneCM;
}

// Calculates the depth(z-) distribution of map points visible in a keyframe
// This function is only used for the first two keyframes - all others
// get this filled in by the tracker
void MapMaker::RefreshSceneDepth(KeyFrame *pKF) {
    double dSumDepth = 0.0;
    double dSumDepthSquared = 0.0;
    int nMeas = 0;
    for (meas_it it = pKF->mMeasurements.begin(); it != pKF->mMeasurements.end(); it++) {
        MapPoint &point = *it->first;
        Vector<3> v3PosK = pKF->se3CfromW * point.v3WorldPos;
        dSumDepth += v3PosK[2];
        dSumDepthSquared += v3PosK[2] * v3PosK[2];
        nMeas++;
    }

    assert(nMeas > 2); // If not then something is seriously wrong with this KF!!
    pKF->dSceneDepthMean = dSumDepth / nMeas;
    pKF->dSceneDepthSigma = sqrt((dSumDepthSquared / nMeas) - (pKF->dSceneDepthMean) * (pKF->dSceneDepthMean));
}

void MapMaker::GUICommandCallBack(void *ptr, std::string sCommand, std::string sParams) {
    Command c;
    c.sCommand = sCommand;
    c.sParams = sParams;
    ((MapMaker *) ptr)->mvQueuedCommands.push_back(c);
}

void MapMaker::GUICommandHandler(std::string sCommand, std::string sParams)  // Called by the callback func..
{
    if (sCommand == "SaveMap") {
        std::cout << "  MapMaker: Saving the map.... " << std::endl;
        std::ofstream ofs("map.dump");
        for (unsigned int i = 0; i < mMap.vpPoints.size(); i++) {
            ofs << mMap.vpPoints[i]->v3WorldPos << "  ";
            ofs << mMap.vpPoints[i]->nSourceLevel << std::endl;
        }
        ofs.close();

        for (unsigned int i = 0; i < mMap.vpKeyFrames.size(); i++) {
            std::ostringstream ost1;
            ost1 << "keyframes/" << i << ".jpg";
            std::ostringstream ost2;
            ost2 << "keyframes/" << i << ".info";
            std::ofstream ofs2;
            ofs2.open(ost2.str().c_str());
            ofs2 << mMap.vpKeyFrames[i]->se3CfromW << std::endl;
            ofs2.close();
        }
        std::cout << "  ... done saving map." << std::endl;
        return;
    }

    std::cout << "! MapMaker::GUICommandHandler: unhandled command " << sCommand << std::endl;
    exit(1);
};

void MapMaker::BuildRelocIndex() {
    // Build descriptors
    for (const auto &kf : mMap.vpKeyFrames) {
        kf->MakeKeyFrame_Reloc(nRelocFeatureCount, insertKeypointRadius);
        keyframeImageMatcher.add(kf->relocFrameDescriptor);
    }
    keyframeImageMatcher.train();
}

void MapMaker::AddRelocImage(KeyFrame &k) {
    std::unique_lock<std::mutex> lock(relocImageMutex);

    Image<CVD::byte> im = k.aLevels[0].im;
    cv::Mat img(im.size().y, im.size().x, CV_8UC1, im.data());
    lastRelocImage = img;
    newRelocImage = true;

    lock.unlock();
}

SE3<> MapMaker::LastRelocPose() {
    newRelocPose = false;
    return lastRelocPose;
}

bool MapMaker::NewRelocPoseReady() {
    return newRelocPose;
}

int MapMaker::BestRelocKeyFrame() {
    return nBestRelocKF;
}

void MapMaker::ProcessReloc() {
    if (!newRelocImage)
        return;

    // Get last frame
    std::unique_lock<std::mutex> lock(relocImageMutex);
    cv::Mat img = lastRelocImage;
    newRelocImage = false;
    lock.unlock();

    // Extract features
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat curentFrameDescriptor;
    auto detector = cv::ORB::create(nRelocFeatureCount, 1.25f, 8, 31, 0, 2);
    detector->detectAndCompute(img, cv::noArray(), keypoints, curentFrameDescriptor);


    if (operationMode == MM_MODE_FULL_AUTO) {
        DBoW2::QueryResults ret;
        std::vector<cv::Mat> feature;
        feature.resize(curentFrameDescriptor.rows);

        for(int i = 0; i < curentFrameDescriptor.rows; ++i) {
            feature[i] = curentFrameDescriptor.row(i);
        }
        relocDBoW.query(feature, ret, 5);

        std::string nextModel;
        if (ret.size() >= 2) {
            std::string best = relocFeatureToModel[ret[0].Id];
            if (best != currentModel) {
                if (ret.size() == 5) {
                    std::map<std::string, int> results;
                    for (const auto &r : ret)
                        results[relocFeatureToModel[r.Id]]++;

                    if (results.find(currentModel) != results.end()) {
                        best = currentModel;
                    } else if (results.size() == 1) {
                        nextModel = results.begin()->first;
                    } else if (results.size() == 2) {
                        for (const auto &r : results) {
                            if (r.second > 2 && r.first == best)
                                nextModel = r.first;
                        }
                    }
                }
            }
        }
        if (last10RelocModels.size() >= 10) {
            last10RelocModels.erase(last10RelocModels.begin());
        }
        last10RelocModels.push_back(nextModel);
        if (!nextModel.empty()) {
            double matchNew = 0;
            for (const auto &m : last10RelocModels) {
                if (m == nextModel) {
                    matchNew++;
                }
            }
            if (matchNew / last10RelocModels.size() < 0.8)
                nextModel = currentModel;
        }
        if (nextModel.empty() && currentModel.empty()) {
            return;
        }

        if (!nextModel.empty() && (nextModel != currentModel || !mMap.IsGood())) {
            SE3<> tmp;
            currentModelName = "";
            currentModel = "";
            if (LoadModelFromFolder(nextModel, CVD::ImageRef(img.cols, img.rows), tmp)) {
                currentModel = nextModel;
            }
            return;
        }
    }

    std::vector<std::vector<cv::DMatch>> matches;
    keyframeImageMatcher.knnMatch(curentFrameDescriptor, matches, 2);

    // Get matches for image with best matching
    std::vector<std::vector<cv::DMatch>> matchesPerKF(mMap.vpKeyFrames.size());
    for (const auto &x :  matches) {
        if (x.size() == 2 && x[0].distance / x[1].distance < 0.9) {
            matchesPerKF[x[0].imgIdx].push_back(x[0]);
        }
    }
    int max = 0;
    nBestRelocKF = -1;
    for (int i = 0; i < matchesPerKF.size(); i++) {
        if (matchesPerKF[i].size() > max) {
            max = matchesPerKF[i].size();
            nBestRelocKF = i;
        }
    }
    if (nBestRelocKF < 0)
        return;
    auto bestMatches = matchesPerKF[nBestRelocKF];
    // Prepare structures for position estimation
    std::vector<cv::Point3f> worldPos;
    std::vector<cv::Point2f> imgPos;
    for (const auto &x : bestMatches) {
        if (mMap.vpKeyFrames[nBestRelocKF]->relocWorldPoints.find(x.trainIdx) != mMap.vpKeyFrames[nBestRelocKF]->relocWorldPoints.end()) {
            imgPos.push_back(keypoints[x.queryIdx].pt);
            worldPos.push_back(mMap.vpKeyFrames[nBestRelocKF]->relocWorldPoints[x.trainIdx]);
        }
    }
    if (imgPos.size() < 5) {
        return;
    }

    // Calculate rotation and translation
    cv::Vec3d R, t;

    Vector<3> kfT = mMap.vpKeyFrames[nBestRelocKF]->se3CfromW.get_translation();
    t(0) = kfT[0];
    t(1) = kfT[1];
    t(2) = kfT[2];
    Vector<3> kfR = mMap.vpKeyFrames[nBestRelocKF]->se3CfromW.get_rotation().ln();
    R(0) = kfR[0];
    R(1) = kfR[1];
    R(2) = kfR[2];


    cv::solvePnPRansac(worldPos, imgPos, mCamera.GetCameraMatrix(), cv::noArray(), R, t, true, 1000, insertKeypointRadius, 0.99);


    // Prepare camera pose
    Vector<3> finalT;
    finalT[0] = t(0);
    finalT[1] = t(1);
    finalT[2] = t(2);
    Vector<3> finalR;
    finalR[0] = R(0);
    finalR[1] = R(1);
    finalR[2] = R(2);
    lock.lock();
    lastRelocPose = SE3<>(SO3<>(finalR), finalT);
    newRelocPose = true;
    lock.unlock();
}

void MapMaker::SetMode(Mode m) {
    if (currentMode == m)
        return;
    currentMode = m;
    if (m == MM_MODE_RELOC)
        mbBundleAbortRequested = true;
}
