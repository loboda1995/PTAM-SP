// Copyright 2008 Isis Innovation Limited
#include "Map.h"
#include <gvars3/instances.h>
#include "PTAMModelFile.h"
#include <opencv2/opencv.hpp>
#include <fstream>
#include <cereal/archives/binary.hpp>
#include "MapMaker.h"
#include "LevelHelpers.h"

Map::Map() {
    pointsCloud.pts = vpPoints;
    pointsKD = new MapPointKD(3, pointsCloud, nanoflann::KDTreeSingleIndexAdaptorParams(50));
    Reset();
}

void Map::Reset() {
    for (unsigned int i = 0; i < vpPoints.size(); i++) {
        pointsKD->removePoint(i);
        delete vpPoints[i];
    }
    for (unsigned int i = 0; i < vpKeyFrames.size(); i++) {
        delete vpKeyFrames[i];
    }
    vpKeyFrames.clear();
    pointsCloud.pts.clear();
    vpPoints.clear();
    bGood = false;
    EmptyTrash();
}

void Map::MoveBadPointsToTrash() {
    for (int i = pointsCloud.pts.size() - 1; i >= 0; i--) {
        if (pointsCloud.pts[i]->bBad)
            pointsKD->removePoint(i);
    }
    for (int i = vpPoints.size() - 1; i >= 0; i--) {
        if (vpPoints[i]->bBad) {
            vpPointsTrash.push_back(vpPoints[i]);
            vpPoints.erase(vpPoints.begin() + i);
        }
    };
};

void Map::EmptyTrash() {
    for (unsigned int i = 0; i < vpPointsTrash.size(); i++)
        delete vpPointsTrash[i];
    vpPointsTrash.clear();
};

bool Map::AddPoint(MapPoint *p, double minRadius) {
    size_t ret_index[1];
    double out_dist_sqr[1];
    nanoflann::KNNResultSet<double> resultSet(1);
    resultSet.init(ret_index, out_dist_sqr );
    double query[3] = { p->v3WorldPos[0], p->v3WorldPos[1], p->v3WorldPos[2]};
    if (pointsKD->findNeighbors(resultSet, query, nanoflann::SearchParams(10))) {
        if (out_dist_sqr[0] < minRadius*minRadius) {
            return false;
        }
    }

    vpPoints.push_back(p);
    pointsCloud.pts.push_back(p);
    pointsKD->addPoints(pointsCloud.pts.size()-1, pointsCloud.pts.size()-1);
    return true;
}

std::string Map::saveKeyFrame(const std::string &folder, int id, const KeyFrame &kf) {
    std::stringstream imageName;
    if (id >= 100) {
        imageName << "frame" << id;
    } else if (id >= 10) {
        imageName << "frame0" << id;
    } else {
        imageName << "frame00" << id;
    }
    imageName << ".jpg";
    std::string name = folder + "/" + imageName.str();

    CVD::Image<CVD::byte> im = kf.aLevels[0].im;
    cv::Mat img(im.size().y, im.size().x, CV_8UC1, im.data());
    cv::imwrite(name, img);

    return imageName.str();
}

bool Map::SaveModelToFile(const std::string &loadFolder, const std::string &name, double cm) {
    std::map<KeyFrame*, int> getKFID;
    std::map<MapPoint*, int> getPointID;

    PTAMModelFile loader(name, cm);
    // Prepare KeyFrames
    for (int i = 0; i < vpKeyFrames.size(); i++) {
        auto kf = vpKeyFrames[i];
        getKFID[kf] = i;
        ModelKeyFrame m;
        m.image = saveKeyFrame(loadFolder, i, *kf);
        m.T = kf->se3CfromW.get_translation();
        m.R = kf->se3CfromW.get_rotation().ln();
        // Save relocalizer data
        for (auto &kp : kf->relocKeypoints) {
            m.keypoints.emplace_back(ModelCVKeypoint(kp));
        }
        m.descriptor = ModelCVMat(kf->relocFrameDescriptor);
        for (auto &p : kf->relocWorldPoints) {
            m.worldPoints[p.first] = ModelCVPoint(p.second);
        }
        loader.keyframes.push_back(m);
    }
    // Prepare MapPoints
    int c = 0;
    for (int i = 0; i < vpPoints.size(); i++) {
        auto p = vpPoints[i];
        if (p->bBad) {
            continue;
        }
        getPointID[p] = c;
        c++;
        ModelPoint m(getKFID[p->pPatchSourceKF], p->irCenter[0], p->irCenter[1], p->v3WorldPos[0], p->v3WorldPos[1], p->v3WorldPos[2], p->nSourceLevel, p->bFromModel);
        loader.points.push_back(m);
    }
    // Prepare Measurements
    for (const auto &kf : vpKeyFrames) {
        if (getKFID.find(kf) == getKFID.end())
            return false;
        int kfID = getKFID[kf];
        for (const auto &m : kf->mMeasurements) {
            if (m.first->bBad)
                continue;
            if (getPointID.find(m.first) == getPointID.end())
                return false;
            int pID = getPointID[m.first];
            int source = 0;
            if (m.second.Source == Measurement::SRC_TRACKER)
                source = 1;
            else if (m.second.Source == Measurement::SRC_REFIND)
                source = 2;
            else if (m.second.Source == Measurement::SRC_EPIPOLAR)
                source = 3;
            else if (m.second.Source == Measurement::SRC_ROOT)
                source = 4;

            ModelMeasurement meas(kfID, pID, m.second.v2RootPos[0], m.second.v2RootPos[1], source, m.second.nLevel, m.second.bSubPix);
            loader.measurements.push_back(meas);
        }
    }
    // Write to file
    std::ofstream loaderFile(loadFolder + "/data_ptam", std::ios::binary);
    {
        cereal::BinaryOutputArchive oarchive(loaderFile);
        oarchive(loader);
    }
    loaderFile.close();
    return true;
}

bool Map::LoadModelFromFile(ATANCamera &cam, const std::string &loadFolder, std::string &name, double &cm) {
    std::ifstream loaderFile(loadFolder + "/data_ptam", std::ios::binary);
    PTAMModelFile loader;
    cereal::BinaryInputArchive iarchive(loaderFile);
    iarchive(loader);

    std::map<int, KeyFrame*> getKFID;
    std::map<int, MapPoint*> getPointID;
    // Load KeyFrames
    for (int i = 0; i < loader.keyframes.size(); i++) {
        auto modelKF = loader.keyframes[i];
        auto kf = new KeyFrame();
        kf->se3CfromW = SE3<>(SO3<>(modelKF.R), modelKF.T);
        kf->bFixed = i == 0;
        kf->imagePath = loadFolder + "/" + modelKF.image;
        vpKeyFrames.push_back(kf);
        getKFID[i] = kf;
        for (auto &kp : modelKF.keypoints) {
            kf->relocKeypoints.push_back(kp.keyPoint);
        }
        kf->relocFrameDescriptor = modelKF.descriptor.mat;
        for (auto &p : modelKF.worldPoints) {
            kf->relocWorldPoints[p.first] = p.second.point;
        }
    }
    // Load MapPoints
    for (int i = 0; i < loader.points.size(); i++) {
        auto modelP = loader.points[i];
        auto p = new MapPoint();
        p->nSourceLevel = modelP.sourceLevel;
        p->bFromModel = modelP.fromModel;
        p->v3WorldPos[0] = modelP.x;
        p->v3WorldPos[1] = modelP.y;
        p->v3WorldPos[2] = modelP.z;

        auto center = CVD::ImageRef(modelP.centerX, modelP.centerY);
        p->SetSourcePatch(cam, getKFID[modelP.sourceKF], modelP.sourceLevel, center, LevelZeroPos(center, modelP.sourceLevel));
        vpPoints.push_back(p);
        getPointID[i] = p;
    }
    // Load Measurements
    for (const auto &modelM : loader.measurements) {
        Measurement m;
        m.nLevel = modelM.level;
        if (modelM.source == 0)
            m.Source = Measurement::SRC_TRAIL;
        else if(modelM.source == 1)
            m.Source = Measurement::SRC_TRACKER;
        else if(modelM.source == 2)
            m.Source = Measurement::SRC_REFIND;
        else if(modelM.source == 3)
            m.Source = Measurement::SRC_EPIPOLAR;
        else if(modelM.source == 4)
            m.Source = Measurement::SRC_ROOT;
        m.v2RootPos[0] = modelM.x;
        m.v2RootPos[1] = modelM.y;
        m.bSubPix = modelM.sub;

        auto kf = getKFID[modelM.kfID];
        auto p = getPointID[modelM.pointID];
        kf->mMeasurements[p] = m;
        p->pMMData->sMeasurementKFs.insert(kf);
    }

    name = loader.name;
    cm = loader.oneCM;
    loaderFile.close();
    return true;
}