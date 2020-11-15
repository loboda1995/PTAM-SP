//
// Created by luka on 24. 07. 20.
//

#ifndef PTAM_PTAMMODELFILE_H
#define PTAM_PTAMMODELFILE_H

#include <string>
#include <vector>
#include <unordered_map>
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/string.hpp>
#include <Eigen/Dense>
#include <TooN/TooN.h>

class ModelCVMat {
public:
    cv::Mat mat;
    ModelCVMat() {}
    ModelCVMat(cv::Mat &m) {
        mat = m;
    }

    template<class Archive>
    void save(Archive & archive) const
    {
        archive(mat.cols, mat.rows);
        for(int row = 0; row < mat.rows; ++row) {
            for(int col = 0; col < mat.cols; ++col) {
                auto el = mat.at<unsigned char>(row, col);
                archive(el);
            }
        }
    }

    template<class Archive>
    void load(Archive & archive)
    {
        int cols = 0;
        int rows = 0;
        archive(cols, rows);
        mat = cv::Mat(rows, cols, CV_8UC1);
        for(int row = 0; row < rows; ++row) {
            for(int col = 0; col < cols; ++col) {
                unsigned char el;
                archive(el);
                mat.at<unsigned char>(row, col) = el;
            }
        }
    }
};

class ModelCVPoint {
public:
    cv::Point3d point;
    ModelCVPoint() {}
    ModelCVPoint(cv::Point3d &p) {
        point = p;
    }

    template<class Archive>
    void serialize(Archive &archive) {
        archive(point.x, point.y, point.z);
    }
};

class ModelCVKeypoint {
public:
    cv::KeyPoint keyPoint;

    ModelCVKeypoint() {}
    ModelCVKeypoint(cv::KeyPoint &kp) {
        keyPoint = kp;
    }

    template<class Archive>
    void serialize(Archive &archive) {
        archive(keyPoint.pt.x, keyPoint.pt.y, keyPoint.size, keyPoint.angle, keyPoint.class_id, keyPoint.octave, keyPoint.response);
    }
};

class ModelMeasurement {
public:
    int kfID;
    int pointID;
    double x;
    double y;
    int source;
    int level;
    bool sub;

    ModelMeasurement() {}
    ModelMeasurement(int kf, int p, double x, double y, int src, int l, bool s) {
        this->kfID = kf;
        this->pointID = p;
        this->x = x;
        this->y = y;
        this->source = src;
        this->level = l;
        this->sub = s;
    }

    template<class Archive>
    void serialize(Archive &archive) {
        archive(kfID, pointID, x, y, source, level, sub);
    }
};

class ModelPoint {
public:
    int sourceKF;
    int centerX;
    int centerY;
    float x;
    float y;
    float z;
    int sourceLevel;
    bool fromModel;

    ModelPoint() {}
    ModelPoint(int kf, int cx, int cy, float a, float b, float c, int lvl, bool model) {
        sourceKF = kf;
        centerX = cx;
        centerY = cy;
        x = a;
        y = b;
        z = c;
        sourceLevel = lvl;
        fromModel = model;
    }


    template<class Archive>
    void serialize(Archive &archive) {
        archive(sourceKF, centerX, centerY, x, y, z, sourceLevel, fromModel);
    }
};

class ModelKeyFrame {
public:
    TooN::Vector<3, double> T;
    TooN::Vector<3, double> R;
    std::string image;

    std::vector<ModelCVKeypoint> keypoints;
    std::map<int, ModelCVPoint> worldPoints;
    ModelCVMat descriptor;


    ModelKeyFrame() {}
    ModelKeyFrame(const std::string &img, TooN::Vector<3, double> t, TooN::Vector<3, double> r) {
        this->image = img;
        this->T = t;
        this->R = r;
    }


    template<class Archive>
    void serialize(Archive &archive) {
        archive(image, T[0], T[1], T[2], R[0], R[1], R[2]);
        archive(keypoints);
        archive(worldPoints);
        archive(descriptor);
    }
};


class PTAMModelFile {
public:
    PTAMModelFile() {}
    PTAMModelFile(const std::string name, double cm) {
        this->name = name;
        this->oneCM = cm;
    }
    std::string name;
    std::vector<ModelKeyFrame> keyframes;
    std::vector<ModelPoint> points;
    std::vector<ModelMeasurement> measurements;

    double oneCM;


    template<class Archive>
    void serialize(Archive & archive)
    {
        archive(name);
        archive(keyframes);
        archive(points);
        archive(measurements);
        archive(oneCM);
    }

};

#endif //PTAM_PTAMMODELFILE_H
