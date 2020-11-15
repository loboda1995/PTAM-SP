//
// Created by luka on 19. 04. 20.
//

#ifndef REALTIME_RECONSTRUCTION_PTAMEXPORTER_H
#define REALTIME_RECONSTRUCTION_PTAMEXPORTER_H

#include <string>
#include <vector>
#include <unordered_map>

#include <cereal/types/unordered_map.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/string.hpp>
#include <Eigen/Dense>

class PTAMImagePoint {
public:
    int level;
    int frame;
    double x;
    double y;

    PTAMImagePoint() {}
    PTAMImagePoint(int level, int frame, double x, double y) {
        this->level = level;
        this->frame = frame;
        this->x = x;
        this->y = y;
    }

    template<class Archive>
    void serialize(Archive &archive) {
        archive(level, frame, x, y);
    }
};

class PTAMPoint {
public:
    std::vector<PTAMImagePoint> imagePoints;
    float x;
    float y;
    float z;

    PTAMPoint() {}
    PTAMPoint(float a, float b, float c) {
        x = a;
        y = b;
        z = c;
    }

    void addPoint(int level, int frame, double x, double y) {
        imagePoints.emplace_back(PTAMImagePoint(level, frame, x, y));
    }
    const int numberOfImagePoints() { return imagePoints.size(); };

    Eigen::Vector4f getEigen4f() {
        return Eigen::Vector4f(x, y, z, 1);
    }

    template<class Archive>
    void serialize(Archive &archive) {
        archive(x);
        archive(y);
        archive(z);
        archive(imagePoints);
    }

};


class PTAMInstallerFile {
public:
    std::unordered_map<int, std::string> frameNames;
    std::vector<PTAMPoint> points;
    std::vector<PTAMPoint> calModelPoints;
    std::vector<PTAMPoint> calWorldPoints;
    int scalePoint1;
    int scalePoint2;
    float scaleDistance;


    template<class Archive>
    void serialize(Archive & archive)
    {
        archive(frameNames);
        archive(points);
        archive(calModelPoints);
        archive(calWorldPoints);
        archive(scalePoint1);
        archive(scalePoint2);
        archive(scaleDistance);
    }

    void addFrame(uint32_t vid, const std::string& frm) {
        frameNames[vid] = frm;
    }

    void addPoint(PTAMPoint &p) {
        points.push_back(p);
    }

    void addCalPoint(PTAMPoint &m, PTAMPoint &w) {
        calModelPoints.push_back(m);
        calWorldPoints.push_back(w);
    }
};

#endif //REALTIME_RECONSTRUCTION_PTAMEXPORTER_H
