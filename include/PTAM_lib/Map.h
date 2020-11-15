// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
//
// This header declares the Map class.
// This is pretty light-weight: All it contains is
// a vector of MapPoints and a vector of KeyFrames.
//
// N.b. since I don't do proper thread safety,
// everything is stored as lists of pointers,
// and map points are not erased if they are bad:
// they are moved to the trash list. That way
// old pointers which other threads are using are not 
// invalidated!

#ifndef __MAP_H
#define __MAP_H

#include <vector>
#include <TooN/se3.h>
#include <cvd/image.h>
#include "nanoflann.hpp"
#include "MapPoint.h"
#include "KeyFrame.h"

struct MapPointCloud
{
    std::vector<MapPoint*>  pts;

    inline size_t kdtree_get_point_count() const { return pts.size(); }

    inline double kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        return pts[idx]->v3WorldPos[dim];
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }

};

typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<nanoflann::L2_Simple_Adaptor<double, MapPointCloud>, MapPointCloud, 3> MapPointKD;

struct Map {
    Map();
    ~Map() {
        delete pointsKD;
    }

    inline bool IsGood() { return bGood; }

    void Reset();

    void MoveBadPointsToTrash();

    void EmptyTrash();

    bool AddPoint(MapPoint *p, double minRadius);

    std::vector<MapPoint *> vpPoints;
    std::vector<MapPoint *> vpPointsTrash;
    std::vector<KeyFrame *> vpKeyFrames;
    MapPointCloud pointsCloud;
    MapPointKD *pointsKD;

    bool SaveModelToFile(const std::string &loadFolder, const std::string &name, double cm);
    bool LoadModelFromFile(ATANCamera &cam, const std::string &loadFolder, std::string &name, double &cm);

    bool bGood;

protected:
    std::string saveKeyFrame(const std::string &folder, int id, const KeyFrame &kf);
};


#endif

