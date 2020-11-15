// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
//
// System.h
//
// Defines the System class
//
// This stores the main functional classes of the system, like the
// mapmaker, map, tracker etc, and spawns the working threads.
//
#ifndef __SYSTEM_H
#define __SYSTEM_H
#include "MapViewer.h"
#include "GLWindow2.h"
#include "Tracker.h"
#include "TrackingStats.h"

#include <cvd/image.h>
#include <cvd/rgb.h>
#include <cvd/byte.h>

class ATANCamera;
class Map;
class MapMaker;
class Tracker;
class MapViewer;

class System
{
public:
    System(const std::string &rootFolder, const std::string &testFolder, MapMaker::OperationMode mode, const std::string &model);
    void Run();

    ~System() {
        delete mpMapViewer;
        delete mpTracker;
        delete mpMapMaker;
        delete mpMap;
        delete mpCamera;
        //delete mGLWindow;
    }
  
private:
    VideoSource mVideoSource;
    GLWindow2 *mGLWindow;
    CVD::Image<CVD::Rgb<CVD::byte> > mimFrameRGB;
    CVD::Image<CVD::byte> mimFrameBW;

    Map *mpMap;
    MapMaker *mpMapMaker;
    Tracker *mpTracker;
    ATANCamera *mpCamera;
    MapViewer *mpMapViewer;
  
    bool mbDone;
    bool mbRestartRecording = false;

    TrackingStats stats;
    void PrintStats();
    static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
};



#endif
