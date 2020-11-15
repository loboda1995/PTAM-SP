#ifndef __HEADLESS_SYSTEM_H
#define __HEADLESS_SYSTEM_H
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

class HeadlessSystem
{
public:
    HeadlessSystem(const std::string &rootFolder, const std::string &testFolder, const std::string &pathFile, MapMaker::OperationMode mode, const std::string &model);
    ~HeadlessSystem() {
        delete mpTracker;
        delete mpMapMaker;
        delete mpMap;
        delete mpCamera;
    }
    void Run();

private:
    VideoSource mVideoSource;

    Map *mpMap;
    MapMaker *mpMapMaker;
    Tracker *mpTracker;
    ATANCamera *mpCamera;

    TrackingStats stats;

    bool mbDone;
    int lastFrameN = -1;

    void PrintStats();
};



#endif
