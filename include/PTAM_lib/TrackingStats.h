#ifndef PTAM_TRACKINGSTATS_H
#define PTAM_TRACKINGSTATS_H

class TrackingStats {
public:
    double GetAvgTrackingTime() {
        return sumTimeForTracking / nTracking;
    }
    int GetSuccessfulRelocs() {
        return nSuccessfulRelocs;
    }
    int GetLoadedModels() {
        return nLoadedModels;
    }
    int GetNumOfStartKeyFrames() {
        return nStartKFs;
    }
    int GetNumOfStartPoints() {
        return nStartPoints;
    }
    int GetNumOfEndKeyFrames() {
        return nEndKFs;
    }
    int GetNumOfEndPoints() {
        return nEndPoints;
    }

    void AddTrackTime(double t) {
        sumTimeForTracking += t;
        nTracking++;
    }
    void AddSuccessfulReloc() {
        nSuccessfulRelocs++;
    }
    void AddLoadedModel(int kfs, int pts) {
        nLoadedModels++;
        nStartKFs = kfs;
        nStartPoints = pts;
    }
    void SetEndStats(int kfs, int pts) {
        nEndKFs = kfs;
        nEndPoints = pts;
    }

private:
    double sumTimeForTracking = 0;
    int nTracking = 0;
    int nSuccessfulRelocs = 0;
    int nLoadedModels = 0;
    int nStartKFs = 0;
    int nEndKFs = 0;
    int nStartPoints = 0;
    int nEndPoints = 0;
};

#endif //PTAM_TRACKINGSTATS_H
