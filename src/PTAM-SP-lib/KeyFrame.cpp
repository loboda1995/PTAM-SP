// Copyright 2008 Isis Innovation Limited
#include "KeyFrame.h"
#include "ShiTomasi.h"
#include "SmallBlurryImage.h"
#include <cvd/vision.h>
#include <cvd/fast_corner.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include "MapPoint.h"
#include "LevelHelpers.h"

using namespace CVD;
using namespace GVars3;

void KeyFrame::MakeKeyFrame_Lite(BasicImage<byte> &im) {
    // Perpares a Keyframe from an image. Generates pyramid levels, does FAST detection, etc.
    // Does not fully populate the keyframe struct, but only does the bits needed for the tracker;
    // e.g. does not perform FAST nonmax suppression. Things like that which are needed by the
    // mapmaker but not the tracker go in MakeKeyFrame_Rest();

    // First, copy out the image data to the pyramid's zero level.
    aLevels[0].im.resize(im.size());
    copy(im, aLevels[0].im);

    // Then, for each level...
    for (int i = 0; i < LEVELS; i++) {
        Level &lev = aLevels[i];
        if (i != 0) {  // .. make a half-size image from the previous level..
            lev.im.resize(aLevels[i - 1].im.size() / 2);
            halfSample(aLevels[i - 1].im, lev.im);
        }

        // .. and detect and store FAST corner points.
        // I use a different threshold on each level; this is a bit of a hack
        // whose aim is to balance the different levels' relative feature densities.
        lev.vCorners.clear();
        lev.vCandidates.clear();
        lev.vMaxCorners.clear();

        if (i == 0)
            fast_corner_detect_10(lev.im, lev.vCorners, 10);
        if (i == 1)
            fast_corner_detect_10(lev.im, lev.vCorners, 15);
        if (i == 2)
            fast_corner_detect_10(lev.im, lev.vCorners, 15);
        if (i == 3)
            fast_corner_detect_10(lev.im, lev.vCorners, 10);

        // Generate row look-up-table for the FAST corner points: this speeds up
        // finding close-by corner points later on.
        unsigned int v = 0;
        lev.vCornerRowLUT.clear();
        for (int y = 0; y < lev.im.size().y; y++) {
            while (v < lev.vCorners.size() && y > lev.vCorners[v].y)
                v++;
            lev.vCornerRowLUT.push_back(v);
        }
    }
    state = LITE;
}

void KeyFrame::MakeKeyFrame_Rest() {
    // Fills the rest of the keyframe structure needed by the mapmaker:
    // FAST nonmax suppression, generation of the list of candidates for further map points,
    // creation of the relocaliser's SmallBlurryImage.
    static gvar3<double> gvdCandidateMinSTScore("MapMaker.CandidateMinShiTomasiScore", 80, SILENT);

    // For each level...
    for (int l = 0; l < LEVELS; l++) {
        Level &lev = aLevels[l];
        // .. find those FAST corners which are maximal..
        if (l == 0 || l == 3) {
            fast_nonmax(lev.im, lev.vCorners, 10, lev.vMaxCorners);
        } else {
            fast_nonmax(lev.im, lev.vCorners, 15, lev.vMaxCorners);
        }
        // .. and then calculate the Shi-Tomasi scores of those, and keep the ones with
        // a suitably high score as Candidates, i.e. points which the mapmaker will attempt
        // to make new map points out of.
        for (std::vector<ImageRef>::iterator i = lev.vMaxCorners.begin(); i != lev.vMaxCorners.end(); i++) {
            if (!lev.im.in_image_with_border(*i, 10))
                continue;
            double dSTScore = FindShiTomasiScoreAtPoint(lev.im, 3, *i);
            if (dSTScore > *gvdCandidateMinSTScore) {
                Candidate c;
                c.irLevelPos = *i;
                c.dSTScore = dSTScore;
                lev.vCandidates.push_back(c);
            }
        }
        random_shuffle(lev.vCandidates.begin(), lev.vCandidates.end());
    };
    state = REST;
}

void KeyFrame::MakeKeyFrame_Reloc(int featureCount, double maxPointRadius) {
    Image<CVD::byte> im = aLevels[0].im;
    cv::Mat img(im.size().y, im.size().x, CV_8UC1, im.data());
    relocKeypoints.clear();
    relocWorldPoints.clear();

    cv::Ptr<cv::DescriptorExtractor> extractor = cv::ORB::create(featureCount, 1.25f, 8, 31, 0, 2);
    extractor->detectAndCompute(img, cv::noArray(), relocKeypoints, relocFrameDescriptor);

    for (int i = 0; i < relocKeypoints.size(); i++) {
        auto kp = relocKeypoints[i];
        double bestRadius = maxPointRadius;
        for (auto m : mMeasurements) {
            auto imp = LevelZeroPos(m.second.v2RootPos, m.second.nLevel);
            cv::Point2f mPos(imp[0], imp[1]);
            if (cv::norm(kp.pt - mPos) < bestRadius) {
                auto wp = m.first->v3WorldPos;
                relocWorldPoints[i] = cv::Point3d(wp[0], wp[1], wp[2]);
                bestRadius = cv::norm(kp.pt - mPos);
            }
        }
    }
}

// The keyframe struct is quite happy with default operator=, but Level needs its own
// to override CVD's reference-counting behaviour.
Level &Level::operator=(const Level &rhs) {
    // Operator= should physically copy pixels, not use CVD's reference-counting image copy.
    im.resize(rhs.im.size());
    copy(rhs.im, im);

    vCorners = rhs.vCorners;
    vMaxCorners = rhs.vMaxCorners;
    vCornerRowLUT = rhs.vCornerRowLUT;
    return *this;
}

// -------------------------------------------------------------
// Some useful globals defined in LevelHelpers.h live here:
Vector<3> gavLevelColors[LEVELS];

// These globals are filled in here. A single static instance of this struct is run before main()
struct LevelHelpersFiller // Code which should be initialised on init goes here; this runs before main()
{
    LevelHelpersFiller() {
        for (int i = 0; i < LEVELS; i++) {
            if (i == 0) gavLevelColors[i] = makeVector(1.0, 0.0, 0.0);
            else if (i == 1) gavLevelColors[i] = makeVector(1.0, 1.0, 0.0);
            else if (i == 2) gavLevelColors[i] = makeVector(0.0, 1.0, 0.0);
            else if (i == 3) gavLevelColors[i] = makeVector(0.0, 0.0, 0.7);
            else gavLevelColors[i] = makeVector(1.0, 1.0, 0.7); // In case I ever run with LEVELS > 4
        }
    }
};

static LevelHelpersFiller foo;







