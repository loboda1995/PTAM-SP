// -*- c++ *--
// Copyright 2008 Isis Innovation Limited
//
// VideoSource.h
// Declares the VideoSource class
// 
// This is a very simple class to provide video input; this can be
// replaced with whatever form of video input that is needed.  It
// should open the video input on construction, and provide two
// function calls after construction: Size() must return the video
// format as an ImageRef, and GetAndFillFrameBWandRGB should wait for
// a new frame and then overwrite the passed-as-reference images with
// GreyScale and Colour versions of the new frame.
//
#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/rgb.h>
#include <chrono>
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>

using namespace std::chrono;

struct VideoSourceData;

class VideoSource
{
    public:
        VideoSource();

        void Open();
        void Open(const std::string &filename);

        bool GetAndFillFrameBWandRGB(CVD::Image<CVD::byte> &imBW, CVD::Image<CVD::Rgb<CVD::byte> > &imRGB);
        bool GetAndFillFrameBWandRGB(cv::Mat &imBW, cv::Mat &imRGB);
        void RestartVideoFile();
        bool IsFromRecording() { return fromFile; };
        CVD::ImageRef Size();
        int totalFrames;
        int GetFrameN() { return lastFrameN; };

    private:
        CVD::ImageRef mirSize;
        bool fromFile = false;
        cv::VideoCapture pcap;

        double tFrame;
        time_point<steady_clock, duration<float>> start;
        int lastFrameN;
        cv::Mat lastFrame;
        bool getNextFrame();
};
