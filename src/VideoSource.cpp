#include "VideoSource.h"
#include <cvd/Linux/dvbuffer3.h>
#include <iostream>
#include <opencv2/imgproc.hpp>

using namespace CVD;
using namespace cv;

VideoSource::VideoSource() {
    fromFile = false;
    lastFrameN = -1;
}

void VideoSource::Open() {
    pcap.open(-1);

    if (!pcap.isOpened()) {
        std::cerr << "Cannot open default capture device. Exiting... " << std::endl;
        exit(-1);
    }

    int width = (int) pcap.get(cv::CAP_PROP_FRAME_WIDTH);
    int height = (int) pcap.get(cv::CAP_PROP_FRAME_HEIGHT);
    mirSize = CVD::ImageRef(width, height);
}

void VideoSource::Open(const std::string &filename) {
    pcap.open(filename);
    fromFile = true;
    lastFrameN = -1;

    if (!pcap.isOpened()) {
        std::cerr << "Cannot read '" << filename << "'. Exiting... " << std::endl;
        exit(-1);
    }
    totalFrames = (int) pcap.get(cv::CAP_PROP_FRAME_COUNT);
    auto fps = (double) pcap.get(cv::CAP_PROP_FPS);
    tFrame = 1000 / fps;

    int width = (int) pcap.get(cv::CAP_PROP_FRAME_WIDTH);
    int height = (int) pcap.get(cv::CAP_PROP_FRAME_HEIGHT);
    mirSize = CVD::ImageRef(width, height);
}

ImageRef VideoSource::Size() {
    return mirSize;
}

bool VideoSource::GetAndFillFrameBWandRGB(Image<byte> &imBW, Image<Rgb<byte> > &imRGB) {
    cv::Mat bw(mirSize.y, mirSize.x, CV_8UC1, imBW.data());
    cv::Mat rgb(mirSize.y, mirSize.x, CV_8UC3, imRGB.data());
    auto res = GetAndFillFrameBWandRGB(bw, rgb);
    cvtColor(rgb, rgb, cv::COLOR_BGR2RGB);
    return res;
}

bool VideoSource::GetAndFillFrameBWandRGB(cv::Mat &imBW, cv::Mat &imRGB) {
    if (fromFile && lastFrameN < 0)
        start = steady_clock::now();

    cv::Mat capFrame;
    if (getNextFrame() || lastFrame.empty()) {
        if (lastFrameN >= totalFrames) {
            return false;
        }
        if (!pcap.read(capFrame))
            return false;
        if (fromFile)
            lastFrame = capFrame;
    } else {
        capFrame = lastFrame;
    }
    if (capFrame.empty())
        return false;

    capFrame.copyTo(imRGB);
    cv::cvtColor(capFrame, imBW, cv::COLOR_RGB2GRAY);
    return true;
}

bool VideoSource::getNextFrame() {
    if (!fromFile)
        return true;

    double elapsed = duration_cast<milliseconds>(steady_clock::now() - start).count();
    int n = floor(elapsed / tFrame);
    if (n > lastFrameN) {
        while (pcap.get(cv::CAP_PROP_POS_FRAMES) < n - 1) {
            pcap.grab();
        }
        lastFrameN = n;
        return true;
    }
    return false;
}

void VideoSource::RestartVideoFile() {
    if (!fromFile)
        return;
    pcap.set(cv::CAP_PROP_POS_FRAMES, 0);
    lastFrameN = -1;
}
