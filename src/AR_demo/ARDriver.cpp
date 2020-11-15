//
// Created by luka on 16. 09. 20.
//
#include "ARDriver.h"
#include <gvars3/instances.h>
#include <opencv2/calib3d.hpp>

ARDriver::ARDriver(const std::string &deviceFolder, const CVD::ImageRef videoSize) {
    GVars3::GUI.LoadFile(deviceFolder + "/PTAM_calib.cfg");
    VideoSource mVideoSource;

    mpCamera = new ATANCamera("Camera");
    mpCamera->SetImageSize(videoSize);
    mpMap = new Map;
    mpMapMaker = new MapMaker(*mpMap, *mpCamera, stats, deviceFolder, MapMaker::MM_MODE_FULL_AUTO);
    mpTracker = new Tracker(videoSize, *mpCamera, *mpMap, *mpMapMaker, mVideoSource , stats, "");

    cvToGlMat = cv::Mat::zeros(4, 4, CV_64F);
    cvToGlMat.at<double>(0, 0) = 1.0f;
    cvToGlMat.at<double>(1, 1) = -1.0f; // Invert the y axis
    cvToGlMat.at<double>(2, 2) = -1.0f; // invert the z axis
    cvToGlMat.at<double>(3, 3) = 1.0f;
}

Tracker::TrackingState ARDriver::TrackFrame(cv::Mat &imBW) {
    auto start = std::chrono::high_resolution_clock::now();
    mpTracker->TrackFrame(imBW);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    stats.AddTrackTime(duration.count());

    return mpTracker->GetTrackingQuality();
}

void ARDriver::GetCameraProjectionMat(double zNear, double zFar, cv::Mat &projectionMatrix, double scale) {
    auto w = mpCamera->GetImageSize()[0] * scale;
    auto h = mpCamera->GetImageSize()[1] * scale;
    auto fx = mpCamera->mvFocal[0] * scale;
    auto fy = mpCamera->mvFocal[1] * scale;
    auto cx = (mpCamera->mvCenter[0]+0.5) * scale - 0.5;
    auto cy = (mpCamera->mvCenter[1]+0.5) * scale - 0.5;

    projectionMatrix.at<double>(0,0) = 2*fx/w;
    projectionMatrix.at<double>(1,0) = 0;
    projectionMatrix.at<double>(2,0) = 0;
    projectionMatrix.at<double>(3,0) = 0;

    projectionMatrix.at<double>(0,1) = 0;
    projectionMatrix.at<double>(1,1) = 2*fy/h;
    projectionMatrix.at<double>(2,1) = 0;
    projectionMatrix.at<double>(3,1) = 0;

    projectionMatrix.at<double>(0,2) = 1-2*cx/w;
    projectionMatrix.at<double>(1,2) = -1+(2*cy+2)/h;
    projectionMatrix.at<double>(2,2) = (zNear+zFar)/(zNear - zFar);
    projectionMatrix.at<double>(3,2) = -1;

    projectionMatrix.at<double>(0,3) = 0;
    projectionMatrix.at<double>(1,3) = 0;
    projectionMatrix.at<double>(2,3) = 2*zNear*zFar/(zNear - zFar);
    projectionMatrix.at<double>(3,3) = 0;
}

void ARDriver::GetCameraPoseMat(cv::Mat &viewMatrix) {
    cv::Vec3d rvec, tvec;
    mpTracker->GetCameraPose(rvec, tvec);
    cv::Mat rotation(4, 4, CV_64F);
    cv::Rodrigues(rvec, rotation);

    for(unsigned int row=0; row<3; ++row) {
        for(unsigned int col=0; col<3; ++col) {
            viewMatrix.at<double>(row, col) = rotation.at<double>(row, col);
        }
        viewMatrix.at<double>(row, 3) = tvec[row];
    }
    viewMatrix.at<double>(3, 3) = 1.0f;
}

void ARDriver::GetCameraPosGLMat(cv::Mat &viewMatrix) {
    cv::Mat tmp = cv::Mat::zeros(4, 4, CV_64F);
    GetCameraPoseMat(tmp);
    tmp = cvToGlMat * tmp;
    cv::transpose(tmp , viewMatrix);
}