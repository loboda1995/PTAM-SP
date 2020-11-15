// Copyright 2008 Isis Innovation Limited
#include "OpenGL.h"
#include <gvars3/instances.h>
#include "VideoCameraCalibrator.h"
#include <TooN/SVD.h>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <cvd/colourspace_convert.h>

using namespace CVD;
using namespace GVars3;

int main() {
    std::cout << "  Welcome to CameraCalibrator " << std::endl;
    std::cout << "  -------------------------------------- " << std::endl;
    std::cout << "  Parallel tracking and mapping for Small AR workspaces" << std::endl;
    std::cout << "  Copyright (C) Isis Innovation Limited 2008 " << std::endl;
    std::cout << std::endl;
    std::cout << "  Parsing calibrator_settings.cfg ...." << std::endl;

    GUI.LoadFile("calibrator_settings.cfg");

    GUI.StartParserThread();
    atexit(GUI.StopParserThread); // Clean up readline when program quits

    GV3::get<Vector<NUMTRACKERCAMPARAMETERS> >("Camera.Parameters", ATANCamera::mvDefaultParams, SILENT);

    try {
        CameraCalibrator c;
        c.Run();
    }
    catch (CVD::Exceptions::All e) {
        std::cout << std::endl;
        std::cout << "!! Failed to run CameraCalibrator; got exception. " << std::endl;
        std::cout << "   Exception was: " << std::endl;
        std::cout << e.what << std::endl;
    }
}


CameraCalibrator::CameraCalibrator()
        : mGLWindow(mVideoSource.Size(), "Camera Calibrator"), mCamera("Camera") {
    mbDone = false;
    GUI.RegisterCommand("CameraCalibrator.GrabNextFrame", GUICommandCallBack, this);
    GUI.RegisterCommand("CameraCalibrator.Reset", GUICommandCallBack, this);
    GUI.RegisterCommand("CameraCalibrator.ShowNext", GUICommandCallBack, this);
    GUI.RegisterCommand("CameraCalibrator.SaveCalib", GUICommandCallBack, this);
    GUI.RegisterCommand("quit", GUICommandCallBack, this);
    GUI.RegisterCommand("exit", GUICommandCallBack, this);
    GV3::Register(mgvnOptimizing, "CameraCalibrator.Optimize", 0, SILENT);
    GV3::Register(mgvnShowImage, "CameraCalibrator.Show", 0, SILENT);
    GV3::Register(mgvnDisableDistortion, "CameraCalibrator.NoDistortion", 0, SILENT);
    GUI.ParseLine("GLWindow.AddMenu CalibMenu");
    GUI.ParseLine("CalibMenu.AddMenuButton Live GrabFrame CameraCalibrator.GrabNextFrame");
    GUI.ParseLine("CalibMenu.AddMenuButton Live Reset CameraCalibrator.Reset");
    GUI.ParseLine("CalibMenu.AddMenuButton Live Optimize \"CameraCalibrator.Optimize=1\"");
    GUI.ParseLine("CalibMenu.AddMenuToggle Live NoDist CameraCalibrator.NoDistortion");
    GUI.ParseLine("CalibMenu.AddMenuSlider Opti \"Show Img\" CameraCalibrator.Show 0 10");
    GUI.ParseLine("CalibMenu.AddMenuButton Opti \"Show Next\" CameraCalibrator.ShowNext");
    GUI.ParseLine("CalibMenu.AddMenuButton Opti \"Grab More\" CameraCalibrator.Optimize=0 ");
    GUI.ParseLine("CalibMenu.AddMenuButton Opti Reset CameraCalibrator.Reset");
    GUI.ParseLine("CalibMenu.AddMenuToggle Opti NoDist CameraCalibrator.NoDistortion");
    GUI.ParseLine("CalibMenu.AddMenuButton Opti Save CameraCalibrator.SaveCalib");
    Reset();
}

void CameraCalibrator::Run() {
    mVideoSource.Open("./calib_video.mp4");
    while (!mbDone) {
        // We use two versions of each video frame:
        // One black and white (for processing by the tracker etc)
        // and one RGB, for drawing.

        Image<Rgb<byte> > imFrameRGB(mVideoSource.Size());
        Image<byte> imFrameBW(mVideoSource.Size());

        // Grab new video frame...
        mVideoSource.GetAndFillFrameBWandRGB(imFrameBW, imFrameRGB);

        // Set up openGL
        mGLWindow.SetupViewport();
        mGLWindow.SetupVideoOrtho();
        mGLWindow.SetupVideoRasterPosAndZoom();

        if (mvCalibImgs.size() < 1)
            *mgvnOptimizing = 0;

        if (!*mgvnOptimizing) {
            if (!calibImageSet) {
                CalibImage c;
                if (c.MakeFromImage(imFrameBW)) {
                    c.GuessInitialPose(mCamera);
                    calibImageSet = true;
                } else {
                    calibImageSet = false;
                }
                currentCalibImage = c;
            }
            if (mbGrabNextFrame) {
                mvCalibImgs.push_back(currentCalibImage);
                mbGrabNextFrame = false;
            };
            GUI.ParseLine("CalibMenu.ShowMenu Live");
            glDrawPixels(imFrameBW);
            if (calibImageSet)
                currentCalibImage.Draw3DGrid(mCamera, false);
        } else {
            OptimizeOneStep();

            GUI.ParseLine("CalibMenu.ShowMenu Opti");
            int nToShow = *mgvnShowImage - 1;
            if (nToShow < 0)
                nToShow = 0;
            if (nToShow >= (int) mvCalibImgs.size())
                nToShow = mvCalibImgs.size() - 1;
            *mgvnShowImage = nToShow + 1;

            glDrawPixels(mvCalibImgs[nToShow].mim);
            mvCalibImgs[nToShow].Draw3DGrid(mCamera, true);
        }

        std::ostringstream ost;
        ost << "Camera Calibration: Grabbed " << mvCalibImgs.size() << " images." << std::endl;
        if (!*mgvnOptimizing) {
            ost << "Take snapshots of the calib grid with the \"GrabFrame\" button," << std::endl;
            ost << "and then press \"Optimize\"." << std::endl;
            ost << "Take enough shots (4+) at different angles to get points " << std::endl;
            ost << "into all parts of the image (corners too.) The whole grid " << std::endl;
            ost << "doesn't need to be visible so feel free to zoom in." << std::endl;
        } else {
            ost << "Current RMS pixel error is " << mdMeanPixelError << std::endl;
            ost << "Current camera params are  " << GV3::get_var("Camera.Parameters") << std::endl;
            ost << "(That would be a pixel aspect ratio of "
                << mCamera.PixelAspectRatio() << ")" << std::endl;
            ost << "Check fit by looking through the grabbed images." << std::endl;
            ost << "RMS should go below 0.5, typically below 0.3 for a wide lens." << std::endl;
            ost << "Press \"save\" to save calibration to camera.cfg file and exit." << std::endl;
        }

        mGLWindow.DrawCaption(ost.str());
        mGLWindow.DrawMenus();
        mGLWindow.HandlePendingEvents();
        mGLWindow.swap_buffers();
    }
}

void CameraCalibrator::Reset() {
    *mCamera.mgvvCameraParams = ATANCamera::mvDefaultParams;
    if (*mgvnDisableDistortion) mCamera.DisableRadialDistortion();

    mCamera.SetImageSize(mVideoSource.Size());
    mbGrabNextFrame = false;
    *mgvnOptimizing = false;
    mvCalibImgs.clear();
}

void CameraCalibrator::GUICommandCallBack(void *ptr, std::string sCommand, std::string sParams) {
    ((CameraCalibrator *) ptr)->GUICommandHandler(sCommand, sParams);
}

void CameraCalibrator::GUICommandHandler(std::string sCommand, std::string sParams)  // Called by the callback func..
{
    if (sCommand == "CameraCalibrator.Reset") {
        Reset();
        return;
    }
    if (sCommand == "CameraCalibrator.GrabNextFrame") {
        mbGrabNextFrame = true;
        return;
    }
    if (sCommand == "CameraCalibrator.ShowNext") {
        int nToShow = (*mgvnShowImage - 1 + 1) % mvCalibImgs.size();
        *mgvnShowImage = nToShow + 1;
        return;
    }
    if (sCommand == "CameraCalibrator.SaveCalib") {
        std::cout << "  Camera calib is " << GV3::get_var("Camera.Parameters") << std::endl;
        std::cout << "  Saving camera calib to camera.cfg..." << std::endl;
        std::ofstream ofs("calib_video_phone_j3.cfg");
        if (ofs.good()) {
            GV2.PrintVar("Camera.Parameters", ofs);
            ofs.close();
            std::cout << "  .. saved." << std::endl;
        } else {
            std::cout << "! Could not open camera.cfg for writing." << std::endl;
            GV2.PrintVar("Camera.Parameters", std::cout);
            std::cout << "  Copy-paste above line to camera.cfg or camera.cfg! " << std::endl;
        }
        mbDone = true;
    }
    if (sCommand == "exit" || sCommand == "quit") {
        mbDone = true;
    }
}

void CameraCalibrator::OptimizeOneStep() {
    int nViews = mvCalibImgs.size();
    int nDim = 6 * nViews + NUMTRACKERCAMPARAMETERS;
    int nCamParamBase = nDim - NUMTRACKERCAMPARAMETERS;

    Matrix<> mJTJ(nDim, nDim);
    Vector<> vJTe(nDim);
    mJTJ = Identity; // Weak stabilizing prior
    vJTe = Zeros;

    if (*mgvnDisableDistortion) mCamera.DisableRadialDistortion();


    double dSumSquaredError = 0.0;
    int nTotalMeas = 0;

    for (int n = 0; n < nViews; n++) {
        int nMotionBase = n * 6;
        std::vector<CalibImage::ErrorAndJacobians> vEAJ = mvCalibImgs[n].Project(mCamera);
        for (unsigned int i = 0; i < vEAJ.size(); i++) {
            CalibImage::ErrorAndJacobians &EAJ = vEAJ[i];
            // All the below should be +=, but the MSVC compiler doesn't seem to understand that. :(
            mJTJ.slice(nMotionBase, nMotionBase, 6, 6) =
                    mJTJ.slice(nMotionBase, nMotionBase, 6, 6) + EAJ.m26PoseJac.T() * EAJ.m26PoseJac;
            mJTJ.slice(nCamParamBase, nCamParamBase, NUMTRACKERCAMPARAMETERS, NUMTRACKERCAMPARAMETERS) =
                    mJTJ.slice(nCamParamBase, nCamParamBase, NUMTRACKERCAMPARAMETERS, NUMTRACKERCAMPARAMETERS) +
                    EAJ.m2NCameraJac.T() * EAJ.m2NCameraJac;
            mJTJ.slice(nMotionBase, nCamParamBase, 6, NUMTRACKERCAMPARAMETERS) =
                    mJTJ.slice(nMotionBase, nCamParamBase, 6, NUMTRACKERCAMPARAMETERS) +
                    EAJ.m26PoseJac.T() * EAJ.m2NCameraJac;
            mJTJ.T().slice(nMotionBase, nCamParamBase, 6, NUMTRACKERCAMPARAMETERS) =
                    mJTJ.T().slice(nMotionBase, nCamParamBase, 6, NUMTRACKERCAMPARAMETERS) +
                    EAJ.m26PoseJac.T() * EAJ.m2NCameraJac;
            // Above does twice the work it needs to, but who cares..

            vJTe.slice(nMotionBase, 6) =
                    vJTe.slice(nMotionBase, 6) + EAJ.m26PoseJac.T() * EAJ.v2Error;
            vJTe.slice(nCamParamBase, NUMTRACKERCAMPARAMETERS) =
                    vJTe.slice(nCamParamBase, NUMTRACKERCAMPARAMETERS) + EAJ.m2NCameraJac.T() * EAJ.v2Error;

            dSumSquaredError += EAJ.v2Error * EAJ.v2Error;
            ++nTotalMeas;
        }
    };

    mdMeanPixelError = sqrt(dSumSquaredError / nTotalMeas);

    TooN::SVD<> svd(mJTJ);
    Vector<> vUpdate(nDim);
    vUpdate = svd.backsub(vJTe);
    vUpdate *= 0.1; // Slow down because highly nonlinear...
    for (int n = 0; n < nViews; n++)
        mvCalibImgs[n].mse3CamFromWorld = SE3<>::exp(vUpdate.slice(n * 6, 6)) * mvCalibImgs[n].mse3CamFromWorld;
    mCamera.UpdateParams(vUpdate.slice(nCamParamBase, NUMTRACKERCAMPARAMETERS));
};















