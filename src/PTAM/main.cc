// Copyright 2008 Isis Innovation Limited
// This is the main extry point for PTAM
#include <stdlib.h>
#include <iostream>
#include <gvars3/instances.h>
#include "System.h"


using namespace GVars3;

int main() {
    std::string root = "/home/luka/mag_data/recordings";
    //std::string deviceFolder = "/home/luka/mag_data/devices/device_Huawei_P10";
    //std::string deviceFolder = "/home/luka/mag_data/devices/device_DSC_HX5";
    //std::string deviceFolder = "/home/luka/mag_data/devices/device_DSC_S60";
    std::string deviceFolder = "/home/luka/mag_data/devices/device_web_cam_Logitech";
    //std::string deviceFolder = "/home/luka/mag_data/devices/device_Samsung_J3";

    std::string testFolder = root + "/garage/test_06";

    std::cout << "  Parsing camera.cfg ...." << std::endl;
    GUI.LoadFile(deviceFolder + "/PTAM_calib.cfg");

    try {
        System s(deviceFolder, testFolder, MapMaker::MM_MODE_FULL_AUTO, "");
        s.Run();
    }
    catch (CVD::Exceptions::All e) {
        std::cout << std::endl;
        std::cout << "!! Failed to run system; got exception. " << std::endl;
        std::cout << "   Exception was: " << std::endl;
        std::cout << e.what << std::endl;
    }
}










