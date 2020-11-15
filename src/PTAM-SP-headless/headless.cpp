#include <stdlib.h>
#include <iostream>
#include <gvars3/instances.h>
#include "HeadlessSystem.h"


using namespace GVars3;

int main(int argc, char** argv) {


    if (argc < 7) {
        std::cout << "Missing required parameters: configFile, mode, deviceFolder, testFolder, model, pathFile" << std::endl;
        exit(1);
    }

    std::string configFile = argv[1];
    std::string modeS = argv[2];
    std::string deviceFolder = argv[3];
    std::string testFolder = argv[4];
    std::string model = argv[5];
    std::string pathFile = argv[6];

    MapMaker::OperationMode mode = MapMaker::MM_MODE_FULL_AUTO;
    if (modeS == "FROM_INSTALLER") {
        mode = MapMaker::MM_MODE_FROM_INSTALLER;
    } else if (modeS == "SINGLE_MODEL") {
        mode = MapMaker::MM_MODE_SINGLE_MODEL;
    }

    std::cout << "Parsing '" << configFile << "' ..." << std::endl;
    GUI.LoadFile(configFile);

    try {
        HeadlessSystem s(deviceFolder, testFolder, pathFile, mode, model);
        s.Run();
    } catch (CVD::Exceptions::All e) {
        std::cout << std::endl;
        std::cout << "!! Failed to run system; got exception. " << std::endl;
        std::cout << "   Exception was: " << std::endl;
        std::cout << e.what << std::endl;
    }
    std::cout << "Finished" << std::endl;
    exit(0);
}










