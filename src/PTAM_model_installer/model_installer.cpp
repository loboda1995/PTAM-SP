//
// Created by luka on 5. 08. 20.
//

#include "MapMaker.h"
#include "MapPoint.h"
#include "Bundle.h"
#include "TrackerData.h"

#include <cvd/vector_image_ref.h>
#include <cvd/vision.h>
#include <cvd/image_interpolate.h>

#include <TooN/SVD.h>
#include <TooN/SymEigen.h>

#include <gvars3/instances.h>
#include <fstream>
#include <algorithm>
#include <dirent.h>
#include <opencv2/imgproc.hpp>

#include "PTAMInstallerFile.h"
#include <cereal/archives/binary.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <sys/stat.h>
#include "PTAMModelFile.h"
#include "DBoW2.h"
#include "SmallBlurryImage.h"

const Vector<2> defaultCameraSize;



void createRelocDB(const std::string &deviceFolder)
{
    std::vector<std::vector<cv::Mat >> features;
    std::vector<std::string> featureToModel;
    std::vector<std::string> featureToKeyFrame;

    std::cout << "Extracting ORB features" << std::endl;
    cv::Ptr<cv::ORB> orb = cv::ORB::create(500, 1.25f, 8, 31, 0, 2);

    DIR *dir = opendir(deviceFolder.c_str());
    struct dirent *entry = readdir(dir);
    while (entry != nullptr)
    {
        if (entry->d_type == DT_DIR) {
            std::string fname = entry->d_name;
            if (fname.find("model_") != std::string::npos) {
                std::string modelFolder = deviceFolder + "/" + fname;

                std::ifstream loaderFile(modelFolder + "/data_ptam", std::ios::binary);
                PTAMModelFile loader;
                cereal::BinaryInputArchive iarchive(loaderFile);
                iarchive(loader);

                std::vector<cv::String> images;
                cv::glob(modelFolder  + "/frame*.jpg", images, false);
                for (const auto & img : images) {
                    cv::Mat image = cv::imread(img, cv::IMREAD_GRAYSCALE);
                    std::vector<cv::KeyPoint> keypoints;
                    cv::Mat descriptors;
                    orb->detectAndCompute(image, cv::noArray(), keypoints, descriptors);
                    features.emplace_back(std::vector<cv::Mat >());
                    features.back().resize(descriptors.rows);
                    for(int i = 0; i < descriptors.rows; ++i)
                    {
                        features.back()[i] = descriptors.row(i);
                    }
                    featureToModel.push_back(fname);
                }
            }
        }
        entry = readdir(dir);
    }
    closedir(dir);
    // branching factor and depth levels
    const int k = 10;
    const int L = 4;
    const DBoW2::WeightingType weight = DBoW2::TF_IDF;
    const DBoW2::ScoringType scoring = DBoW2::L1_NORM;

    OrbVocabulary voc(k, L, weight, scoring);
    voc.create(features);

    std::cout << "Creating reloc DBoW2 database" << std::endl;
    OrbDatabase db(voc, false, 0);

    // add images to the database
    for(const auto & feature : features)
        db.add(feature);

    // we can save the database. The created file includes the vocabulary
    // and the entries added
    std::cout << "Saving reloc DBoW2 database" << std::endl;
    cv::FileStorage fs(deviceFolder + "/reloc_db_meta.yml", cv::FileStorage::WRITE);
    fs << "feature_to_model" << featureToModel;
    fs.release();

    db.save(deviceFolder + "/reloc_db.yml.gz");

    std::cout << "Testing DBoW2 database reload" << std::endl;
    OrbDatabase db2(deviceFolder + "/reloc_db.yml.gz");
}

void InstallModel(const std::string &installerFolder, const std::string &deviceFolder, const std::string &name) {
    // Get folder name for next model install
    DIR *dir = opendir(deviceFolder.c_str());
    struct dirent *entry = readdir(dir);
    int c = 0;
    while (entry != nullptr)
    {
        if (entry->d_type == DT_DIR) {
            std::string fname = entry->d_name;
            if (fname.find("model_") != std::string::npos) {
                c++;
            }
        }
        entry = readdir(dir);
    }
    closedir(dir);
    c++;
    std::stringstream destFolder;
    destFolder << deviceFolder << "/";
    if (c >= 100) {
        destFolder << "model_" << c;
    } else if (c >= 10) {
        destFolder << "model_0" << c;
    } else {
        destFolder << "model_00" << c;
    }

    // Get calibration for device
    std::string calibFile;
    calibFile.append(deviceFolder);
    calibFile.append("/PTAM_calib.cfg");
    GVars3::GUI.LoadFile(calibFile);
    Vector<NUMTRACKERCAMPARAMETERS> vTest = GVars3::GV3::get<Vector<NUMTRACKERCAMPARAMETERS> >("Camera.Parameters", ATANCamera::mvDefaultParams, GVars3::HIDDEN);
    auto mpCamera = new ATANCamera("Camera");
    if (vTest == ATANCamera::mvDefaultParams) {
        std::cout << "Camera.Parameters is not set, need to run the CameraCalibrator tool" << std::endl;
        std::cout << "and/or put the Camera.Parameters= line into the appropriate .cfg file." << std::endl;
        exit(1);
    }
    Vector<2> cameraSize = GVars3::GV3::get<Vector<2> >("Camera.Size", defaultCameraSize, GVars3::HIDDEN);
    if (cameraSize == defaultCameraSize) {
        std::cout << "Camera.Size is not set" << std::endl;
        exit(1);
    }
    SmallBlurryImage::mirSize[0] = -1;
    SmallBlurryImage::mirSize[1] = -1;
    // Load model from installer
    TrackingStats stats;
    auto mpMap = new Map;
    auto mMapMaker = new MapMaker(*mpMap, *mpCamera, stats, deviceFolder, MapMaker::MM_MODE_INSTALL);
    CVD::ImageRef imSize(cameraSize[0], cameraSize[1]);
    SE3<> tmp;
    std::cout << "Loading map from installer: " << installerFolder << std::endl;
    if (!mMapMaker->LoadMapFromInstaller(installerFolder, imSize, tmp) || !mpMap->bGood) {
        std::cout << "Failed to load map from installer" << std::endl;
        exit(1);
    }

    // Save model to file
    std::cout << "Saving model to loader" << std::endl;
    mkdir(destFolder.str().c_str(), 0777);
    if (!mpMap->SaveModelToFile(destFolder.str(), name, mMapMaker->GetOneCM())) {
        std::cout << "Failed to save map to loader file" << std::endl;
        exit(1);
    }

    // Test load
    std::cout << "Testing reload" << std::endl;
    mpMap->Reset();
    std::string modelName;
    double cm;
    if (!mpMap->LoadModelFromFile(*mpCamera, destFolder.str(), modelName, cm)) {
        std::cout << "Test load failed" << std::endl;
        exit(1);
    }

    std::cout << "Model '" << modelName << "' successfully installed to: " << destFolder.str() << std::endl << std::endl;

    delete mpMap;
    delete mMapMaker;
    delete mpCamera;
}

void ClearFolder(const std::string &deviceFolder) {
    DIR *dir = opendir(deviceFolder.c_str());
    struct dirent *entry = readdir(dir);
    while (entry != nullptr)
    {
        if (entry->d_type == DT_DIR) {
            std::string fname = entry->d_name;
            if (fname.find("model_") != std::string::npos) {
                std::string cmd = "rm -r " + deviceFolder + "/" + fname;
                if (system(cmd.c_str()) != 0) {
                    std::cout << "Failed to clear folder" << std::endl;
                    exit(1);
                }
            }
        }
        entry = readdir(dir);
    }
    closedir(dir);
}

int main(int argc, char **argv) {
    int i = 0;

    std::vector<std::string> devices = {"web_cam_Logitech", "Huawei_P10", "Samsung_J3", "DSC_S60", "DSC_HX5"};
    std::vector<std::string> models = {"doorstep", "desk_living_room", "desk_PC", "kitchen", "statues", "toys", "garage"};

    std::cout << "#########################################" << std::endl;
    std::cout << "  Installing device: " + devices[i] << std::endl;
    std::cout << "#########################################" << std::endl;

    std::string installFolder = "/home/luka/mag_data/installers/";
    std::string deviceFolder = "/home/luka/mag_data/devices/device_" + devices[i];

    ClearFolder(deviceFolder);
    for (const auto &m : models) {
        InstallModel(installFolder + m, deviceFolder, m);
    }

    // Create reloc DB
    std::cout << "Creating reloc DB" << std::endl;
    createRelocDB(deviceFolder);

    std::cout << "#########################################" << std::endl << std::endl;
}
