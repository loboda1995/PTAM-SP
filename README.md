# PTAM-SP
PTAM-SP (PTAM Scene Prior) is a modified version of PTAM algorithm (*G. Klein and D. Murray, Parallel Tracking and Mapping for Small AR Workspaces*) that uses reconstructed 3D models of the scene with keyframes to 
initialize itself and determine scale of the scene. Project was developed as part of master's thesis.

PTAM-SP uses completely automatic initialization of map based on importing
preprocessed reconstructed 3D model of the scene. Using calibration points 
provided during reconstruction it is also able to translate internal map into
reference coordinate system and determine scale of the scene. PTAM-SP recognises 
the scene and loads appropriate model to initialize map with points and keyframes.
Relocalization was also redesigned and is now based on ORB descriptors.

### How To Use
PTAM-SP needs reconstructed scene to work. 3D reconstructed model is prepared 
by software provided here: https://github.com/loboda1995/PTAM-SP-reconstruction. Calibration points and their position in real world 
need to be specified before exporting the model.

Exported model is then preprocessed for specific device (camera) with  **ModelInstaller**
where we also need camera parameters that can be estimated with **VideoCameraCalibrator**.
Preprocessed models are saved in folder structure for each device and this device folder
can then be used to run **PTAM-SP** on video recordings.

### Building
Dependencies are the same as for PTAM:
* TooN
* libCVD
* Gvars3

Useful guide: http://hustcalm.me/blog/2013/09/27/ptam-compilation-on-linux-howto/

PTAM-SP also uses:
* OpenCV
* DBoW2 - https://github.com/dorian3d/DBoW2
* nanoflann - https://github.com/jlblancoc/nanoflann
* cereal - https://github.com/USCiLab/cereal


### Project Structure
There is several subprojects (executables):
* **PTAM-SP** - application based on original PTAM with simple GUI
* **PTAM-SP-calib** - applications for calibrating camera and extracting camera parameters
* **PTAM-SP-headless** - headles version of PTAM-SP, can be used for testing
* **PTAM-SP-lib** - core of the PTAM-SP, contains everythind needed for tracking and mapping
* **PTAM-SP-model-installer** - application used to install several models for specifice device/camera