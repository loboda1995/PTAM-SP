// Copyright 2008 Isis Innovation Limited
#include "MapPoint.h"
#include "KeyFrame.h"
#include "ATANCamera.h"
#include "MapMaker.h"
#include "LevelHelpers.h"

void MapPoint::RefreshPixelVectors()
{
  KeyFrame &k = *pPatchSourceKF;
  
  // Find patch pos in KF camera coords
  // Actually this might not exactly correspond to the patch pos!
  // Treat it as a general point on the plane.
  Vector<3> v3PlanePoint_C = k.se3CfromW * v3WorldPos;
  
  // Find the height of this above the plane.
  // Assumes the normal is  pointing toward the camera.
  double dCamHeight = fabs(v3PlanePoint_C * v3Normal_NC);

  double dPixelRate = fabs(v3Center_NC * v3Normal_NC);
  double dOneRightRate = fabs(v3OneRightFromCenter_NC * v3Normal_NC);
  double dOneDownRate = fabs(v3OneDownFromCenter_NC * v3Normal_NC);
  
  // Find projections onto plane
  Vector<3> v3CenterOnPlane_C = v3Center_NC * dCamHeight / dPixelRate;
  Vector<3> v3OneRightOnPlane_C = v3OneRightFromCenter_NC * dCamHeight / dOneRightRate;
  Vector<3> v3OneDownOnPlane_C = v3OneDownFromCenter_NC * dCamHeight / dOneDownRate;
  
  // Find differences of these projections in the world frame
  v3PixelRight_W = k.se3CfromW.get_rotation().inverse() * (v3OneRightOnPlane_C - v3CenterOnPlane_C);
  v3PixelDown_W = k.se3CfromW.get_rotation().inverse() * (v3OneDownOnPlane_C - v3CenterOnPlane_C);
}

void MapPoint::SetSourcePatch(ATANCamera &cam, KeyFrame *kf, int level, CVD::ImageRef center, Vector<2> zeroPos) {
    int nLevelScale = LevelScale(level);

    pPatchSourceKF = kf;
    nSourceLevel = level;
    v3Normal_NC = makeVector(0, 0, -1);
    irCenter = center;
    v3Center_NC = unproject(cam.UnProject(zeroPos));
    v3OneDownFromCenter_NC = unproject(cam.UnProject(zeroPos + vec(CVD::ImageRef(0, nLevelScale))));
    v3OneRightFromCenter_NC = unproject(cam.UnProject(zeroPos + vec(CVD::ImageRef(nLevelScale, 0))));
    normalize(v3Center_NC);
    normalize(v3OneDownFromCenter_NC);
    normalize(v3OneRightFromCenter_NC);
    RefreshPixelVectors();
    pMMData = new MapMakerData();
}
