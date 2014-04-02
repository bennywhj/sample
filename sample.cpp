/*
   IRTES-SET laboratory
   Authors: You Li (liyou026@gmail.com)
   Descirption: This a sample code of my PhD works
*/

#include <iostream>

#include "basicStructure.hpp"
#include "vo_stereo.hpp"
#include "uvdisparity.hpp"

using namespace std;
using namespace cv;

//set calibration parameters for stereo camera
double f = 1007.41278; //focal length in pixels
double c_u = 611.10115; //principal point (u-coordinate) in pixels
double c_v = 479.39295; //principal point (v-coordinate) in pixels
double base = 0.24011825; //baseline in meters


int main()
{

  // set visual odometry parameters
  VisualOdometryStereo::parameters param;
  // set calibration paramerers for odometry
  param.calib.f  = f;      param.calib.cu = c_u;
  param.calib.cv = c_v;    param.base     = base;
  // RANSAC parameter for classifying inlier and outlier
  param.inlier_threshold = 6.0f; //1.2f;

  //set the ROI region for the field of view
  ROI3D roi_(10000,0,30000);//x,y,z
  
  // init visual odometry
  VisualOdometryStereo viso(param);

  /*initialize the parameters of UVDisparity*/
  CalibPars calib_(f,c_u,c_v,base);
  UVDisparity uv_disparity;
  uv_disparity.SetCalibPars(calib_); uv_disparity.SetROI3D(roi_);
  uv_disparity.SetOutThreshold(6.0f); uv_disparity.SetInlierTolerance(3);
  uv_disparity.SetMinAdjustIntense(20);

  //Read consecutive stereo images
  string imgpath_lp = "images/L_755.png"; string imgpath_rp = "images/R_755.png";
  string imgpath_lc = "images/L_756.png"; string imgpath_rc = "images/R_756.png";
  cv::Mat img_lc,img_lp,img_rc,img_rp;

  img_lc = cv::imread(imgpath_lc,0);img_lp = cv::imread(imgpath_lp,0);
  img_rc = cv::imread(imgpath_rc,0);img_rp = cv::imread(imgpath_rp,0);

  //detect and matching feature points circlely in four images
  QuadFeatureMatch* quadmatcher = new QuadFeatureMatch(img_lc,img_rc,img_lp,img_rp,true);
  quadmatcher->init(DET_GFTT,DES_SIFT);
  quadmatcher->detectFeature();
  quadmatcher->circularMatching();
  cout<<"Detected features"<<quadmatcher->quadmatches.size()<<endl;

  if(viso.Process(*quadmatcher)==true)
  {
      cv::Mat motion;
      //get ego-motion matrix (6DOF)
      motion = viso.getMotion();
      cout<<"Ego-motion: "<<motion<<endl;

      //computing disparity image (SGBM or BM method) and 3D reconstruction by triangulation
      cv::Mat disp_sgbm,disp_show_sgbm;
      cv:: Mat xyz, roi_mask;
      calDisparity_SGBM(img_lc,img_rc,disp_sgbm,disp_show_sgbm);
      triangulate10D(img_lc,disp_sgbm, xyz,f,c_u,c_v,base,roi_);

      //analyzing the stereo image by U-V disparity images (here I put moving object detection result)
      cv::Mat moving_mask, ground_mask;
      double pitch1,pitch2;
      moving_mask = uv_disparity.Process(img_lc, disp_sgbm, viso, xyz,roi_mask,ground_mask,pitch1,pitch2);
  }

  delete quadmatcher;

}



































