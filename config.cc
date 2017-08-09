/*
 *  Copyright (C) 1997-2017 JdeRobot Developers Team
 *
 *  This program is free software; you can redistribute it and/or modifdisty
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 *   Authors : Eduardo Perdices <eperdices@gsyc.es>
 *
 */

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include "./config.h"

using std::cerr;
using std::endl;

namespace sdvl {

Config::Config() {
  // Default values
  camera_params_.width = 640;
  camera_params_.height = 480;
  camera_params_.fx = 300.0;
  camera_params_.fy = 300.0;
  camera_params_.u0 = 320.0;
  camera_params_.v0 = 240.0;
  camera_params_.d1 = 0.0;
  camera_params_.d2 = 0.0;
  camera_params_.d3 = 0.0;
  camera_params_.d4 = 0.0;
  camera_params_.d5 = 0.0;

  video_params_.type = 0;
  video_params_.device = 0;
  video_params_.width = 640;
  video_params_.height = 480;
  video_params_.fps = 30;
  video_params_.path = "";
  video_params_.filename = "";

  kPyramidLevels_ = 5;
  kCellSize_ = 32;
  kMinAvgShift_ = 50;
  kMaxMatches_ = 150;
  kMinMatches_ = 20;
  kMaxKeyframes_ = 100;
  kMinKeyframeIts_ = 30;
  kMaxFailed_ = 15;
  kMaxSearchKeyframes_ = 5;
  kMaxOptimPoseIts_ = 10;
  kMaxRansacPoints_ = 5;
  kMaxRansacIts_ = 100;
  kThresholdConverged_ = 0.1;
  kMinInitCorners_ = 50;
  kInlierErrorThreshold_ = 2.0;
  kMapScale_ = 1.0;
  kMaxAlignLevel_ = 4;
  kMinAlignLevel_ = 2;
  kMaxImgAlignIts_ = 30;
  kAlignPatchSize_ = 4;
  kScaleMinDist_ = 0.25;
  kLostRatio_ = 0.7;
  kPatchSize_ = 8;
  kMaxAlignIts_ = 10;
  kSearchSize_ = 6;
  kUseORB_ = false;
  kORBSize_ = 31;
  kMaxFastLevels_ = 3;
  kFastThreshold_ = 10;
  kMinFeatureScore_ = 50;
  kNumFeatures_ = 1000;
}

bool Config::ReadParameters(std::string filename) {
  cv::FileStorage fs;

  try {
    // Read config file
    fs.open(filename.c_str(), cv::FileStorage::READ);
    if (!fs.isOpened()) {
      cerr << "[ERROR] Failed to open file: " << filename << endl;
      return false;
    }
  } catch(cv::Exception &ex) {
    cerr << "[ERROR] Parse error: " << ex.what();
    return false;
  }

  // Camera parameters
  if (fs["Camera.width"].isNamed()) fs["Camera.width"] >> camera_params_.width;
  if (fs["Camera.height"].isNamed()) fs["Camera.height"] >> camera_params_.height;
  if (fs["Camera.fx"].isNamed()) fs["Camera.fx"] >> camera_params_.fx;
  if (fs["Camera.fy"].isNamed()) fs["Camera.fy"] >> camera_params_.fy;
  if (fs["Camera.u0"].isNamed()) fs["Camera.u0"] >> camera_params_.u0;
  if (fs["Camera.v0"].isNamed()) fs["Camera.v0"] >> camera_params_.v0;
  if (fs["Camera.d1"].isNamed()) fs["Camera.d1"] >> camera_params_.d1;
  if (fs["Camera.d2"].isNamed()) fs["Camera.d2"] >> camera_params_.d2;
  if (fs["Camera.d3"].isNamed()) fs["Camera.d3"] >> camera_params_.d3;
  if (fs["Camera.d4"].isNamed()) fs["Camera.d4"] >> camera_params_.d4;
  if (fs["Camera.d5"].isNamed()) fs["Camera.d5"] >> camera_params_.d5;

  // Video parameters
  if (fs["Video.type"].isNamed()) fs["Video.type"] >> video_params_.type;
  if (video_params_.type == 0) {
    if (fs["Video.device"].isNamed()) fs["Video.device"] >> video_params_.device;
    if (fs["Video.width"].isNamed()) fs["Video.width"] >> video_params_.width;
    if (fs["Video.height"].isNamed()) fs["Video.height"] >> video_params_.height;
    if (fs["Video.fps"].isNamed()) fs["Video.fps"] >> video_params_.fps;
  } else {
    if (fs["Video.path"].isNamed()) fs["Video.path"] >> video_params_.path;
    if (fs["Video.filename"].isNamed()) fs["Video.filename"] >> video_params_.filename;
  }

  // SDVL parameters
  if (fs["SDVL.pyramid_levels"].isNamed()) fs["SDVL.pyramid_levels"] >> kPyramidLevels_;
  if (fs["SDVL.cell_size"].isNamed()) fs["SDVL.cell_size"] >> kCellSize_;
  if (fs["SDVL.min_avg_shift"].isNamed()) fs["SDVL.min_avg_shift"] >> kMinAvgShift_;
  if (fs["SDVL.max_matches"].isNamed()) fs["SDVL.max_matches"] >> kMaxMatches_;
  if (fs["SDVL.min_matches"].isNamed()) fs["SDVL.min_matches"] >> kMinMatches_;
  if (fs["SDVL.max_keyframes"].isNamed()) fs["SDVL.max_keyframes"] >> kMaxKeyframes_;
  if (fs["SDVL.min_keyframe_its"].isNamed()) fs["SDVL.min_keyframe_its"] >> kMinKeyframeIts_;
  if (fs["SDVL.max_failed"].isNamed()) fs["SDVL.max_failed"] >> kMaxFailed_;
  if (fs["SDVL.max_search_keyframes"].isNamed()) fs["SDVL.max_search_keyframes"] >> kMaxSearchKeyframes_;
  if (fs["SDVL.max_optim_pose_its"].isNamed()) fs["SDVL.max_optim_pose_its"] >> kMaxOptimPoseIts_;
  if (fs["SDVL.max_ransac_points"].isNamed()) fs["SDVL.max_ransac_points"] >> kMaxRansacPoints_;
  if (fs["SDVL.max_ransac_its"].isNamed()) fs["SDVL.max_ransac_its"] >> kMaxRansacIts_;
  if (fs["SDVL.threshold_converged"].isNamed()) fs["SDVL.threshold_converged"] >> kThresholdConverged_;
  if (fs["SDVL.min_init_corners"].isNamed()) fs["SDVL.min_init_corners"] >> kMinInitCorners_;
  if (fs["SDVL.inlier_error_threshold"].isNamed()) fs["SDVL.inlier_error_threshold"] >> kInlierErrorThreshold_;
  if (fs["SDVL.map_scale"].isNamed()) fs["SDVL.map_scale"] >> kMapScale_;
  if (fs["SDVL.max_alignLevel"].isNamed()) fs["SDVL.max_alignLevel"] >> kMaxAlignLevel_;
  if (fs["SDVL.min_alignLevel"].isNamed()) fs["SDVL.min_alignLevel"] >> kMinAlignLevel_;
  if (fs["SDVL.max_img_align_its"].isNamed()) fs["SDVL.max_img_align_its"] >> kMaxImgAlignIts_;
  if (fs["SDVL.align_patch_size"].isNamed()) fs["SDVL.align_patch_size"] >> kAlignPatchSize_;
  if (fs["SDVL.scale_min_dist"].isNamed()) fs["SDVL.scale_min_dist"] >> kScaleMinDist_;
  if (fs["SDVL.lost_ratio"].isNamed()) fs["SDVL.lost_ratio"] >> kLostRatio_;
  if (fs["SDVL.patch_size"].isNamed()) fs["SDVL.patch_size"] >> kPatchSize_;
  if (fs["SDVL.max_align_its"].isNamed()) fs["SDVL.max_align_its"] >> kMaxAlignIts_;
  if (fs["SDVL.search_size"].isNamed()) fs["SDVL.search_size"] >> kSearchSize_;
  if (fs["SDVL.use_orb"].isNamed()) fs["SDVL.use_orb"] >> kUseORB_;
  if (fs["SDVL.orb_size"].isNamed()) fs["SDVL.orb_size"] >> kORBSize_;
  if (fs["SDVL.max_fast_levels"].isNamed()) fs["SDVL.max_fast_levels"] >> kMaxFastLevels_;
  if (fs["SDVL.fast_threshold"].isNamed()) fs["SDVL.fast_threshold"] >> kFastThreshold_;
  if (fs["SDVL.min_feature_score"].isNamed()) fs["SDVL.min_feature_score"] >> kMinFeatureScore_;
  if (fs["SDVL.num_features"].isNamed()) fs["SDVL.num_features"] >> kNumFeatures_;

  fs.release();

  return true;
}

}  // namespace sdvl
