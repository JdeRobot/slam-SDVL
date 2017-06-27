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

#include <libconfig.h++>
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
  kCellSize_ = 25;
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
}

bool Config::ReadParameters(std::string filename) {
  libconfig::Config cfg;

  // Read config file
  try {
    cfg.readFile(filename.c_str());
  } catch(const libconfig::FileIOException &fioex) {
    cerr << "[ERROR] I/O error while reading file." << endl;
    return false;
  } catch(const libconfig::ParseException &pex) {
    cerr << "[ERROR] Parse error at " << pex.getFile() << ":" << pex.getLine() << " - " << pex.getError() << endl;
    return false;
  }

  const libconfig::Setting& root = cfg.getRoot();

  // Camera parameters
  try {
    const libconfig::Setting &camsettings = root["camera"];
    camsettings.lookupValue("width", camera_params_.width);
    camsettings.lookupValue("height", camera_params_.height);
    camsettings.lookupValue("fx", camera_params_.fx);
    camsettings.lookupValue("fy", camera_params_.fy);
    camsettings.lookupValue("u0", camera_params_.u0);
    camsettings.lookupValue("v0", camera_params_.v0);
    camsettings.lookupValue("d1", camera_params_.d1);
    camsettings.lookupValue("d2", camera_params_.d2);
    camsettings.lookupValue("d3", camera_params_.d3);
    camsettings.lookupValue("d4", camera_params_.d4);
    camsettings.lookupValue("d5", camera_params_.d5);
  } catch(const libconfig::SettingNotFoundException &nfex) {
    cerr << "[ERROR] Config error: Camera parameters not found" << endl;
    return false;
  }

  // Video parameters
  try {
    const libconfig::Setting &videosettings = root["video"];
    videosettings.lookupValue("type", video_params_.type);
    if (video_params_.type == 0) {
      videosettings.lookupValue("device", video_params_.device);
      videosettings.lookupValue("width", video_params_.width);
      videosettings.lookupValue("height", video_params_.height);
      videosettings.lookupValue("fps", video_params_.fps);
    } else {
      videosettings.lookupValue("path", video_params_.path);
      videosettings.lookupValue("filename", video_params_.filename);
    }
  } catch(const libconfig::SettingNotFoundException &nfex) {
    cerr << "[ERROR] Config error: Video parameters not found" << endl;
    return false;
  }

  // Main parameters
  try {
    const libconfig::Setting &mainsettings = root["sdvl"];
    mainsettings.lookupValue("pyramid_levels", kPyramidLevels_);
    mainsettings.lookupValue("cell_size", kCellSize_);
    mainsettings.lookupValue("min_avg_shift", kMinAvgShift_);
    mainsettings.lookupValue("max_matches", kMaxMatches_);
    mainsettings.lookupValue("min_matches", kMinMatches_);
    mainsettings.lookupValue("max_keyframes", kMaxKeyframes_);
    mainsettings.lookupValue("min_keyframe_its", kMinKeyframeIts_);
    mainsettings.lookupValue("max_failed", kMaxFailed_);
    mainsettings.lookupValue("max_search_keyframes", kMaxSearchKeyframes_);
    mainsettings.lookupValue("max_optim_pose_its", kMaxOptimPoseIts_);
    mainsettings.lookupValue("max_ransac_points", kMaxRansacPoints_);
    mainsettings.lookupValue("max_ransac_its", kMaxRansacIts_);
    mainsettings.lookupValue("threshold_converged", kThresholdConverged_);
    mainsettings.lookupValue("min_init_corners", kMinInitCorners_);
    mainsettings.lookupValue("inlier_error_threshold", kInlierErrorThreshold_);
    mainsettings.lookupValue("map_scale", kMapScale_);
    mainsettings.lookupValue("max_alignLevel", kMaxAlignLevel_);
    mainsettings.lookupValue("min_alignLevel", kMinAlignLevel_);
    mainsettings.lookupValue("max_img_align_its", kMaxImgAlignIts_);
    mainsettings.lookupValue("align_patch_size", kAlignPatchSize_);
    mainsettings.lookupValue("scale_min_dist", kScaleMinDist_);
    mainsettings.lookupValue("lost_ratio", kLostRatio_);
    mainsettings.lookupValue("patch_size", kPatchSize_);
    mainsettings.lookupValue("max_align_its", kMaxAlignIts_);
    mainsettings.lookupValue("search_size", kSearchSize_);
    mainsettings.lookupValue("use_orb", kUseORB_);
    mainsettings.lookupValue("orb_size", kORBSize_);
    mainsettings.lookupValue("max_fast_levels", kMaxFastLevels_);
    mainsettings.lookupValue("fast_threshold", kFastThreshold_);
    mainsettings.lookupValue("min_feature_score", kMinFeatureScore_);

  } catch(const libconfig::SettingNotFoundException &nfex) {
    cerr << "[ERROR] Config error: SDVL parameters not found" << endl;
    return false;
  }

  return true;
}

}  // namespace sdvl
