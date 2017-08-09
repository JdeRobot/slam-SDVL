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

#ifndef SDVL_CONFIG_H_
#define SDVL_CONFIG_H_

#include <iostream>
#include <string>

namespace sdvl {

struct CameraParameters {
  int width;
  int height;
  double fx;
  double fy;
  double u0;
  double v0;
  double d1;
  double d2;
  double d3;
  double d4;
  double d5;
};

struct VideoParameters {
  int type;
  int device;
  int width;
  int height;
  int fps;
  std::string path;
  std::string filename;
};

class Config {
 public:
  static Config& GetInstance() {
    static Config instance;
    return instance;
  }

  // Read parameters from file
  bool ReadParameters(std::string filename);

  static CameraParameters& GetCameraParameters() { return GetInstance().camera_params_; }
  static VideoParameters& GetVideoParameters() { return GetInstance().video_params_; }
  static int PyramidLevels() { return GetInstance().kPyramidLevels_; }
  static int CellSize() { return GetInstance().kCellSize_; }
  static int MinAvgShift() { return GetInstance().kMinAvgShift_; }
  static int MaxMatches() { return GetInstance().kMaxMatches_; }
  static int MinMatches() { return GetInstance().kMinMatches_; }
  static int MaxKeyframes() { return GetInstance().kMaxKeyframes_; }
  static int MinKeyframeIts() { return GetInstance().kMinKeyframeIts_; }

  static int MaxFailed() { return GetInstance().kMaxFailed_; }

  static int MaxSearchKeyframes() { return GetInstance().kMaxSearchKeyframes_; }
  static int MaxOptimPoseIts() { return GetInstance().kMaxOptimPoseIts_; }
  static int MaxRansacPoints() { return GetInstance().kMaxRansacPoints_; }
  static int MaxRansacIts() { return GetInstance().kMaxRansacIts_; }

  static double ThresholdConverged() { return GetInstance().kThresholdConverged_; }

  static int MinInitCorners() { return GetInstance().kMinInitCorners_; }
  static double InlierErrorThreshold() { return GetInstance().kInlierErrorThreshold_; }
  static double MapScale() { return GetInstance().kMapScale_; }

  static int MaxAlignLevel() { return GetInstance().kMaxAlignLevel_; }
  static int MinAlignLevel() { return GetInstance().kMinAlignLevel_; }
  static int MaxImgAlignIts() { return GetInstance().kMaxImgAlignIts_; }
  static int AlignPatchSize() { return GetInstance().kAlignPatchSize_; }

  static double ScaleMinDist() { return GetInstance().kScaleMinDist_; }
  static double LostRatio() { return GetInstance().kLostRatio_; }

  static int PatchSize() { return GetInstance().kPatchSize_; }
  static int MaxAlignIts() { return GetInstance().kMaxAlignIts_; }
  static int SearchSize() { return GetInstance().kSearchSize_; }
  static bool UseORB() { return GetInstance().kUseORB_; }
  static int ORBSize() { return GetInstance().kORBSize_; }

  static int MaxFastLevels() { return GetInstance().kMaxFastLevels_; }
  static int FastThreshold() { return GetInstance().kFastThreshold_; }
  static int MinFeatureScore() { return GetInstance().kMinFeatureScore_; }
  static int NumFeatures() { return GetInstance().kNumFeatures_; }

 private:
  Config();

  CameraParameters camera_params_;
  VideoParameters video_params_;

  // SDVL
  int kPyramidLevels_;    // Number of pyramid levels
  int kCellSize_;         // Cell size in image
  int kMinAvgShift_;      // Min average shift between features to initialize
  int kMaxMatches_;       // Max matches needed
  int kMinMatches_;       // Min matches needed
  int kMaxKeyframes_;     // Max number of keyframes saved
  int kMinKeyframeIts_;   // Min number of iterations before creating a new keyframe

  // Point
  int kMaxFailed_;            // Max failed projections to delete a point

  // Feature align
  int kMaxSearchKeyframes_;   // Max number of keyframes used to compare matches
  int kMaxOptimPoseIts_;      // Max number of iterations for pose optimization
  int kMaxRansacPoints_;      // Max number of selected points in RANSAC
  int kMaxRansacIts_;         // Max number of RANSAC iterations

  // Point candidate
  double kThresholdConverged_;  // Theshold of convergence

  // Homography
  int kMinInitCorners_;           // Min corners used to initialize
  double kInlierErrorThreshold_;  // Error threshold to be an inlier
  double kMapScale_;              // Map scale

  // Image align
  int kMaxAlignLevel_;    // Max level in pyramid
  int kMinAlignLevel_;    // Min level in pyramid
  int kMaxImgAlignIts_;   // Max image align iterations
  int kAlignPatchSize_;   // Patch size

  // Map
  double kScaleMinDist_;    // Min distance to point compared to map scale
  double kLostRatio_;       // Lost points ratio to create new Keyframes

  // Matcher
  int kPatchSize_;          // Path size for matches
  int kMaxAlignIts_;        // Max interations allowed to align point
  int kSearchSize_;         // Search size around each pixel in deep search
  bool kUseORB_;            // Use ORB descriptors
  int kORBSize_;            // ORB descriptor size

  // Fast
  int kMaxFastLevels_;    // Max number of pyramid levels in FAST
  int kFastThreshold_;    // FAST threahold
  int kMinFeatureScore_;  // Min feature Shi Tomasi score
  int kNumFeatures_;      // Number of features detected per frame
};

}  // namespace sdvl


#endif  // SDVL_CONFIG_H_
