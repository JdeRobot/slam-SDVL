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

#ifndef SDVL_HOMOGRAPHY_INIT_H_
#define SDVL_HOMOGRAPHY_INIT_H_

#include <vector>
#include <memory>
#include "./map.h"
#include "./frame.h"
#include "extra/se3.h"
#include "extra/orb_detector.h"

namespace sdvl {

// Storage for each homography decomposition
struct HomographyDecomposition {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d v3Tp;
  Eigen::Matrix3d m3Rp;
  double d;
  Eigen::Vector3d v3n;
  // The resolved composition..
  SE3 se3;
  int score;
};

class HomographyInit {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  HomographyInit(Map * map, int min_shift);
  ~HomographyInit() {}

  bool InitFirstFrame(const std::shared_ptr<Frame> &frame);
  bool InitSecondFrame(const std::shared_ptr<Frame> &frame);
  void Reset();

  inline bool HasError() { return pixels1_.empty(); }

 private:
  // Track previous frame
  bool TrackSecondFrame();
  // Calc homography with saved frames data
  bool ComputeHomography();
  // Decompose an Homography
  bool DecomposeHomography(const Eigen::Matrix3d &H);
  // Check homography inliers
  void CheckInliers(const Eigen::Matrix3d &H, const std::vector<cv::Point2f> &src, const std::vector<cv::Point2f> &dst);
  // Choose best Homography decomposition
  bool ChooseBestDecomposition(const std::vector<cv::Point2f> &src, const std::vector<cv::Point2f> &dst);
  // Check decomposition inliers
  void CheckDecompositionInliers();

  // Calculate reprojection error
  double ReprojectionError(Camera * camera, const Eigen::Vector3d vector, const Eigen::Vector3d p3d);
  double ReprojectionError(Camera * camera, const Eigen::Matrix3d &R, const Eigen::Vector3d &T,
                           const Eigen::Vector3d vector, const Eigen::Vector3d p3d);

  double SampsonusError(const Eigen::Matrix3d &m3Essential, int i,
                        const std::vector<cv::Point2f> &src, const std::vector<cv::Point2f> &dst);

  Map * map_;
  ORBDetector detector_;

  std::shared_ptr<Frame> frame1_;           // First frame
  std::shared_ptr<Frame> frame2_;           // Second frame
  std::vector<cv::Point2f> pixels1_;        // 2D pixels in image
  std::vector<cv::Point2f> pixels2_;        // 2D pixels in image
  std::vector<Eigen::Vector3d> vectors1_;   // 3D vectors for each 2d pixel
  std::vector<Eigen::Vector3d> vectors2_;   // 3D vectors for each 2d pixel

  int min_shift_;   // Min average shift between features

  Eigen::Matrix3d bestH_;       // Best homography calculated
  std::vector<int> inliers_;    // Save indexes of inliers
  std::vector<HomographyDecomposition> decompositions_;   // Store homography decompositions
  std::vector<Eigen::Vector3d> triangulations_;           // Save valid triangulations
  SE3 se3_;
};

}  // namespace sdvl

#endif  // SDVL_HOMOGRAPHY_INIT_H_
