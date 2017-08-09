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

#ifndef SDVL_MATCHER_H_
#define SDVL_MATCHER_H_

#include <iostream>
#include <vector>
#include "./frame.h"
#include "./point.h"
#include "extra/orb_detector.h"

namespace sdvl {

class Point;
class Frame;

const double MAX_SSD_PER_PIXEL = 500;
const int MIN_ORB_THRESHOLD = 100;

class Matcher {
 public:
  explicit Matcher(int size);
  ~Matcher();

  // Search a point in current frame close to a epipolar line
  bool SearchPoint(const std::shared_ptr<Frame> &frame, const std::shared_ptr<Feature> &feature,
                      double idepth, double idepth_std, bool fixed, Eigen::Vector2d *px, int *flevel);

 private:
  // Compute affine warp matrix
  void WarpMatrixAffine(Camera *camera, const Eigen::Vector2d &px, const Eigen::Vector3d &v,
                        double depth, const SE3 &pose, int level, Eigen::Matrix2d *res_matrix);

  // Get best search level according to affine warp matrix
  int GetSearchLevel(const Eigen::Matrix2d &matrix);

  // Create path with affine matrix
  void CreatePatch(const Eigen::Matrix2d& matrix, const cv::Mat& img, const Eigen::Vector2d& px,
                   int level, int search_level, uint8_t* border_patch, uint8_t* patch);

  // Subpixel refinement of a reference point patch with current image
  bool AlignPatch(const cv::Mat& img, uint8_t* border_patch, uint8_t* patch, Eigen::Vector2d *px);

  // Calculate patch Zero Mean Sum of Squared Differences
  void GetZMSSDScore(uint8_t *patch, int *sumA, int *sumAA);

  // Compare patch to previous calculated ZMSSD
  double CompareZMSSDScore(uint8_t *ref_patch, uint8_t *patch, int sumA, int sumAA, int cols, int *sumB, int *sumBB, int *sumAB);

  // Get features close to epipolar line
  void GetCornersInRange(const std::shared_ptr<Frame> &frame, const Eigen::Vector2d& pxa, const Eigen::Vector2d& pxb, int level, double range, std::vector<int> *indices);

  // Get corners close to a pixel
  void GetCornersInRange(const std::shared_ptr<Frame> &frame, const Eigen::Vector2d& cpos, int level, double range, std::vector<int> *indices);

  // Search best feature
  bool SearchFeatures(const std::shared_ptr<Frame> &frame, const std::vector<int> &indices, uint8_t* patch, Eigen::Vector2d *px, const std::vector<uchar> &desc);

  uint8_t *patch_;
  uint8_t *border_patch_;
  int patch_size_;

  ORBDetector detector_;
};

}  // namespace sdvl

#endif  // SDVL_MATCHER_H_
