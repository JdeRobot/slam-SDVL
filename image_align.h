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

#ifndef SDVL_IMAGE_ALIGN_H_
#define SDVL_IMAGE_ALIGN_H_

#include <iostream>
#include <vector>
#include "./frame.h"
#include "extra/utils.h"


namespace sdvl {

class ImageAlign {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImageAlign();
  ~ImageAlign();

  // Compute Pose between frames
  int ComputePose(const std::shared_ptr<Frame> &frame1, const std::shared_ptr<Frame> &frame2, bool fast = false);

  inline double GetError() { return error_; }

 private:
  // Optimize using Gauss Newton strategy
  void Optimize(SE3 *se3, int level);
  // Compute residual and jacobians
  double ComputeResiduals(const SE3 &se3, int level, bool linearize, bool patches);
  // Compute patches within a pyramid level
  void PrecomputePatches(int level);

  std::shared_ptr<Frame> frame1_;      // First frame
  std::shared_ptr<Frame> frame2_;      // Second frame

  double chi2_;
  cv::Mat patch_cache_;             // Cache for patches
  std::vector<bool> visible_fts_;   // Visible points
  size_t  n_meas_;                  // Number of measurements
  bool stop_;                       // Stop flag
  double error_;                    // Last optimization error

  Eigen::Matrix<double, 6, 6>  H_;      // Hessian approximation
  Eigen::Matrix<double, 6, 1>  Jres_;   // Store Jacobian residual
  Eigen::Matrix<double, 6, Eigen::Dynamic, Eigen::ColMajor> jacobian_cache_;
};

}  // namespace sdvl

#endif  // SDVL_IMAGE_ALIGN_H_
