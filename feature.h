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

#ifndef SDVL_FEATURE_H_
#define SDVL_FEATURE_H_

#include <iostream>
#include <memory>
#include <vector>
#include <Eigen/Dense>
#include "./frame.h"
#include "./point.h"

namespace sdvl {

class Point;
class Frame;

// Save information about each feature detected in Frame
class Feature {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Feature(const std::shared_ptr<Frame> &f, const Eigen::Vector2d& p, int l);
  Feature(const std::shared_ptr<Frame> &f, const std::shared_ptr<Point> &ft, const Eigen::Vector2d& p, int l);
  Feature(const std::shared_ptr<Frame> &f, const std::shared_ptr<Point> &ft, const Eigen::Vector2d& p, const Eigen::Vector3d& v, int l);

  inline std::shared_ptr<Frame> GetFrame() {
    return frame_;
  }

  inline void SetFrame(const std::shared_ptr<Frame> &f) {
    frame_ = f;
  }

  inline std::shared_ptr<Point> GetPoint() const {
    return point_;
  }

  inline void SetPoint(const std::shared_ptr<Point> &f) {
    point_ = f;
  }

  inline const Eigen::Vector2d & GetPosition() const {
    return p2d_;
  }

  inline const Eigen::Vector3d & GetVector() const {
    return v_;
  }

  inline const void SetVector(Eigen::Vector3d &v) {
    v_ = v;
  }

  inline int GetLevel() const {
    return level_;
  }

  inline const std::vector<uchar> & GetDescriptor() const {
    return descriptor_;
  }

  inline void SetDescriptor(const std::vector<uchar> &d) {
    copy(d.begin(), d.end(), descriptor_.begin());
    has_descriptor_ = true;
  }

  inline bool HasDescriptor() const {
    return has_descriptor_;
  }

  // Get position in level where it was detected
  inline Eigen::Vector2d GetLevelPosition() {
    return p2d_/(1 << level_);
  }

 private:
  std::shared_ptr<Frame> frame_;      // Frame where feature was detected
  std::shared_ptr<Point> point_;      // Point related to this feature
  Eigen::Vector2d p2d_;               // 2D image position
  Eigen::Vector3d v_;                 // 3D vector
  int level_;                         // Pyramid level where it was detected

  std::vector<uchar> descriptor_;   // ORB descriptor
  bool has_descriptor_;             // True if descriptor has been calculated
};

}  // namespace sdvl

#endif  // SDVL_FEATURE_H_
