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

#ifndef SDVL_EXTRA_SE3_H_
#define SDVL_EXTRA_SE3_H_

#include <iostream>
#include <Eigen/Dense>

namespace sdvl {

const double SMALL_EPS = 1e-10;

class SE3 {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SE3();
  SE3(const Eigen::Quaterniond &q, const Eigen::Vector3d &t);
  SE3(const Eigen::Matrix3d &r, const Eigen::Vector3d &t);

  inline Eigen::Vector3d GetTranslation() const { return t_; }
  inline Eigen::Matrix3d GetRotation() const { return Eigen::Quaterniond(q0_, q1_, q2_, q3_).toRotationMatrix(); }
  inline Eigen::Quaterniond GetQuaternion() const { return Eigen::Quaterniond(q0_, q1_, q2_, q3_); }

  inline void SetTranslation(const Eigen::Vector3d &t) { t_ = t; }
  inline void SetRotation(const Eigen::Matrix3d &r) {
    Eigen::Quaterniond q = Eigen::Quaterniond(r);
    q0_ = q.w(); q1_ = q.x(); q2_ = q.y(); q3_ = q.z();
  }
  inline void SetQuaternion(const Eigen::Quaterniond &q) {
    q0_ = q.w(); q1_ = q.x(); q2_ = q.y(); q3_ = q.z();
  }

  // Restart SE3
  void Restart();

  // Calculate SE3 inverse
  SE3 Inverse() const;

  static SE3 Exp(const Eigen::Matrix<double, 6, 1> &update);
  static Eigen::Matrix<double, 6, 1> Log(const SE3 &se3);

  // Operators
  inline SE3& operator=(const SE3 &se3) {
    t_ = se3.t_;
    q0_ = se3.q0_; q1_ = se3.q1_; q2_ = se3.q2_; q3_ = se3.q3_;
    return *this;
  }
  inline Eigen::Vector3d operator*(const Eigen::Vector3d &pos) const { return GetRotation()*pos + t_; }
  SE3 operator*(const SE3 &se3) const;

 private:
  static Eigen::Quaterniond RotationExp(const Eigen::Vector3d &omega, double *theta);
  static Eigen::Matrix3d RotationHat(const Eigen::Vector3d &v);
  static Eigen::Vector3d RotationLog(const Eigen::Quaterniond &other, double *theta);

  Eigen::Vector3d t_;         // Translation vector
  double q0_, q1_, q2_, q3_;  // Quaternion vector
};

std::ostream& operator<< (std::ostream &os, const SE3 &se3);

}  // namespace sdvl


#endif  // SDVL_EXTRA_SE3_H_
