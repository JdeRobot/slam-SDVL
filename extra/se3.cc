/*
 *  Copyright (C) 1997-2017 JdeRobot Developers Team
 *
 *  This program is free software; you can redistribute it and/or modifdisty_
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANty_; without even the implied warranty_ of
 *  MERCHANTABILIty_ or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 *   Authors : Eduardo Perdices <eperdices@gsyc.es>
 *
 */

#include "./se3.h"

namespace sdvl {

SE3::SE3() {
  t_.setZero();
  q0_ = 1.0;
  q1_ = 0.0;
  q2_ = 0.0;
  q3_ = 0.0;
}

SE3::SE3(const Eigen::Quaterniond &q, const Eigen::Vector3d &t) {
  t_ = t;
  q0_ = q.w();
  q1_ = q.x();
  q2_ = q.y();
  q3_ = q.z();
}

SE3::SE3(const Eigen::Matrix3d &r, const Eigen::Vector3d &t) {
  t_ = t;
  Eigen::Quaterniond q = Eigen::Quaterniond(r);
  q0_ = q.w();
  q1_ = q.x();
  q2_ = q.y();
  q3_ = q.z();
}

void SE3::Restart() {
  t_.setZero();
  q0_ = 1.0;
  q1_ = 0.0;
  q2_ = 0.0;
  q3_ = 0.0;
}

SE3 SE3::Inverse() const {
  SE3 result;

  Eigen::Quaterniond q = Eigen::Quaterniond(q0_, q1_, q2_, q3_).inverse();
  result.q0_ = q.w();
  result.q1_ = q.x();
  result.q2_ = q.y();
  result.q3_ = q.z();
  result.t_ = -(q.toRotationMatrix() * t_);

  return result;
}

SE3 SE3::Exp(const Eigen::Matrix<double, 6, 1> &update) {
  Eigen::Vector3d upsilon = update.head<3>();
  Eigen::Vector3d omega = update.tail<3>();

  double theta;
  Eigen::Quaterniond q = SE3::RotationExp(omega, &theta);
  Eigen::Matrix3d Omega = SE3::RotationHat(omega);
  Eigen::Matrix3d Omega_sq = Omega*Omega;
  Eigen::Matrix3d V;

  if (theta < SMALL_EPS) {
    V = q.toRotationMatrix();
    // Note: That is an accurate expansion!
  } else {
    double theta_sq = theta*theta;
    V = (Eigen::Matrix3d::Identity()
         + (1-cos(theta))/(theta_sq)*Omega
         + (theta-sin(theta))/(theta_sq*theta)*Omega_sq);
  }

  Eigen::Vector3d t = V*upsilon;
  return SE3(q, t);
}

Eigen::Matrix<double, 6, 1> SE3::Log(const SE3 &se3) {
  Eigen::Matrix<double, 6, 1> upsilon_omega;
  double theta;
  upsilon_omega.tail<3>() = RotationLog(se3.GetQuaternion(), &theta);

  if (theta < SMALL_EPS) {
    Eigen::Matrix3d Omega = RotationHat(upsilon_omega.tail<3>());
    Eigen::Matrix3d V_inv = Eigen::Matrix3d::Identity()- 0.5*Omega + (1./12.)*(Omega*Omega);

    upsilon_omega.head<3>() = V_inv*se3.GetTranslation();
  } else {
    Eigen::Matrix3d Omega = RotationHat(upsilon_omega.tail<3>());
    Eigen::Matrix3d V_inv = (Eigen::Matrix3d::Identity() - 0.5*Omega + (1-theta/(2*tan(theta/2)))/(theta*theta)*(Omega*Omega));
    upsilon_omega.head<3>() = V_inv*se3.GetTranslation();
  }
  return upsilon_omega;
}

Eigen::Quaterniond SE3::RotationExp(const Eigen::Vector3d &omega, double *theta) {
  *theta = omega.norm();
  double half_theta = 0.5*(*theta);

  double imag_factor;
  double real_factor = cos(half_theta);
  if ((*theta) < SMALL_EPS) {
    double theta_sq = (*theta)*(*theta);
    double theta_po4 = theta_sq*theta_sq;
    imag_factor = 0.5-0.0208333*theta_sq+0.000260417*theta_po4;
  } else {
    double sin_half_theta = sin(half_theta);
    imag_factor = sin_half_theta/(*theta);
  }

  return Eigen::Quaterniond(real_factor, imag_factor*omega.x(), imag_factor*omega.y(), imag_factor*omega.z());
}

Eigen::Matrix3d SE3::RotationHat(const Eigen::Vector3d &v) {
  Eigen::Matrix3d Omega;
  Omega <<  0, -v(2),  v(1)
      ,  v(2),     0, -v(0)
      , -v(1),  v(0),     0;
  return Omega;
}

Eigen::Vector3d SE3::RotationLog(const Eigen::Quaterniond &other, double *theta) {
  double n = other.vec().norm();
  double w = other.w();
  double squared_w = w*w;
  double two_atan_nbyw_by_n;

  if (n < SMALL_EPS) {
    // If quaternion is normalized and n=1, then w should be 1;
    // w=0 should never happen here!
    assert(fabs(w) > SMALL_EPS);
    two_atan_nbyw_by_n = 2./w - 2.*(n*n)/(w*squared_w);
  } else  {
    if (fabs(w) < SMALL_EPS) {
      if (w > 0) {
        two_atan_nbyw_by_n = M_PI/n;
      } else {
        two_atan_nbyw_by_n = -M_PI/n;
      }
    }
    two_atan_nbyw_by_n = 2*atan(n/w)/n;
  }

  *theta = two_atan_nbyw_by_n*n;
  return two_atan_nbyw_by_n * other.vec();
}

SE3 SE3::operator*(const SE3 &se3) const {
  SE3 result;

  Eigen::Quaterniond q = GetQuaternion()*se3.GetQuaternion();
  q.normalize();
  result.q0_ = q.w();
  result.q1_ = q.x();
  result.q2_ = q.y();
  result.q3_ = q.z();
  result.t_ = t_ + (GetRotation()*se3.t_);
  return result;
}

std::ostream& operator<<(std::ostream &os, const SE3 &se3) {
  Eigen::MatrixXd rt(3, 4);
  rt.block(0, 0, 3, 3) = se3.GetRotation();
  rt.block(0, 3, 3, 1) = se3.GetTranslation();
  os << rt;
  return os;
}

}  // namespace sdvl
