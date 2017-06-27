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

#include "./camera.h"
#include "./config.h"

namespace sdvl {

Camera::Camera() {
  CameraParameters &params = Config::GetCameraParameters();
  width_ = params.width;
  height_ = params.height;
  fx_ = params.fx;
  fy_ = params.fy;
  u0_ = params.u0;
  v0_ = params.v0;

  this->SetDistortions(params.d1, params.d2, params.d3, params.d4, params.d5);
}

void Camera::SetDistortions(double d0, double d1, double d2, double d3, double d4) {
  d0_ = d0;
  d1_ = d1;
  d2_ = d2;
  d3_ = d3;
  d4_ = d4;

  if (d0_ == 0.0 && d0_ == 0.0 && d0_ == 0.0 && d0_ == 0.0 && d0_ == 0.0) {
    has_distortion_ = false;
    return;
  }

  has_distortion_ = true;

  // Calculate rectify map
  cvK_ = cv::Mat(3, 3, CV_64F, 0.0);
  cvD_ = cv::Mat(5, 1, CV_64F, 0.0);

  cvK_.at<double>(0, 0) = fx_;
  cvK_.at<double>(0, 2) = u0_;
  cvK_.at<double>(1, 1) = fy_;
  cvK_.at<double>(1, 2) = v0_;
  cvK_.at<double>(2, 2) = 1.0;
  cvD_.at<double>(0, 0) = d0_;
  cvD_.at<double>(1, 0) = d1_;
  cvD_.at<double>(2, 0) = d2_;
  cvD_.at<double>(3, 0) = d3_;
  cvD_.at<double>(4, 0) = d4_;
}

void Camera::Project(const Eigen::Vector3d &p3D, Eigen::Vector2d *p2D) const {
  (*p2D)(0) = u0_ + fx_ * p3D(0) / p3D(2);
  (*p2D)(1) = v0_ + fy_ * p3D(1) / p3D(2);
}

void Camera::Unproject(const Eigen::Vector2d &p2D, Eigen::Vector3d *p3D) const {
  (*p3D)(0) = (p2D(0) - u0_)/fx_;
  (*p3D)(1) = (p2D(1) - v0_)/fy_;
  (*p3D)(2) = 1.0;
  p3D->normalize();
}

Eigen::Vector2d Camera::Project(const Eigen::Vector3d &p3D) const {
  Eigen::Vector2d p2D;
  Project(p3D, &p2D);
  return p2D;
}

Eigen::Vector2d Camera::Project(const Eigen::Vector2d &p3D) const {
  Eigen::Vector3d p3D_aux(p3D(0), p3D(1), 1.0);
  Eigen::Vector2d p2D;
  Project(p3D_aux, &p2D);
  return p2D;
}

Eigen::Vector3d Camera::Unproject(const Eigen::Vector2d &p2D) const {
  Eigen::Vector3d p3D;
  Unproject(p2D, &p3D);
  return p3D;
}

void Camera::UndistortImage(const cv::Mat &in, cv::Mat *out) const {
  if (has_distortion_)
    cv::undistort(in, *out, cvK_, cvD_);
  else
    *out = in.clone();
}

}  // namespace sdvl
