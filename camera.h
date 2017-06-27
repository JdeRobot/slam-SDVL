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

#ifndef SDVL_CAMERA_H_
#define SDVL_CAMERA_H_

#include <iostream>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/cvaux.h>

namespace sdvl {

class Camera {
 public:
  Camera();

  inline double GetWidth() const {
    return width_;
  }

  inline double GetHeight() const {
    return height_;
  }

  inline double GetFx() const {
    return fx_;
  }

  inline double GetFy() const {
    return fy_;
  }

  inline double GetU0() const {
    return u0_;
  }

  inline double GetV0() const {
    return v0_;
  }

  inline double GetD0() const {
    return d0_;
  }

  inline double GetD1() const {
    return d1_;
  }

  inline double GetD2() const {
    return d2_;
  }

  inline double GetD3() const {
    return d3_;
  }

  inline double GetD4() const {
    return d4_;
  }

  // Set camera distortions
  void SetDistortions(double d0, double d1, double d2, double d3, double d4);

  // Project/Unproject a point between camera and world coordinates
  void Project(const Eigen::Vector3d &p3D, Eigen::Vector2d *p2D) const;
  void Unproject(const Eigen::Vector2d &p2D, Eigen::Vector3d *p3D) const;
  Eigen::Vector2d Project(const Eigen::Vector3d &p3D) const;
  Eigen::Vector2d Project(const Eigen::Vector2d &p3D) const;
  Eigen::Vector3d Unproject(const Eigen::Vector2d &p2D) const;

  // Check if a pixel is inside image (Optional margin and level)
  inline bool IsInsideImage(const Eigen::Vector2i &p2D, int m = 0) const {
    return p2D(0) >= m && p2D(0) < width_-m && p2D(1) >= m && p2D(1) < height_-m;
  }
  inline bool IsInsideImage(const Eigen::Vector2i &p2D, int m, int l) const {
    return p2D(0) >= m && p2D(0) < width_/(1 << l)-m && p2D(1) >= m && p2D(1) < height_/(1 << l)-m;
  }

  // Undistort an image taking into account distortion parameters
  void UndistortImage(const cv::Mat &in, cv::Mat *out) const;

  // Get pixel error angle (Law of chords)
  inline double GetPixelErrorAngle() const {
    double px_noise = 1.0;
    return atan(px_noise/(2.0*fx_))*2.0;
  }

  // Project/unproject without using camera parameters
  inline static Eigen::Vector2d SimpleProject(const Eigen::Vector3d& p3D) {
    return Eigen::Vector2d(p3D(0)/p3D(2), p3D(1)/p3D(2));
  }

  inline static Eigen::Vector3d SimpleUnproject(const Eigen::Vector2d &p2D) {
    return Eigen::Vector3d(p2D(0), p2D(1), 1.0);
  }

 private:
  double width_;
  double height_;
  double fx_;
  double fy_;
  double u0_;
  double v0_;

  // Distortions
  double d0_;
  double d1_;
  double d2_;
  double d3_;
  double d4_;
  double has_distortion_;
  cv::Mat cvK_;
  cv::Mat cvD_;
};

}  // namespace sdvl


#endif  // SDVL_CAMERA_H_
