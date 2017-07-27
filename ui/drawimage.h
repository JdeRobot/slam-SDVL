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

#ifndef SDVL_UI_DRAWIMAGE_H_
#define SDVL_UI_DRAWIMAGE_H_

#include <string>
#include <iostream>
#include <vector>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <Eigen/Dense>
#include "../camera.h"
#include "../sdvl.h"
#include "../extra/se3.h"

namespace sdvl {

class DrawImage {
 public:
  explicit DrawImage(Camera * camera);
  virtual ~DrawImage();

  // Save image and features
  void SetBackground(const cv::Mat &src);
  void SetFeatures(const std::vector<Eigen::Vector3i> &features);
  inline void SetCurrentPose(const sdvl::SE3 &pose) { pose_ = pose; }
  inline void SetTrackingQuality(SDVL::TrackingQuality quality) { quality_ = quality; }

  // Show currrent image
  void ShowImage();

 protected:
  // Change channel color
  void SetChannel(cv::Mat *mat, unsigned int channel, unsigned char value);

  // Project 3d point into image
  bool Project(double x, double y, double z, cv::Point *p);

  std::mutex mutex_3D_;
  int refresh_time_;
  sdvl::SE3 pose_;
  SDVL::TrackingQuality quality_;

  // Image
  Camera * camera_;
  cv::Mat image_;

  bool updated_;
};

}  // namespace sdvl

#endif  // SDVL_UI_DRAWIMAGE_H_
