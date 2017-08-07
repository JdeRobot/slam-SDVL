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

#ifndef SDVL_EXTRA_ORB_DETECTOR_H_
#define SDVL_EXTRA_ORB_DETECTOR_H_

#include <opencv/cvaux.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <Eigen/Dense>

namespace sdvl {

class ORBDetector {
 public:
  ORBDetector();

  // Get descriptor for given feature
  bool GetDescriptor(const cv::Mat &src, const Eigen::Vector2i &pos, std::vector<uchar> *desc);

  // Bit set count
  int Distance(const std::vector<uchar> &a, const std::vector<uchar> &b);

 private:
  // Init parameters
  void InitParameters();

  // Get feature orientation in image
  double GetOrientation(const cv::Mat &src, const Eigen::Vector2i& p);

  // Check limits
  bool IsInsideLimits(const cv::Mat &src, const Eigen::Vector2i &p);

  std::vector<int> umax_;
  std::vector<cv::Point> pattern_;
};

}  // namespace sdvl

#endif  // SDVL_EXTRA_ORB_DETECTOR_H_
