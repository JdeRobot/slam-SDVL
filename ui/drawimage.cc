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

#include "./drawimage.h"
#include "../point.h"
#include "../config.h"

using std::vector;
using std::cerr;
using std::endl;

namespace sdvl {

DrawImage::DrawImage(Camera * camera) {
  refresh_time_ = 1;  // ms
  camera_ = camera;
  updated_ = false;

  cv::namedWindow("SDVL: Current Frame");

  // Image
  image_ = cv::Mat(Config::GetCameraParameters().width, Config::GetCameraParameters().height, CV_8UC3);
}

DrawImage::~DrawImage() {
}

void DrawImage::SetBackground(const cv::Mat &src) {
  // Convert to RGB
  std::unique_lock<std::mutex> lock(mutex_3D_);
  cv::cvtColor(src, image_, CV_GRAY2RGB);
  updated_ = true;
}

void DrawImage::SetFeatures(const vector<Eigen::Vector3i> &features) {
  CvScalar color;

  std::unique_lock<std::mutex> lock(mutex_3D_);
  for (vector<Eigen::Vector3i>::const_iterator it=features.begin(); it != features.end(); it++) {
    if ((*it)(2) == Point::P_FOUND)
      color = CV_RGB(0, 255, 0);
    else if ((*it)(2) == Point::P_NOT_FOUND || (*it)(2) == Point::P_OUTLIER)
      color = CV_RGB(255, 0, 0);
    else
      color = CV_RGB(255, 255, 0);
    cv::circle(image_, cvPoint((*it)(0), (*it)(1)), 3, color, 1, CV_AA, 0);
  }
}

void DrawImage::ShowImage() {
  if (!updated_)
    return;

  // Check quality
  if (quality_ == SDVL::TRACKING_BAD)
    SetChannel(&image_, 2, 200);
  if (quality_ == SDVL::TRACKING_INSUFFICIENT)
    SetChannel(&image_, 0, 200);

  cv::imshow("SDVL: Current Frame", image_);
  cv::waitKey(refresh_time_);
}

void DrawImage::SetChannel(cv::Mat *mat, unsigned int channel, unsigned char value) {
  std::unique_lock<std::mutex> lock(mutex_3D_);
  const int cols = mat->cols;
  const int step = mat->channels();
  const int rows = mat->rows;
  for (int y = 0; y < rows; y++) {
    // get pointer to the first byte to be changed in this row
    unsigned char *p_row = mat->ptr(y) + channel;
    unsigned char *row_end = p_row + cols*step;
    for (; p_row != row_end; p_row += step)
      *p_row = value;
  }
}

bool DrawImage::Project(double x, double y, double z, cv::Point *p) {
  Eigen::Vector3d p3D, rel_p;
  Eigen::Vector2d p2D;
  int limit = 2000;

  p3D << x, y, z;
  rel_p = pose_ * p3D;
  if (rel_p(2) < 0.0)
    return false;

  camera_->Project(rel_p, &p2D);
  p->x = p2D(0);
  p->y = p2D(1);
  if (p->x < -limit || p->x > limit)
    return false;
  if (p->y < -limit || p->y > limit)
    return false;
  return true;
}

}  // namespace sdvl

