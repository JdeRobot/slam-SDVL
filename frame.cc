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

#include "./frame.h"
#include "./config.h"
#include "extra/fast_detector.h"
#include "extra/utils.h"

using std::shared_ptr;
using std::vector;

namespace sdvl {

int Frame::counter_ = 0;

Frame::Frame(Camera* camera, ORBDetector * detector, const cv::Mat& img, bool corners) {
  id_ = counter_;
  kf_id_ = 0;
  camera_ = camera;
  orb_detector_ = detector;
  pyramid_levels_ = Config::PyramidLevels();
  pose_ = SE3();
  width_ = img.cols;
  height_ = img.rows;
  is_keyframe_ = false;
  selected_ = false;
  last_ba_ = -1;
  delete_ = false;

  // Create Image Pyramid
  CreatePyramid(img);

  // Create corners
  if (corners)
    CreateCorners(Config::MaxFastLevels(), Config::NumFeatures());

  counter_ += 1;
}

Frame::~Frame() {
  RemoveFeatures();

  for (auto it=descriptors_.begin(); it != descriptors_.end(); it++)
    it->clear();
  descriptors_.clear();
}

void Frame::SetKeyframe() {
  is_keyframe_ = true;
}

double Frame::GetSceneDepth() {
  double z;
  vector<double> depth_vec;

  for (auto it=features_.begin(); it != features_.end(); it++) {
    if (!(*it))
      continue;

    shared_ptr<Point> point = (*it)->GetPoint();
    if (!point)
      continue;

    // Transform to frame relative coordinates
    z = GetRelativePos(point->GetPosition())(2);
    depth_vec.push_back(z);
  }

  if (depth_vec.empty())
    return 0.0;

  return GetMedianVector(&depth_vec);
}

bool Frame::Project(const Eigen::Vector3d &p3D, Eigen::Vector2d *p2D) {
  Eigen::Vector3d rel_p;

  rel_p = GetRelativePos(p3D);
  if (rel_p(2) < 0.0)
    return false;  // Point behind camera

  camera_->Project(rel_p, p2D);
  return true;
}

bool Frame::IsPointVisible(const Eigen::Vector3d &p) {
  Eigen::Vector3d rel_p = GetRelativePos(p);
  if (rel_p(2) < 0.0)
    return false;  // Point is behind camera

  Eigen::Vector2d image_p;
  camera_->Project(rel_p, &image_p);
  return camera_->IsInsideImage(image_p.cast<int>());
}

void Frame::CreatePyramid(const cv::Mat& img) {
  pyramid_.resize(pyramid_levels_);
  pyramid_[0] = img;
  // Save each sublevel
  for (int i=1; i < pyramid_levels_; i++)
    cv::pyrDown(pyramid_[i-1], pyramid_[i], cv::Size(pyramid_[i-1].cols/2, pyramid_[i-1].rows/2));
}

void Frame::CreateCorners(int levels, int nfeatures) {
  FastDetector detector(width_, height_, false);

  // Get corners for each level
  detector.DetectPyramid(pyramid_, &corners_, nfeatures);

  // Reserve memory for ORB descriptors
  if (Config::UseORB())
    descriptors_.resize(corners_.size());
}

void Frame::FilterCorners() {
  int index, level;

  assert(filtered_corners_.empty());

  FastDetector detector(width_, height_);

  // Lock cells where we already have features
  for (auto it=features_.begin(); it != features_.end(); it++) {
    assert(*it != nullptr);
    detector.LockCell((*it)->GetPosition());
  }

  detector.FilterCorners(pyramid_, corners_, &filtered_corners_);

  if (Config::UseORB()) {
    vector<uchar> desc(32);

    // Calc ORB descriptors
    for (auto it=filtered_corners_.begin(); it != filtered_corners_.end(); it++) {
      index = *it;
      Eigen::Vector3i corner = corners_[index];
      level = corner(2);

      if (descriptors_[index].empty()) {
        descriptors_[index].resize(32);
        orb_detector_->GetDescriptor(pyramid_[level], Eigen::Vector2i(corner(0), corner(1)), &descriptors_[index]);
      }
    }
  }
}

int Frame::GetNumPoints() const {
  int count = 0;

  for (auto it=features_.begin(); it != features_.end(); it++) {
    if (!(*it))
      continue;

    shared_ptr<Point> f = (*it)->GetPoint();
    if (!f)
      continue;

    count++;
  }

  return count;
}

void Frame::AddConnection(const std::pair<std::shared_ptr<Frame>, int> kf) {
  connections_.push_back(kf);
}

struct SharedPointsComparator {
  bool operator()(const std::pair<shared_ptr<Frame>, int>& left, const std::pair<shared_ptr<Frame>, int>& right) {
      return left.second > right.second;
  }
};

void Frame::GetBestConnections(vector<shared_ptr<Frame>>* connections, int n) {
  int count = 0;
  connections->clear();

  if (n == 0 || n >= static_cast<int>(connections_.size())) {
    // Copy all
    for (auto it=connections_.begin(); it != connections_.end(); it++) {
      if (!it->first->ToDelete())
        connections->push_back(it->first);
    }
  } else {
    // Sort by num points shared
    sort(connections_.begin(), connections_.end(), SharedPointsComparator());
    for (auto it=connections_.begin(); it != connections_.end() && count < n; it++) {
      if (!it->first->ToDelete()) {
        connections->push_back(it->first);
        count++;
      }
    }
  }
}

void Frame::RemoveFeatures() {
  for (auto it=features_.begin(); it != features_.end(); it++) {
    *it = nullptr;
  }
  features_.clear();
}

}  // namespace sdvl
