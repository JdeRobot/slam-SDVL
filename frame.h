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

#ifndef SDVL_FRAME_H_
#define SDVL_FRAME_H_

#include <iostream>
#include <vector>
#include <utility>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include "./feature.h"
#include "./camera.h"
#include "extra/orb_detector.h"
#include "extra/se3.h"

namespace sdvl {

class Feature;

class Frame {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Frame(Camera* camera, ORBDetector * detector, const cv::Mat& img, bool corners);
  ~Frame();

  bool IsKeyframe() { return is_keyframe_; }
  void SetKeyframe();

  // Filter corners (required with Keyframes)
  void FilterCorners();

  inline SE3& GetPose() { return pose_; }
  inline const SE3& GetPose() const { return pose_; }
  void SetPose(const SE3 &se3) {  pose_ = se3; }

  inline std::vector<cv::Mat>& GetPyramid() { return pyramid_; }
  inline std::vector<std::shared_ptr<Feature>>& GetFeatures() { return features_; }
  inline std::vector<Eigen::Vector3i>& GetCorners() { return corners_; }
  inline std::vector<int>& GetFilteredCorners() { return filtered_corners_; }
  inline std::vector<Eigen::Vector2d>& GetOutliers() { return outliers_; }

  inline std::vector<std::vector<uchar>>& GetDescriptors() { return descriptors_; }

  inline Camera * GetCamera() const { return camera_; }
  inline int GetWidth() const { return width_; }
  inline int GetHeight() const { return height_; }
  inline int GetID() const { return id_; }

  inline void SetKeyframeID(int id) { kf_id_ = id; }
  inline int GetKeyframeID() const { return kf_id_; }

  inline bool IsSelected() { return selected_; }
  inline void SetSelected(bool v) { selected_ = v; }

  inline int GetLastBA() const { return last_ba_; }
  inline void SetLastBA(int id) { last_ba_ = id; }

  inline bool ToDelete() const { return delete_; }
  inline void SetDelete() { delete_ = true; }

  // Return pose in world coordinate
  inline SE3 GetWorldPose() const { return pose_.Inverse(); }

  // Return 3d position in world coordinate.
  inline Eigen::Vector3d GetWorldPosition() const { return pose_.Inverse().GetTranslation(); }

  // Transform absolute coordinates to frame relative coordinates
  inline Eigen::Vector3d GetRelativePos(const Eigen::Vector3d &pos) const { return pose_ * pos; }

  // Angular distance between two frames
  inline double GetAngularDistance(const Frame &frame) {
    return pose_.GetQuaternion().angularDistance(frame.pose_.GetQuaternion());
  }

  // Add a feature to frame
  inline void AddFeature(const std::shared_ptr<Feature> &feature) {
    features_.push_back(feature);
  }

  // Add outlier to frame
  inline void AddOutlier(const Eigen::Vector2d &pos) {
    outliers_.push_back(pos);
  }

  // Return number of features
  inline int GetNumFeatures() const { return features_.size(); }

  // Get num points seen
  int GetNumPoints() const;

  // Add new connection
  void AddConnection(const std::pair<std::shared_ptr<Frame>, int> kf);

  // Get best connections
  void GetBestConnections(std::vector<std::shared_ptr<Frame>>* connections, int n);

  // Calculate average depth from points
  double GetSceneDepth();

  // Project 3d point into image frame
  bool Project(const Eigen::Vector3d &p3D, Eigen::Vector2d *p2D);

  // Return true if a point in visible from frame
  bool IsPointVisible(const Eigen::Vector3d &p);

  // Distance from current frame to another frame
  inline double DistanceTo(const Frame &frame) const {
    return (GetWorldPosition() - frame.GetWorldPosition()).norm();
  }

  // Distance from current frame to a point
  inline double DistanceTo(const Eigen::Vector3d &p) const {
    return (GetWorldPosition() - p).norm();
  }

  // Create corners
  void CreateCorners(int levels, int nfeatures);

  // Remove features
  void RemoveFeatures();

 private:
  // Create image pyramid
  void CreatePyramid(const cv::Mat& img);

  int id_;                      // Frame unique id
  int kf_id_;                   // Keyframe unique id
  Camera * camera_;             // Camera model
  ORBDetector * orb_detector_;  // ORB detector
  int pyramid_levels_;          // Pyramid levels calculated

  bool is_keyframe_;                // Frame selected as Keyframe
  std::vector<cv::Mat> pyramid_;    // Image pyramid
  int width_;                       // Image width in first pyramid level
  int height_;                      // Image width in first pyramid level
  SE3 pose_;                        // Frame to world pose
  bool delete_;                     // True if it will be deleted

  std::vector<std::shared_ptr<Feature>> features_;    // Features detected in this frame
  std::vector<Eigen::Vector3i> corners_;              // Corners detected. Third parameter to store level
  std::vector<int> filtered_corners_;                 // Indices of corners filtered
  std::vector<Eigen::Vector2d> outliers_;             // Outliers, only for DEBUG

  std::vector<std::vector<uchar>> descriptors_;           // ORB descriptor for detected corners

  std::vector<std::pair<std::shared_ptr<Frame>, int>> connections_;     // Connected keyframes

  static int counter_;    // Counter to set frame id
  bool selected_;         // True if frame is selected to initialize
  int last_ba_;           // Last BA where it was used, avoid using it twice
};

}  // namespace sdvl

#endif  // SDVL_FRAME_H_
