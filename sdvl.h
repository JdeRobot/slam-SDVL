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

#ifndef SDVL_SDVL_H_
#define SDVL_SDVL_H_

#include <set>
#include <memory>
#include <vector>
#include <utility>
#include "./map.h"
#include "./feature_align.h"
#include "./homography_init.h"
#include "extra/se3.h"

namespace sdvl {

class SDVL {
 public:
  enum State {
    STATE_FIRST_FRAME,
    STATE_SECOND_FRAME,
    STATE_RUNNING,
  };
  enum TrackingQuality {
    TRACKING_GOOD,
    TRACKING_INSUFFICIENT,
    TRACKING_BAD
  };

  explicit SDVL(Camera* camera);
  ~SDVL();

  void GetCameraTrail(std::vector<std::pair<sdvl::SE3, bool>> * positions);
  void GetPoints(std::vector<Eigen::Vector3d> * positions);
  void GetLastFeatures(std::vector<Eigen::Vector3i> * positions);
  inline TrackingQuality GetTrackingQuality() const { return tracking_quality_; }

  // Start/Stop SDVL algorithm threads
  void Start() { map_.Start(); }
  void Stop() { map_.Stop(); }
  void Mapping() { map_.UpdateMap(); }

  // Provide an image.
  bool HandleFrame(const cv::Mat& img);

  // Get current position and orientation
  sdvl::SE3 GetPose() const;

  // Return true if map has already been created
  inline bool HasMap() { return map_created_; }

 private:
  // Save first and second frame. Return false if something was wrong
  bool SaveFirstFrame();
  bool SaveSecondFrame();

  // Process frame after initialization
  bool ProcessFrame(const std::shared_ptr<Frame> &last_frame, const std::shared_ptr<Frame> &last_kf);

  // Search a valid frame inside keyframes
  bool Relocalize(std::shared_ptr<Frame> *last_kf);

  // Calc motion model
  void GetMotionModel();
  void SetMotionModel();
  inline void ResetMotionModel() { vel_.setZero(); }

  // Calculate tracking quality based on matches
  void CalcTrackingQuality(int matches, int attempts);

  Camera * camera_;
  Map map_;             // Map of keyframes and points
  ORBDetector orb_detector_;

  State state_;
  std::shared_ptr<Frame> current_frame_;    // Store current keyframe
  std::shared_ptr<Frame> last_frame_;       // Store last frame
  std::shared_ptr<Frame> last_kf_;          // Store last keyframe
  TrackingQuality tracking_quality_;        // Tracking quality
  Eigen::Matrix<double, 6, 1> vel_;           // Velocity between iterations

  FeatureAlign feature_align_;    // Align points from other keyframes into the current frame
  HomographyInit h_init_;         // Compute homography with initial keyframes
  bool map_created_;              // True if map was created
  int lost_frames_;               // Lost frames

  int matches_;                 // Matches performed in last iteration
  int attempts_;                // Matches attempts in last iteration
};

}  // namespace sdvl

#endif  // SDVL_SDVL_H_
