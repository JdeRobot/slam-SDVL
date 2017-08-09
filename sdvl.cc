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

#include "./sdvl.h"
#include "./point.h"
#include "./image_align.h"
#include "./config.h"
#include "extra/bundle.h"

using std::shared_ptr;
using std::vector;
using std::cout;
using std::cerr;
using std::endl;

namespace sdvl {

SDVL::SDVL(Camera* camera) :
  camera_(camera),
  map_(),
  feature_align_(&map_, camera_, Config::MaxMatches()),
  h_init_(&map_, Config::MinAvgShift()) {
  state_ = STATE_FIRST_FRAME;
  tracking_quality_ = TRACKING_GOOD;
  last_frame_ = nullptr;
  last_kf_ = nullptr;
  current_frame_ = nullptr;
  map_created_ = false;
  lost_frames_ = 0;

  ResetMotionModel();
}

SDVL::~SDVL() {
}

bool SDVL::HandleFrame(const cv::Mat& img) {
  bool relocalize;

  // Create frame with current image
  current_frame_ = std::make_shared<Frame>(camera_, &orb_detector_, img.clone(), state_ == STATE_RUNNING);

  if (state_ == STATE_FIRST_FRAME) {
    SaveFirstFrame();
    last_frame_ = current_frame_;
    last_kf_ = current_frame_;
  } else if (state_ == STATE_SECOND_FRAME) {
    if (!SaveSecondFrame()) {
      cout << "[INFO] Initialization not valid, SDVL is lost" << endl;
      return false;
    }
    last_frame_ = current_frame_;
    last_kf_ = current_frame_;
  } else if (state_ == STATE_RUNNING) {
    relocalize = lost_frames_ >= 3;

    if (relocalize) {
      cout << "[INFO] Relocalizing" << endl;
      ResetMotionModel();

      // Stop mapping
      map_.SetRelocalizing(true);

      if (Relocalize(&last_kf_)) {
        // Restart mapping
        map_.SetRelocalizing(false);
        last_frame_ = last_kf_;
        relocalize = false;
        cout << "[INFO] Relocalized!!!" << endl;
      }
    }

    if (!relocalize) {
      SetMotionModel();
      ProcessFrame(last_frame_, last_kf_);
      GetMotionModel();

      // Check tracking quality
      CalcTrackingQuality(matches_, attempts_);

      if (tracking_quality_ != TRACKING_BAD) {
        if (tracking_quality_ == TRACKING_GOOD && map_.NeedKeyframe(current_frame_, matches_)) {
          // Link points to features, the opposite was done in FeatureAlign
          vector<shared_ptr<Feature>>& features = current_frame_->GetFeatures();
          for (auto it=features.begin(); it != features.end(); it++) {
            if ((*it)->GetPoint())
              (*it)->GetPoint()->AddFeature(*it);
          }

          // Save Keyframe
          current_frame_->SetKeyframe();
          map_.AddKeyframe(current_frame_);
          last_kf_ = current_frame_;

          // Remove furthest keyframe
          map_.LimitKeyframes(current_frame_);
        } else {
          map_.AddFrame(current_frame_);
        }

        last_frame_ = current_frame_;
      }
    }
  }

  current_frame_ = nullptr;

  // Perform clean up
  map_.EmptyTrash();

  return true;
}

bool SDVL::SaveFirstFrame() {
  cout << "[DEBUG] Process first frame" << endl;

  current_frame_->CreateCorners(Config::MaxFastLevels(), 2*Config::NumFeatures());
  current_frame_->SetPose(sdvl::SE3());
  if (!h_init_.InitFirstFrame(current_frame_)) {
    cerr << "[ERROR] First frame couldn't be initialized" << endl;
    return false;
  }

  // Add a new keyframe
  current_frame_->SetKeyframe();
  map_.AddKeyframe(current_frame_, false);

  state_ = STATE_SECOND_FRAME;
  return true;
}

bool SDVL::SaveSecondFrame() {
  cout << "[DEBUG] Process second frame" << endl;

  current_frame_->CreateCorners(Config::MaxFastLevels(), Config::NumFeatures());
  if (!h_init_.InitSecondFrame(current_frame_)) {
    if (h_init_.HasError()) {
      cerr << "[ERROR] Homography couldn't be performed" << endl;
      return false;
    }
    cerr << "[ERROR] Second frame couldn't be initialized" << endl;
    return true;
  }

  // Add a new keyframe
  current_frame_->SetKeyframe();
  map_.AddKeyframe(current_frame_, false);

  // Transform map to set z=0 plane
  map_.TransformInitialMap(current_frame_);
  map_created_ = true;

  // Perform bundle adjustment
  map_.BundleAdjustment();

  h_init_.Reset();
  state_ = STATE_RUNNING;
  return true;
}

bool SDVL::ProcessFrame(const shared_ptr<Frame> &last_frame, const shared_ptr<Frame> &last_kf) {
  vector<PointInfo> selected_fs;
  vector<Feature> inliers, outliers;

  cout << "[DEBUG] Process standard frame" << endl;

  ImageAlign image_align;
  // Align image frames
  {
    std::unique_lock<std::mutex> lock(map_.GetMutex());
    image_align.ComputePose(last_frame, current_frame_);
  }

  // Align points with new frame
  feature_align_.Reproject(current_frame_, last_frame, last_kf);
  matches_ = feature_align_.GetMatches();
  attempts_ = feature_align_.GetAttempts();

  cout << "[DEBUG] Matches: " << matches_ << ", Attempts: " << attempts_ <<  endl;

  // Optimize pose with successful matches
  feature_align_.OptimizePose(current_frame_);

  return true;
}

bool SDVL::Relocalize(shared_ptr<Frame> *last_kf) {
  vector<shared_ptr<Frame>>& kfs = map_.GetKeyframes();

  // Start from last keyframe
  for (auto it=kfs.rbegin(); it != kfs.rend(); it++) {
    shared_ptr<Frame> cframe = *it;
    current_frame_->SetPose(cframe->GetPose());

    ImageAlign image_align;
    // Align image
    {
      std::unique_lock<std::mutex> lock(map_.GetMutex());
      image_align.ComputePose(cframe, current_frame_, true);
    }

    // Max error
    if (image_align.GetError() >= 0.001)
      continue;

    // Align points with new frame
    feature_align_.Reproject(current_frame_, cframe, cframe, true);
    matches_ = feature_align_.GetMatches();
    attempts_ = feature_align_.GetAttempts();

    cout << "[DEBUG] Matches: " << matches_ << ", Attempts: " << attempts_ << endl;

    if (matches_ >= Config::MinMatches()) {
      *last_kf = cframe;
      return true;
    }
  }

  return false;
}

void SDVL::CalcTrackingQuality(int matches, int attempts) {
  double ratio;

  if (attempts == 0)
    ratio = 0.0;
  else
    ratio = static_cast<double>(matches) / static_cast<double>(attempts);

  if (ratio > 0.2) {
    tracking_quality_ = TRACKING_GOOD;
    lost_frames_ = 0;
    return;
  }

  if (matches < Config::MinMatches()) {
    tracking_quality_ = TRACKING_BAD;
    lost_frames_++;
    cerr << "[ERROR] Tracking quality is bad" << endl;
    return;
  }

  lost_frames_ = 0;
  tracking_quality_ = TRACKING_INSUFFICIENT;
  cerr << "[ERROR] Tracking quality is insufficient" << endl;
}

void SDVL::GetMotionModel() {
  double reduction = 0.9;

  // Calc current velocity
  SE3 mov = current_frame_->GetPose() * last_frame_->GetPose().Inverse();
  Eigen::Matrix<double, 6, 1> vel = SE3::Log(mov);

  // Calc final velocity with current and last velocity
  Eigen::Matrix<double, 6, 1> old_vel = vel_;
  vel_ = reduction * (0.5*vel + 0.5*old_vel);
}

void SDVL::SetMotionModel() {
  // Set initial frame position taking into account current velocity
  current_frame_->SetPose(SE3::Exp(vel_) * last_frame_->GetPose());
}

void SDVL::GetCameraTrail(vector<std::pair<sdvl::SE3, bool>> * positions) {
  std::unique_lock<std::mutex> lock(map_.GetMutex());
  positions->clear();
  for (auto it=map_.GetKeyframes().begin(); it != map_.GetKeyframes().end(); it++) {
    positions->push_back(std::make_pair((*it)->GetWorldPose(), (*it)->IsSelected()));
  }
}

void SDVL::GetPoints(vector<Eigen::Vector3d> * positions) {
  double zmin, zmax;
  Eigen::Vector3d v;
  SE3 pose;

  std::unique_lock<std::mutex> lock(map_.GetMutex());

  positions->clear();
  for (auto it=map_.GetKeyframes().begin(); it != map_.GetKeyframes().end(); it++) {
    for (auto feature=(*it)->GetFeatures().begin(); feature != (*it)->GetFeatures().end(); feature++) {
      assert(*feature != nullptr);
      shared_ptr<Point> p = (*feature)->GetPoint();
      if (!p || p->ToDelete())
        continue;

      if (p->HasConverged()) {
        // Point converged
        positions->push_back(p->GetPosition());
        positions->push_back(p->GetPosition());
      } else {
        // Point not converged
        zmin = 1.0/(p->GetInverseDepth() + 2.0*p->GetStd());
        zmax = 1.0/(std::max(p->GetInverseDepth() - 2.0*p->GetStd(), 0.00000001));
        v = p->GetInitFeature()->GetVector();
        pose = p->GetInitFeature()->GetFrame()->GetWorldPose();
        positions->push_back(pose * (p->GetInitFeature()->GetVector()*zmin));
        positions->push_back(pose * (p->GetInitFeature()->GetVector()*zmax));
      }
    }
  }
}

void SDVL::GetLastFeatures(vector<Eigen::Vector3i> * positions) {
  Eigen::Vector2d pos;

  std::unique_lock<std::mutex> lock(map_.GetMutex());

  positions->clear();

  if (last_frame_) {
    vector<shared_ptr<Feature>>& features = last_frame_->GetFeatures();
    for (auto feature=features.begin(); feature != features.end(); feature++) {
      assert(*feature != nullptr);
      shared_ptr<Point> point = (*feature)->GetPoint();
      if (!point || point->ToDelete())
        continue;
      pos = (*feature)->GetPosition();
      positions->push_back(Eigen::Vector3i(pos(0), pos(1), point->GetStatus()));
    }
    vector<Eigen::Vector2d>& outliers = last_frame_->GetOutliers();
    for (auto outlier=outliers.begin(); outlier != outliers.end(); outlier++) {  
      positions->push_back(Eigen::Vector3i((*outlier)(0), (*outlier)(1), Point::P_OUTLIER));
    }
  }
}

sdvl::SE3 SDVL::GetPose() const {
  if (last_frame_)
    return last_frame_->GetWorldPose();
  return sdvl::SE3();
}

}  // namespace sdvl
