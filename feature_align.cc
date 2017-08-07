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

#include "./feature_align.h"
#include "./config.h"
#include "extra/utils.h"

using std::shared_ptr;
using std::vector;
using std::cout;
using std::endl;

namespace sdvl {

FeatureAlign::FeatureAlign(Map *map, Camera *camera, int max_matches) {
  map_ = map;
  cell_size_ = Config::CellSize();
  max_matches_ = max_matches;
  matches_ = 0;
  num_attempts_ = 0;
  relocalizing_ = false;

  // Init grid
  grid_width_ = ceil(static_cast<double>(camera->GetWidth())/cell_size_);
  grid_height_ = ceil(static_cast<double>(camera->GetHeight())/cell_size_);

  int size = grid_width_*grid_height_;
  grid_.resize(size);
  for_each(grid_.begin(), grid_.end(), [&](GridCell*& c){ c = new GridCell; });
  grid_used_ = Eigen::MatrixXi::Zero(grid_height_, grid_width_);

  // Set random cell order
  for (int i=0; i < size; ++i)
    cell_order_.push_back(i);
  random_shuffle(cell_order_.begin(), cell_order_.end());
}

FeatureAlign::~FeatureAlign() {
}

void FeatureAlign::Reproject(const shared_ptr<Frame> &frame, const shared_ptr<Frame> &last_frame, const shared_ptr<Frame> &last_kf, bool reloc) {
  vector<shared_ptr<Feature>> selected_fs;

  inliers_.clear();
  outliers_.clear();
  relocalizing_ = reloc;

  // Select inliers
  SelectPoints(frame, last_frame, last_kf, &selected_fs);
  SelectInliers(frame, selected_fs, &inliers_, &outliers_);
  cout << "[DEBUG] Selected points: " << selected_fs.size() << endl;
  cout << "[DEBUG] Inliers: " << inliers_.size() << " outliers " << outliers_.size() << endl;
}

bool FeatureAlign::OptimizePose(const shared_ptr<Frame> &frame) {
  OptimizePose(frame, &inliers_, &outliers_);
  // Rescue high innovation outliers
  if (RescueOutliers(frame, &inliers_, &outliers_)) {
    OptimizePose(frame, &inliers_, &outliers_);
    cout << "[DEBUG] Rescued inliers: " << inliers_.size() << " outliers: " << outliers_.size() << endl;
  }
  RemoveOutliers(frame, &outliers_);
  return true;
}

bool CompareQuality(const PointInfo &a, const PointInfo &b) {
  return (a.first->Score() > b.first->Score());
}

void FeatureAlign::SelectPoints(const shared_ptr<Frame> &frame, const shared_ptr<Frame> &last_frame,
                                 const shared_ptr<Frame> &last_kf, vector<shared_ptr<Feature>> *fs_found) {

  Matcher matcher(Config::PatchSize());
  Eigen::Vector2d pos;
  bool found;
  int level;

  // Project points in grid cells
  ProjectPoints(frame, last_frame);

  matches_ = 0;
  num_attempts_ = 0;

  // In each cell, project only one point
  random_shuffle(cell_order_.begin(), cell_order_.end());
  int size = grid_.size();
  for (int i=0; i < size && matches_ < max_matches_; i++) {
    found = false;

    // Sort points according to their quality
    GridCell * cell = grid_.at(cell_order_[i]);
    cell->sort(CompareQuality);

    // Try to match at least one point per cell
    for (auto it=cell->begin(); it != cell->end() && !found; it++) {
      shared_ptr<Point> point = it->first;
      if (point->ToDelete())
        continue;

      shared_ptr<Feature> feature = point->GetInitFeature();
      if (!feature)
        continue;

      num_attempts_++;
      pos = it->second;

      // Search point in current frame
      found = matcher.SearchPoint(frame, feature, point->GetInverseDepth(), point->GetStd(), point->IsFixed(), &pos, &level);
      if (found) {
        if (!relocalizing_) {
          point->Promote();

          // Link feature to point, the opposite will be done only if this frame is set as keyframe
          shared_ptr<Feature> feature = std::make_shared<Feature>(frame, pos, level);
          feature->SetPoint(point);
          frame->AddFeature(feature);

          point->SetStatus(Point::P_FOUND);
          fs_found->push_back(feature);
        }
        matches_++;
      } else {
        if (!relocalizing_) {
          // If feature is unpromoted, delete it
          if (point->Unpromote())
            map_->DeletePoint(point);
          point->SetStatus(Point::P_NOT_FOUND);
        }
      }
    }
  }
}

void FeatureAlign::SelectInliers(const shared_ptr<Frame> &frame, vector<shared_ptr<Feature>> &fs_found,
                                vector<shared_ptr<Feature>> *inliers, vector<shared_ptr<Feature>> *outliers) {
  Matcher matcher(Config::PatchSize());
  Eigen::Vector2d pos;
  int index, size, npoints;
  vector<shared_ptr<Feature>> selected, best_fs;
  int supporters, best_supporters;
  SE3 se3, best_se3;

  inliers->clear();
  outliers->clear();

  if (fs_found.empty())
    return;

  size = fs_found.size();
  npoints = std::min(Config::MaxRansacPoints(), size);
  int it, nits, indexes[npoints];
  double sprob = 0.99;

  // Search inliers and outliers with RANSAC
  nits = Config::MaxRansacIts();
  best_supporters = 0;
  it = 0;
  while (it < nits) {
    selected.clear();

    // Select N points
    index = rand()%size;
    for (int i=0; i < npoints; i++) {
      indexes[i] = (index+i)%size;
      selected.push_back(fs_found.at(indexes[i]));
    }

    // Converge with selected points
    if (!ConvergePose(frame, selected, &se3)) {
      it++;
      continue;
    }

    // Get supporters
    supporters = CheckReprojectionError(fs_found, se3, Config::InlierErrorThreshold()/frame->GetCamera()->GetFx());

    // Save best iteration
    if (supporters > best_supporters) {
      best_fs = selected;
      best_supporters = supporters;
      best_se3 = se3;

      // Update max iterations
      double epsilon = 1.0 - (static_cast<double>(supporters) / static_cast<double>(size));
      double tmp = 1.0 - epsilon;
      for (int k=1; k < npoints; k++)
        tmp *= tmp;
      if (tmp < 1e-5)
        nits = Config::MaxRansacIts();
      else
        nits = std::min(Config::MaxRansacIts(), static_cast<int>(log(1.0 - sprob) / log(1.0 - tmp)));
    }
    it++;
  }

  // Get low innovation inliers
  CheckReprojectionError(fs_found, best_se3, Config::InlierErrorThreshold()/frame->GetCamera()->GetFx(), inliers, outliers);
}

void FeatureAlign::OptimizePose(const shared_ptr<Frame> &frame, vector<shared_ptr<Feature>> *features,
                               vector<shared_ptr<Feature>> *outliers) {
  // Converge frame pose
  SE3 se3 = frame->GetPose();
  if (!ConvergePose(frame, *features, &se3))
    return;
  frame->SetPose(se3);

  // Recalculate inliers and outliers
  vector<shared_ptr<Feature>> cfeatures = *features;
  features->clear();
  CheckReprojectionError(cfeatures, frame->GetPose(), Config::InlierErrorThreshold()/frame->GetCamera()->GetFx(), features, outliers);
}

bool FeatureAlign::RescueOutliers(const shared_ptr<Frame> &frame, vector<shared_ptr<Feature>> *inliers,
                                 vector<shared_ptr<Feature>> *outliers) {
  int ninliers, init_inliers = inliers->size();
  vector<shared_ptr<Feature>> cfeatures = *outliers;
  outliers->clear();

  // Get inliers with high innovation
  CheckReprojectionError(cfeatures, frame->GetPose(), 2*Config::InlierErrorThreshold()/frame->GetCamera()->GetFx(), inliers, outliers);

  ninliers = inliers->size();
  return ninliers > init_inliers;
}

void FeatureAlign::RemoveOutliers(const shared_ptr<Frame> &frame, vector<shared_ptr<Feature>> *outliers) {
  for (auto it=outliers->begin(); it != outliers->end(); it++) {
    shared_ptr<Point> p = (*it)->GetPoint();
    if (!p)
      continue;

    // Point to feature reference was not yet created, so we don't need to delete it
    (*it)->SetPoint(nullptr);
    p->SetStatus(Point::P_NOT_FOUND);
    frame->AddOutlier((*it)->GetPosition());
  }
}

int FeatureAlign::CheckReprojectionError(const vector<shared_ptr<Feature>> features, const SE3 &se3, double threshold,
                                        vector<shared_ptr<Feature>> *inliers, vector<shared_ptr<Feature>> *outliers) {
  Eigen::Vector2d error;
  Eigen::Vector3d pos;
  int valids = 0;

  for (auto it=features.begin(); it != features.end(); it++) {
    shared_ptr<Point> point = (*it)->GetPoint();
    if (!point)
      continue;

    pos = se3*point->GetPosition();
    error = Camera::SimpleProject((*it)->GetVector()) - Camera::SimpleProject(pos);
    double sqrt_inv_cov = 1.0 / (1 << (*it)->GetLevel());
    error *= sqrt_inv_cov;
    if (error.norm() <= threshold) {
      valids++;
      if (inliers != NULL)
        inliers->push_back(*it);
    } else {
      if (outliers != NULL)
        outliers->push_back(*it);
    }
  }
  return valids;
}

void FeatureAlign::ResetGrid() {
  matches_ = 0;
  num_attempts_ = 0;
  for_each(grid_.begin(), grid_.end(), [&](GridCell* c) {
    for (auto it=c->begin(); it != c->end(); it++)
      (*it).first = nullptr;
    c->clear();
  });
  grid_used_.setZero();
}

void FeatureAlign::ProjectPoints(const shared_ptr<Frame> &frame, const shared_ptr<Frame> &last_frame) {
  ResetGrid();

  vector<shared_ptr<Feature>>& features = last_frame->GetFeatures();

  // Project points and calculate their corresponding grid cells
  for (auto it_fts=features.begin(); it_fts != features.end(); it_fts++) {
    if (*it_fts == nullptr)
      continue;

    shared_ptr<Point> point = (*it_fts)->GetPoint();
    if (!point || point->ToDelete())
      continue;

    // Check if point has already been projected in this frame
    if (frame->GetID() == point->GetLastFrame())
      continue;

    ProjectPoint(frame, point);
    if (!relocalizing_)
      point->SetLastFrame(frame->GetID());
  }
}

bool FeatureAlign::ProjectPoint(const shared_ptr<Frame> &frame, const shared_ptr<Point> &point) {
  Eigen::Vector2d p;
  int k;

  if (!frame->Project(point->GetPosition(), &p)) {
    point->SetStatus(Point::P_UNSEEN);
    return false;
  }

  if (!frame->GetCamera()->IsInsideImage(p.cast<int>(), Config::PatchSize())) {
    point->SetStatus(Point::P_UNSEEN);
    return false;
  }

  k = static_cast<int>(p(1)/cell_size_)*grid_width_+ static_cast<int>(p(0)/cell_size_);
  grid_.at(k)->push_back(std::make_pair(point, p));
  grid_used_(static_cast<int>(p(1)/cell_size_), static_cast<int>(p(0)/cell_size_)) = 1;
  point->SetStatus(Point::P_SEEN);
  return true;
}

bool FeatureAlign::ConvergePose(const std::shared_ptr<Frame> &frame, const std::vector<std::shared_ptr<Feature>> &features, SE3 *se3) {
  Eigen::Vector2d error;
  Eigen::Vector3d pos;
  Eigen::Matrix<double, 6, 6> A;
  Eigen::Matrix<double, 6, 1> b;
  Eigen::Matrix<double, 2, 6> J;
  Camera * camera;
  SE3 last_se3;
  double scale, chi2, new_chi2, weight;

  last_se3 = frame->GetPose();
  camera = frame->GetCamera();
  *se3 = last_se3;
  chi2 = 0.0;

  // Calculate scaled error
  vector<double> errors;
  for (auto it=features.begin(); it != features.end(); it++) {
    shared_ptr<Point> point = (*it)->GetPoint();
    if (!point)
      continue;
    pos = (*se3)*point->GetPosition();
    error = Camera::SimpleProject((*it)->GetVector()) - Camera::SimpleProject(pos);
    error *= 1.0 / (1 << (*it)->GetLevel());
    errors.push_back(error.norm());
  }
  if (errors.empty())
    return false;

  // Median absolute derivation
  scale = KMADNorm * GetMedianVector(&errors);

  // Try to converge position with features
  for (int i=0; i < Config::MaxOptimPoseIts(); i++) {
    b.setZero();
    A.setZero();
    new_chi2 = 0.0;

    // Force estimator after 5th iteration
    if (i == 5)
      scale = 0.85/camera->GetFx();

    // Compute residual
    for (auto it=features.begin(); it != features.end(); it++) {
      shared_ptr<Point> point = (*it)->GetPoint();
      if (!point)
        continue;

      pos = (*se3)*point->GetPosition();
      Jacobian3DToPlane(pos, &J);
      error = Camera::SimpleProject((*it)->GetVector()) - Camera::SimpleProject(pos);
      double sqrt_inv_cov = 1.0 / (1 << (*it)->GetLevel());
      error *= sqrt_inv_cov;
      J *= sqrt_inv_cov;
      weight = GetTukeyValue(error.norm()/scale);
      A.noalias() += J.transpose()*J*weight;
      b.noalias() -= J.transpose()*error*weight;
      new_chi2 += error.squaredNorm()*weight;
    }

    // Solve linear system
    const Eigen::Matrix<double, 6, 1> dT(A.ldlt().solve(b));

    // Check if error increased
    if ((i > 0 && new_chi2 > chi2) || static_cast<bool>(std::isnan(static_cast<double>(dT[0])))) {
      *se3 = last_se3;  // roll-back
      break;
    }

    // Update model
    SE3 T_new = SE3::Exp(dT)*(*se3);
    last_se3 = *se3;
    *se3 = T_new;
    chi2 = new_chi2;

    // Stop when converged
    if (AbsMax(dT) <= 1e-10)
      break;
  }
  return true;
}

double FeatureAlign::GetTukeyValue(double x) {
  const double x_square = x * x;
  if (x_square <= KTukeyC) {
    const double tmp = 1.0 - x_square / KTukeyC;
    return tmp * tmp;
  } else {
    return 0.0;
  }
}

}  // namespace sdvl
