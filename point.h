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

#ifndef SDVL_POINT_H_
#define SDVL_POINT_H_

#include <iostream>
#include <list>
#include <memory>
#include "./frame.h"
#include "./feature.h"
#include "extra/se3.h"

namespace sdvl {

class Frame;
class Feature;

class Point {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum PointStatus {
    P_FOUND,
    P_NOT_FOUND,
    P_SEEN,
    P_UNSEEN,
    P_OUTLIER
  };

  Point();
  ~Point();

  inline double GetInverseDepth() { return rho_; }
  inline double GetStd() { return sqrt(sigma2_); }

  inline std::shared_ptr<Feature> GetInitFeature() { return feature_; }
  inline void SetInitFeature(const std::shared_ptr<Feature> &feature) { feature_ = feature; }

  inline int GetID() const { return id_; }

  // Calculate current 3D position
  Eigen::Vector3d GetPosition() const;
  void SetPosition(const Eigen::Vector3d &pos);

  inline std::list<std::shared_ptr<Feature>>& GetFeatures() {
    return features_;
  }

  inline int Score() const { return n_successful_; }

  inline int GetLastFrame() const { return last_frame_; }
  inline void SetLastFrame(int id) { last_frame_ = id; }

  inline int GetLastBA() const { return last_ba_; }
  inline void SetLastBA(int id) { last_ba_ = id; }

  inline PointStatus GetStatus() const { return status_; }
  inline void SetStatus(PointStatus s) { status_ = s; }

  inline bool ToDelete() const { return delete_; }
  inline void SetDelete() { delete_ = true; }

  inline void SetFixed() { fixed_ = true; }
  inline bool IsFixed() { return fixed_; }

  // Add feature from frame where point was detected
  inline void AddFeature(const std::shared_ptr<Feature> &feature) {
    features_.push_front(feature);
  }

  // Get feature in last Keyframe
  inline std::shared_ptr<Feature> GetLastFeature() {
    return features_.front();
  }

  // Init candidate with feature
  void InitCandidate(const std::shared_ptr<Feature> &p, double depth);

  // Update candidate with new information
  void Update(const std::shared_ptr<Frame> &frame, double depth, double px_error_angle);

  // Check if type can change to promote/unpromote point
  bool Promote();
  bool Unpromote();

  // Return true if point has converged
  bool HasConverged();

  // Check if point is seen from a Keyframe
  bool SeenFrom(const std::shared_ptr<Frame> &frame) const;

  // Delete feature from list
  void DeleteFeature(const std::shared_ptr<Feature> &feature);

 protected:
  // Compute tau
  double ComputeTau(const SE3 &pose, const Eigen::Vector3d &v, double depth, double px_error_angle);

  // Probability density function (PDF) for normal distribution
  double PDFNormal(double mean, double sd, double x);

  int id_;                      // Point unique id
  PointStatus status_;          // Point status in last frame
  bool delete_;                 // True if it will be deleted

  std::list<std::shared_ptr<Feature>> features_;     // Store all matches of this point

  int last_frame_;        // Last frame where it was projected, avoid projecting it twice
  int last_ba_;           // Last BA where it was used, avoid using it twice
  int n_successful_;      // Number of successful projections
  int n_failed_;          // Number of failed projections
  bool map_saved_;        // Return true if points was saved in map

  static int counter_;    // Counter to set point id

  // Depth uncertainty
  double a_;                // Beta distribution: When high, probability of inlier is large.
  double b_;                // Beta distribution: When high, probability of outlier is large.
  double rho_;              // Inverse depth.
  double sigma2_;           // Variance of inverse depth.
  double z_range_;          // Max range of the possible depth.

  std::shared_ptr<Feature> feature_;      // Feature where point was created
  double cos_alpha_;                      // Parallax in last update
  double last_distance_;                  // Distance to last frame
  bool fixed_;                            // True if point has a fixed position
  Eigen::Vector3d p3d_;                   // 3D position (only if converged)
};

}  // namespace sdvl

#endif  // SDVL_POINT_H_
