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

#ifndef SDVL_FEATURE_ALIGN_H_
#define SDVL_FEATURE_ALIGN_H_

#include <iostream>
#include <vector>
#include <list>
#include <utility>
#include <memory>
#include "./frame.h"
#include "./map.h"

namespace sdvl {

class Map;
class Point;

// Store info for each grid cell
typedef std::pair<std::shared_ptr<Point>, Eigen::Vector2d> PointInfo;
typedef std::list<PointInfo> GridCell;

class FeatureAlign {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FeatureAlign(Map *map, Camera *camera, int max_matches);
  ~FeatureAlign();

  // Reproject all points in current frame found in other frames
  void Reproject(const std::shared_ptr<Frame> &frame, const std::shared_ptr<Frame> &last_frame,
                 const std::shared_ptr<Frame> &last_kf, bool reloc = false);

  // Minimize reprojection error of a single frame.
  bool OptimizePose(const std::shared_ptr<Frame> &frame);

  inline int GetMatches() { return matches_; }
  inline int GetAttempts() { return num_attempts_; }

 private:
  // Select points to reproject. Return selected points.
  void SelectPoints(const std::shared_ptr<Frame> &frame, const std::shared_ptr<Frame> &last_frame,
                      const std::shared_ptr<Frame> &last_kf, std::vector<std::shared_ptr<Feature>> *fs_found);

  // Rule out outiliers using RANSAC
  void SelectInliers(const std::shared_ptr<Frame> &frame, std::vector<std::shared_ptr<Feature>> &fs_found,
                    std::vector<std::shared_ptr<Feature>> *inliers, std::vector<std::shared_ptr<Feature>> *outliers);

  // Minimize reprojection error. Return observations
  void OptimizePose(const std::shared_ptr<Frame> &frame, std::vector<std::shared_ptr<Feature>> *features,
                    std::vector<std::shared_ptr<Feature>> *outliers);

  // Rescue high innovation outliers
  bool RescueOutliers(const std::shared_ptr<Frame> &frame, std::vector<std::shared_ptr<Feature>> *inliers,
                    std::vector<std::shared_ptr<Feature>> *outliers);

  // Remove outliers
  void RemoveOutliers(const std::shared_ptr<Frame> &frame, std::vector<std::shared_ptr<Feature>> *outliers);

  // Check reprojection error. Return number of inliers
  int CheckReprojectionError(const std::vector<std::shared_ptr<Feature>> features, const SE3 &se3, double threshold,
                             std::vector<std::shared_ptr<Feature>> *inliers = NULL,
                             std::vector<std::shared_ptr<Feature>> *outliers = NULL);

  // Reset grid
  void ResetGrid();

  // Project points into frame
  void ProjectPoints(const std::shared_ptr<Frame> &frame, const std::shared_ptr<Frame> &last_frame);
  bool ProjectPoint(const std::shared_ptr<Frame> &frame, const std::shared_ptr<Point> &point);

  // Converge frame pose to minimize reprojection error
  bool ConvergePose(const std::shared_ptr<Frame> &frame, const std::vector<std::shared_ptr<Feature>> &features, SE3 *se3);

  // Tukey's hard re-descending function
  double GetTukeyValue(double x);

  Map * map_;                     // Stored map
  int cell_size_;                 // Grid cell size
  int max_matches_;               // Max matches detected
  int grid_width_;                // Grid width
  int grid_height_;               // Grid height
  std::vector<GridCell*> grid_;   // Image grid
  Eigen::MatrixXi grid_used_;     // True if grid is occupied
  std::vector<int> cell_order_;   // Randomly access to each grid cell

  std::vector<std::shared_ptr<Frame>> fov_kfs_;   // Store all keyframes from current frame of view
  int matches_;                                   // Number of matches found
  int num_attempts_;                              // Number of attempted matches
  bool relocalizing_;                             // True if algorithm is relocalizing

  std::vector<std::shared_ptr<Feature>> inliers_;     // Store inliers
  std::vector<std::shared_ptr<Feature>> outliers_;    // Store outliers

  static constexpr double KMADNorm = 1.4826;          // See: https://en.wikipedia.org/wiki/Median_absolute_deviation
  static constexpr double KTukeyC = 4.6851*4.6851;    // See: http://en.wikipedia.org/wiki/Redescending_M-estimator
};

}  // namespace sdvl

#endif  // SDVL_FEATURE_ALIGN_H_
