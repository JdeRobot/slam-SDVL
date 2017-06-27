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

#ifndef SDVL_EXTRA_FAST_DETECTOR_H_
#define SDVL_EXTRA_FAST_DETECTOR_H_

#include <opencv/cvaux.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <Eigen/Dense>

namespace sdvl {

// Store detected corner information
struct CandidateCorner {
  int x;          // X image coordinate
  int y;          // Y image coordinate
  int level;      // Pyramid level where it was detected
  double score;   // Corner score
  explicit CandidateCorner(double s) : x(0), y(0), level(0), score(s) {}
  CandidateCorner(int x, int y, int l, double s) : x(x), y(y), level(l), score(s) {}
};

class FastDetector {
 public:
  explicit FastDetector(int size = 0);
  FastDetector(int width, int height, int size);

  // Detect fast corners with non maximum suppresion.
  void Detect(const cv::Mat &src, std::vector<Eigen::Vector2i> *corners);
  void DetectCV(const cv::Mat &src, std::vector<Eigen::Vector2i> *corners, bool nms);

  // Detect fast corners in an image pyramid. Return corners with their pyramid level
  void DetectPyramid(const std::vector<cv::Mat> &pyramid, std::vector<std::vector<Eigen::Vector2i>> *corners);

  // Filter corners choosing the best corner for each image grid cell
  void FilterCorners(const std::vector<cv::Mat> &pyramid, const std::vector<std::vector<Eigen::Vector2i>> &corners,
                     std::vector<Eigen::Vector3i> *fcorners);

  // Init new grid
  void InitGrid(int width, int height);

  // Reset grid mask
  void ResetGrid();

  // Lock/Unlock a grid cell
  void LockCell(Eigen::Vector2d p);
  void UnlockCell(Eigen::Vector2d p);

 private:
  // Calculate fast corners
  void FastCornerDetector_10(const cv::Mat &src, std::vector<Eigen::Vector2i> *corners, int threshold);

  // Compute score for each corner
  void ComputeFastScore(const cv::Mat &src, const std::vector<Eigen::Vector2i> &corners, int threshold, std::vector<int> *scores);
  int CornerScore(const cv::Mat &src, const Eigen::Vector2i &c, const int *pointer_dir, int threshold);

  // Get nonmax suppression features
  void NonMaxSuppression(const cv::Mat &src, const std::vector<Eigen::Vector2i> &corners_in, std::vector<Eigen::Vector2i> *corners_out);
  void NonMaxSuppression(const std::vector<Eigen::Vector2i> &corners, const std::vector<int> &scores, std::vector<Eigen::Vector2i> *nonmax_corners);

  std::vector<CandidateCorner> corners_grid_;   // Grid with corners detected
  std::vector<bool> grid_mask_;                 // Mask with locked grid cells
  int grid_width_;                              // Grid width
  int grid_height_;                             // Grid height
  int cell_size_;                               // Cell size
};

}  // namespace sdvl

#endif  // SDVL_EXTRA_FAST_DETECTOR_H_
