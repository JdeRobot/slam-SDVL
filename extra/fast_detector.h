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

class FastDetector {
 public:
  FastDetector(int width, int height, bool grid=true);

  // Detect fast corners in an image pyramid. Return corners with their pyramid level
  void DetectPyramid(const std::vector<cv::Mat> &pyramid, std::vector<Eigen::Vector3i> *corners, int nfeatures);

  // Filter corners choosing the best corner for each image grid cell
  void FilterCorners(const std::vector<cv::Mat> &pyramid, const std::vector<Eigen::Vector3i> &corners, std::vector<int> *indices);

  // Lock/Unlock a grid cell
  void LockCell(Eigen::Vector2d p);
  void UnlockCell(Eigen::Vector2d p);

 private:
  // Detect N fast corners in given image
  void SelectPixels(const cv::Mat &src, std::vector<Eigen::Vector3i> *pixels, int level, int nfeatures);

  // Init new grid
  void InitGrid(int width, int height);

  // Reset grid mask
  void ResetGrid();

  std::vector<std::pair<int, int>> cgrid_;      // Grid with corners detected: Index and score
  std::vector<bool> grid_mask_;                 // Mask with locked grid cells
  int grid_width_;                              // Grid width
  int grid_height_;                             // Grid height
  int cell_size_;                               // Cell size
};

}  // namespace sdvl

#endif  // SDVL_EXTRA_FAST_DETECTOR_H_
