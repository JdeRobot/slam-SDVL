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

#include "./fast_detector.h"
#include "./utils.h"
#include "../config.h"

using std::vector;

namespace sdvl {

FastDetector::FastDetector(int width, int height, bool grid) {
  cell_size_ = Config::CellSize();
  grid_width_  = ceil(static_cast<double>(width)/cell_size_);
  grid_height_ = ceil(static_cast<double>(height)/cell_size_);
  if (grid)
    InitGrid(width, height);
}

void FastDetector::InitGrid(int width, int height) {
  corners_grid_.resize(grid_width_*grid_height_, CandidateCorner(Config::MinFeatureScore()));
  grid_mask_.resize(grid_width_*grid_height_, false);
}

void FastDetector::ResetGrid() {
  fill(corners_grid_.begin(), corners_grid_.end(), CandidateCorner(Config::MinFeatureScore()));
  fill(grid_mask_.begin(), grid_mask_.end(), false);
}

void FastDetector::LockCell(Eigen::Vector2d p) {
  int index = static_cast<int>(p(1)/cell_size_)*grid_width_ + static_cast<int>(p(0)/cell_size_);
  grid_mask_.at(index) = true;
}

void FastDetector::UnlockCell(Eigen::Vector2d p) {
  int index = static_cast<int>(p(1)/cell_size_)*grid_width_ + static_cast<int>(p(0)/cell_size_);
  grid_mask_.at(index) = false;
}

void FastDetector::SelectPixels(const cv::Mat &src, std::vector<Eigen::Vector2i> *pixels, int nfeatures) {
  int initx, inity, maxx, maxy;
  int margin;
  int nempty;

  if (Config::UseORB())
    margin = 1+Config::ORBSize()/2;
  else
    margin = 1+Config::PatchSize()/2;

  vector<cv::KeyPoint> fts;

  int ncols = src.cols;
  int nrows = src.rows;
  int wcells = ceil((double)ncols/(double)cell_size_);
  int hcells = ceil((double)nrows/(double)cell_size_);

  vector<vector<vector<cv::KeyPoint>>> cell_fts(hcells, vector<vector<cv::KeyPoint>>(wcells));
  vector<vector<int>> nleft(hcells, vector<int>(wcells, 0));
  vector<vector<int>> nselected(hcells, vector<int>(wcells, 0));

  nempty = 0;
  for(int i=0; i<hcells; i++) {
    inity = std::max(margin, i*cell_size_);
    maxy = std::min(src.rows-margin, i*cell_size_ + cell_size_);

    if (maxy <= inity)
      continue;

    for(int j=0; j<wcells; j++) {
      initx = std::max(margin, j*cell_size_);
      maxx = std::min(src.cols-margin, j*cell_size_ + cell_size_);

      if (maxx <= initx)
        continue;

      // Get fast points in image region
      FAST(src.rowRange(inity,maxy).colRange(initx,maxx), cell_fts[i][j], Config::FastThreshold(), true);

      if (!cell_fts[i][j].empty()) {
        for (auto vit=cell_fts[i][j].begin(); vit!=cell_fts[i][j].end();vit++) {
          (*vit).pt.x += initx;
          (*vit).pt.y += inity;
        }
        nleft[i][j] = cell_fts[i][j].size();
      } else
        nempty++;
    }
  }

  // Count how many features will be selected in each cell
  int ncells = hcells*wcells;
  int selected = 0;
  int cells_left = ncells-nempty;

  // Iterate until enough features have been selected or cells don't have more features
  while ((nfeatures-selected) > 0 && cells_left > 0) {
    int npercell = ceil((double)(nfeatures-selected)/(double)cells_left);
    cells_left = 0;

    for (int i=0; i<hcells; i++) {
      for(int j=0; j<wcells; j++) {
        // Still have features not selected
        if (nleft[i][j] > 0) {
          if (nleft[i][j] > npercell) {  // More features than needed
            nselected[i][j] += npercell;
            selected += npercell;
            nleft[i][j] -= npercell;
            cells_left++;
          } else {                        // Not enough features in cell
            nselected[i][j] += nleft[i][j];
            selected += nleft[i][j];
            nleft[i][j] = 0;
          }
        }
      }
    }
  }

  // Select best features for each cell
  for (int i=0; i<hcells; i++) {
    for(int j=0; j<wcells; j++) {
      cv::KeyPointsFilter::retainBest(cell_fts[i][j],nselected[i][j]);
      for (auto it=cell_fts[i][j].begin(); it != cell_fts[i][j].end(); it++)
        fts.push_back(*it);
    }
  }

  // Remove some features if we have selected too many
  if ((int)fts.size() > nfeatures)
    cv::KeyPointsFilter::retainBest(fts,nfeatures);

  for (auto it=fts.begin(); it != fts.end(); it++)
    pixels->push_back(Eigen::Vector2i((*it).pt.x, (*it).pt.y));
}

void FastDetector::DetectPyramid(const vector<cv::Mat> &pyramid, std::vector<std::vector<Eigen::Vector2i>> *corners, int nfeatures) {
  int levelfeatures;
  double val, scale, factor;

  assert(static_cast<int>(pyramid.size()) >= Config::MaxFastLevels());
  assert(static_cast<int>(corners->size()) == Config::MaxFastLevels());

  // Calc features detected per level
  scale = 1.2;
  factor = 1.0;
  val = 0.0;
  for(int i=0; i<Config::MaxFastLevels(); i++) {
    val += factor;
    factor /= scale;
  }
  levelfeatures = static_cast<int>(nfeatures/val);

  // Detect fast corners for each pyramid level
  for (int i=0; i < Config::MaxFastLevels(); i++) {
    SelectPixels(pyramid[i], &(*corners)[i], levelfeatures);
    levelfeatures = static_cast<int>(levelfeatures/scale);
  }
}

void FastDetector::FilterCorners(const std::vector<cv::Mat> &pyramid, const std::vector<std::vector<Eigen::Vector2i>> &corners,
                     std::vector<Eigen::Vector3i> *fcorners) {
  int margin;
  int pos, px, py, scale;
  double score;

  // Set margin
  if (Config::UseORB())
    margin = 1+Config::ORBSize()/2;
  else
    margin = 1+Config::PatchSize()/2;

  for (int level=0; level < Config::MaxFastLevels(); level++) {

    // Save a corner in each grid cell
    for (auto it=corners[level].begin(); it != corners[level].end(); it++) {
      px = (*it)(0);
      py = (*it)(1);
      scale = (1 << level);

      if (px < margin || py < margin || px >= pyramid[level].cols-margin || py >= pyramid[level].rows-margin)
        continue;

      pos = static_cast<int>((py*scale)/cell_size_)*grid_width_ + static_cast<int>((px*scale)/cell_size_);

      // Check if cell is locked
      if (grid_mask_[pos])
        continue;

      // Save corner according to its Shi-Thomasi score
      score = FindShiTomasiScoreAtPoint(pyramid[level], px, py);
      if (score > corners_grid_.at(pos).score) {
        corners_grid_.at(pos) = CandidateCorner(px*scale, py*scale, level, score);
      }
    }
  }

  // Create points from grid candidates
  for (vector<CandidateCorner>::iterator it=corners_grid_.begin(); it != corners_grid_.end(); it++) {
    if ((*it).score > Config::MinFeatureScore())
      fcorners->push_back(Eigen::Vector3i(it->x, it->y, it->level));
  }
}

}  // namespace sdvl



