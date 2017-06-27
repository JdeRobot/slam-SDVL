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

FastDetector::FastDetector(int size) {
  cell_size_ = size;
}

FastDetector::FastDetector(int width, int height, int size) {
  cell_size_ = size;
  InitGrid(width, height);
}

void FastDetector::InitGrid(int width, int height) {
  grid_width_  = ceil(static_cast<double>(width)/cell_size_);
  grid_height_ = ceil(static_cast<double>(height)/cell_size_);

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

void FastDetector::Detect(const cv::Mat &src, vector<Eigen::Vector2i> *corners) {
  FastCornerDetector_10(src, corners, Config::FastThreshold());
}

void FastDetector::DetectCV(const cv::Mat &src, vector<Eigen::Vector2i> *corners, bool nms) {
  vector<cv::KeyPoint> fst;

  // Get all corners
  cv::FASTX(src, fst, Config::FastThreshold(), nms, cv::FastFeatureDetector::TYPE_7_12);
  for (auto it=fst.begin(); it != fst.end(); it++)
    corners->push_back(Eigen::Vector2i((*it).pt.x, (*it).pt.y));
}

void FastDetector::DetectPyramid(const vector<cv::Mat> &pyramid, std::vector<std::vector<Eigen::Vector2i>> *corners) {
  assert(static_cast<int>(pyramid.size()) >= Config::MaxFastLevels());
  assert(static_cast<int>(corners->size()) == Config::MaxFastLevels());

  // Detect fast corners for each pyramid level
  for (int i=0; i < Config::MaxFastLevels(); i++)
    Detect(pyramid[i], &(*corners)[i]);
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
    vector<Eigen::Vector2i> corners_nms;

    // Non maximum suppression
    NonMaxSuppression(pyramid[level], corners[level], &corners_nms);

    // Save a corner in each grid cell
    for (auto it=corners_nms.begin(); it != corners_nms.end(); it++) {
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
    if ((*it).score > Config::MinFeatureScore()) {
      fcorners->push_back(Eigen::Vector3i(it->x, it->y, it->level));
    }
  }
}

void FastDetector::NonMaxSuppression(const cv::Mat &src, const std::vector<Eigen::Vector2i> &corners_in,
                                     std::vector<Eigen::Vector2i> *corners_out) {
  vector<int> scores;

  // Compute scores and select non max suppression corners
  ComputeFastScore(src, corners_in, Config::FastThreshold(), &scores);
  NonMaxSuppression(corners_in, scores, corners_out);
}

void FastDetector::FastCornerDetector_10(const cv::Mat &src, vector<Eigen::Vector2i> *corners, int threshold) {
  int y, cb, c_b;
  const unsigned char *line_max, *line_min;
  const unsigned char *cache_0;
  int imgstep = src.step;

  int pixel[16] = {
    0 + imgstep * 3,
    1 + imgstep * 3,
    2 + imgstep * 2,
    3 + imgstep * 1,
    3 + imgstep * 0,
    3 + imgstep * -1,
    2 + imgstep * -2,
    1 + imgstep * -3,
    0 + imgstep * -3,
    -1 + imgstep * -3,
    -2 + imgstep * -2,
    -3 + imgstep * -1,
    -3 + imgstep * 0,
    -3 + imgstep * 1,
    -2 + imgstep * 2,
    -1 + imgstep * 3,
  };

  for (y = 3; y < src.rows - 3; y++) {
    cache_0 = src.data + y * src.step + 3;
    line_min = cache_0 - 3;
    line_max = src.data + y * src.step + (src.cols - 3);

    for (; cache_0 < line_max; cache_0++) {
      cb = *cache_0 + threshold;
      c_b = *cache_0 - threshold;

      if (*(cache_0 + pixel[0]) > cb)
        if (*(cache_0 + pixel[8]) > cb)
          if (*(cache_0 + pixel[3]) > cb)
            if (*(cache_0 + pixel[5]) > cb)
              if (*(cache_0 + pixel[2]) > cb)
                if (*(cache_0 + pixel[6]) > cb)
                  if (*(cache_0 + 3) > cb)
                    if (*(cache_0 + pixel[7]) > cb)
                      if (*(cache_0 + pixel[1]) > cb)
                        if (*(cache_0 + pixel[9]) > cb)
                          goto success;
                        else if (*(cache_0 + pixel[15]) > cb)
                          goto success;
                        else
                          continue;
                      else if (*(cache_0 + pixel[1]) < c_b)
                        if (*(cache_0 + pixel[9]) > cb)
                          if (*(cache_0 + pixel[10]) > cb)
                            if (*(cache_0 + pixel[11]) > cb)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else if (*(cache_0 + pixel[11]) > cb)
                        if (*(cache_0 + pixel[10]) > cb)
                          if (*(cache_0 + pixel[9]) > cb)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else if (*(cache_0 + pixel[7]) < c_b)
                      if (*(cache_0 + pixel[1]) > cb)
                        if (*(cache_0 + pixel[13]) > cb)
                          if (*(cache_0 + pixel[14]) > cb)
                            if (*(cache_0 + pixel[15]) > cb)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else if (*(cache_0 + pixel[13]) > cb)
                      if (*(cache_0 + pixel[14]) > cb)
                        if (*(cache_0 + pixel[15]) > cb)
                          if (*(cache_0 + pixel[1]) > cb)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else if (*(cache_0 + 3) < c_b)
                    if (*(cache_0 + pixel[10]) > cb)
                      if (*(cache_0 + pixel[11]) > cb)
                        if (*(cache_0 + -3) > cb)
                          if (*(cache_0 + pixel[13]) > cb)
                            if (*(cache_0 + pixel[14]) > cb)
                              if (*(cache_0 + pixel[1]) > cb)
                                if (*(cache_0 + pixel[15]) > cb)
                                  goto success;
                                else if (*(cache_0 + pixel[7]) > cb)
                                  if (*(cache_0 + pixel[9]) > cb)
                                    goto success;
                                  else
                                    continue;
                                else
                                  continue;
                              else if (*(cache_0 + pixel[7]) > cb)
                                if (*(cache_0 + pixel[9]) > cb)
                                  goto success;
                                else
                                  continue;
                              else
                                continue;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else if (*(cache_0 + -3) > cb)
                    if (*(cache_0 + pixel[14]) > cb)
                      if (*(cache_0 + pixel[10]) > cb)
                        if (*(cache_0 + pixel[11]) > cb)
                          if (*(cache_0 + pixel[13]) > cb)
                            if (*(cache_0 + pixel[1]) > cb)
                              if (*(cache_0 + pixel[7]) > cb)
                                if (*(cache_0 + pixel[9]) > cb)
                                  goto success;
                                else if (*(cache_0 + pixel[15]) > cb)
                                  goto success;
                                else
                                  continue;
                              else if (*(cache_0 + pixel[15]) > cb)
                                goto success;
                              else
                                continue;
                            else if (*(cache_0 + pixel[1]) < c_b)
                              if (*(cache_0 + pixel[7]) > cb)
                                if (*(cache_0 + pixel[9]) > cb)
                                  goto success;
                                else
                                  continue;
                              else
                                continue;
                            else if (*(cache_0 + pixel[9]) > cb)
                              if (*(cache_0 + pixel[7]) > cb)
                                goto success;
                              else
                                continue;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else if (*(cache_0 + pixel[6]) < c_b)
                  if (*(cache_0 + -3) > cb)
                    if (*(cache_0 + pixel[13]) > cb)
                      if (*(cache_0 + pixel[14]) > cb)
                        if (*(cache_0 + pixel[15]) > cb)
                          if (*(cache_0 + pixel[1]) > cb)
                            if (*(cache_0 + 3) > cb)
                              goto success;
                            else if (*(cache_0 + pixel[10]) > cb)
                              if (*(cache_0 + pixel[11]) > cb)
                                goto success;
                              else
                                continue;
                            else
                              continue;
                          else if (*(cache_0 + pixel[7]) > cb)
                            if (*(cache_0 + pixel[9]) > cb)
                              if (*(cache_0 + pixel[10]) > cb)
                                if (*(cache_0 + pixel[11]) > cb)
                                  goto success;
                                else
                                  continue;
                              else
                                continue;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else if (*(cache_0 + -3) > cb)
                  if (*(cache_0 + pixel[14]) > cb)
                    if (*(cache_0 + pixel[15]) > cb)
                      if (*(cache_0 + pixel[13]) > cb)
                        if (*(cache_0 + pixel[1]) > cb)
                          if (*(cache_0 + 3) > cb)
                            goto success;
                          else if (*(cache_0 + pixel[10]) > cb)
                            if (*(cache_0 + pixel[11]) > cb)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else if (*(cache_0 + pixel[1]) < c_b)
                          if (*(cache_0 + pixel[7]) > cb)
                            if (*(cache_0 + pixel[9]) > cb)
                              if (*(cache_0 + pixel[10]) > cb)
                                if (*(cache_0 + pixel[11]) > cb)
                                  goto success;
                                else
                                  continue;
                              else
                                continue;
                            else
                              continue;
                          else
                            continue;
                        else if (*(cache_0 + pixel[7]) > cb)
                          if (*(cache_0 + pixel[10]) > cb)
                            if (*(cache_0 + pixel[11]) > cb)
                              if (*(cache_0 + pixel[9]) > cb)
                                goto success;
                              else
                                continue;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else if (*(cache_0 + pixel[2]) < c_b)
                if (*(cache_0 + -3) > cb)
                  if (*(cache_0 + pixel[9]) > cb)
                    if (*(cache_0 + pixel[10]) > cb)
                      if (*(cache_0 + pixel[11]) > cb)
                        if (*(cache_0 + pixel[7]) > cb)
                          if (*(cache_0 + pixel[6]) > cb)
                            if (*(cache_0 + 3) > cb)
                              goto success;
                            else if (*(cache_0 + pixel[13]) > cb)
                              if (*(cache_0 + pixel[14]) > cb)
                                goto success;
                              else
                                continue;
                            else
                              continue;
                          else if (*(cache_0 + pixel[13]) > cb)
                            if (*(cache_0 + pixel[14]) > cb)
                              if (*(cache_0 + pixel[15]) > cb)
                                goto success;
                              else
                                continue;
                            else
                              continue;
                          else
                            continue;
                        else if (*(cache_0 + pixel[1]) > cb)
                          if (*(cache_0 + pixel[13]) > cb)
                            if (*(cache_0 + pixel[14]) > cb)
                              if (*(cache_0 + pixel[15]) > cb)
                                goto success;
                              else
                                continue;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else if (*(cache_0 + pixel[11]) > cb)
                if (*(cache_0 + pixel[10]) > cb)
                  if (*(cache_0 + -3) > cb)
                    if (*(cache_0 + pixel[9]) > cb)
                      if (*(cache_0 + pixel[7]) > cb)
                        if (*(cache_0 + pixel[6]) > cb)
                          if (*(cache_0 + 3) > cb)
                            goto success;
                          else if (*(cache_0 + 3) < c_b)
                            if (*(cache_0 + pixel[13]) > cb)
                              if (*(cache_0 + pixel[14]) > cb)
                                goto success;
                              else
                                continue;
                            else
                              continue;
                          else if (*(cache_0 + pixel[14]) > cb)
                            if (*(cache_0 + pixel[13]) > cb)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else if (*(cache_0 + pixel[6]) < c_b)
                          if (*(cache_0 + pixel[13]) > cb)
                            if (*(cache_0 + pixel[14]) > cb)
                              if (*(cache_0 + pixel[15]) > cb)
                                goto success;
                              else
                                continue;
                            else
                              continue;
                          else
                            continue;
                        else if (*(cache_0 + pixel[14]) > cb)
                          if (*(cache_0 + pixel[13]) > cb)
                            if (*(cache_0 + pixel[15]) > cb)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else if (*(cache_0 + pixel[7]) < c_b)
                        if (*(cache_0 + pixel[1]) > cb)
                          if (*(cache_0 + pixel[13]) > cb)
                            if (*(cache_0 + pixel[14]) > cb)
                              if (*(cache_0 + pixel[15]) > cb)
                                goto success;
                              else
                                continue;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else if (*(cache_0 + pixel[14]) > cb)
                        if (*(cache_0 + pixel[1]) > cb)
                          if (*(cache_0 + pixel[13]) > cb)
                            if (*(cache_0 + pixel[15]) > cb)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else if (*(cache_0 + pixel[5]) < c_b)
              if (*(cache_0 + pixel[13]) > cb)
                if (*(cache_0 + pixel[11]) > cb)
                  if (*(cache_0 + -3) > cb)
                    if (*(cache_0 + pixel[14]) > cb)
                      if (*(cache_0 + pixel[15]) > cb)
                        if (*(cache_0 + pixel[10]) > cb)
                          if (*(cache_0 + pixel[9]) > cb)
                            if (*(cache_0 + pixel[1]) > cb)
                              goto success;
                            else if (*(cache_0 + pixel[7]) > cb)
                              goto success;
                            else
                              continue;
                          else if (*(cache_0 + pixel[1]) > cb)
                            if (*(cache_0 + pixel[2]) > cb)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else if (*(cache_0 + pixel[1]) > cb)
                          if (*(cache_0 + pixel[2]) > cb)
                            if (*(cache_0 + 3) > cb)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else if (*(cache_0 + -3) > cb)
              if (*(cache_0 + pixel[14]) > cb)
                if (*(cache_0 + pixel[11]) > cb)
                  if (*(cache_0 + pixel[15]) > cb)
                    if (*(cache_0 + pixel[10]) > cb)
                      if (*(cache_0 + pixel[13]) > cb)
                        if (*(cache_0 + pixel[1]) > cb)
                          if (*(cache_0 + pixel[2]) > cb)
                            goto success;
                          else if (*(cache_0 + pixel[9]) > cb)
                            goto success;
                          else
                            continue;
                        else if (*(cache_0 + pixel[7]) > cb)
                          if (*(cache_0 + pixel[9]) > cb)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else if (*(cache_0 + pixel[10]) < c_b)
                      if (*(cache_0 + pixel[1]) > cb)
                        if (*(cache_0 + pixel[2]) > cb)
                          if (*(cache_0 + 3) > cb)
                            if (*(cache_0 + pixel[13]) > cb)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else if (*(cache_0 + 3) > cb)
                      if (*(cache_0 + pixel[2]) > cb)
                        if (*(cache_0 + pixel[1]) > cb)
                          if (*(cache_0 + pixel[13]) > cb)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else
              continue;
          else if (*(cache_0 + pixel[3]) < c_b)
            if (*(cache_0 + -3) > cb)
              if (*(cache_0 + pixel[10]) > cb)
                if (*(cache_0 + pixel[13]) > cb)
                  if (*(cache_0 + pixel[9]) > cb)
                    if (*(cache_0 + pixel[11]) > cb)
                      if (*(cache_0 + pixel[14]) > cb)
                        if (*(cache_0 + pixel[15]) > cb)
                          if (*(cache_0 + pixel[7]) > cb)
                            goto success;
                          else if (*(cache_0 + pixel[1]) > cb)
                            goto success;
                          else
                            continue;
                        else if (*(cache_0 + pixel[5]) > cb)
                          if (*(cache_0 + pixel[6]) > cb)
                            if (*(cache_0 + pixel[7]) > cb)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else if (*(cache_0 + 3) > cb)
                        if (*(cache_0 + pixel[5]) > cb)
                          if (*(cache_0 + pixel[6]) > cb)
                            if (*(cache_0 + pixel[7]) > cb)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else
              continue;
          else if (*(cache_0 + -3) > cb)
            if (*(cache_0 + pixel[10]) > cb)
              if (*(cache_0 + pixel[14]) > cb)
                if (*(cache_0 + pixel[11]) > cb)
                  if (*(cache_0 + pixel[13]) > cb)
                    if (*(cache_0 + pixel[9]) > cb)
                      if (*(cache_0 + pixel[7]) > cb)
                        if (*(cache_0 + pixel[15]) > cb)
                          goto success;
                        else if (*(cache_0 + pixel[5]) > cb)
                          if (*(cache_0 + pixel[6]) > cb)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else if (*(cache_0 + pixel[1]) > cb)
                        if (*(cache_0 + pixel[15]) > cb)
                          goto success;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else if (*(cache_0 + pixel[14]) < c_b)
                if (*(cache_0 + 3) > cb)
                  if (*(cache_0 + pixel[5]) > cb)
                    if (*(cache_0 + pixel[6]) > cb)
                      if (*(cache_0 + pixel[7]) > cb)
                        if (*(cache_0 + pixel[9]) > cb)
                          if (*(cache_0 + pixel[11]) > cb)
                            if (*(cache_0 + pixel[13]) > cb)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else if (*(cache_0 + 3) > cb)
                if (*(cache_0 + pixel[13]) > cb)
                  if (*(cache_0 + pixel[6]) > cb)
                    if (*(cache_0 + pixel[11]) > cb)
                      if (*(cache_0 + pixel[7]) > cb)
                        if (*(cache_0 + pixel[5]) > cb)
                          if (*(cache_0 + pixel[9]) > cb)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else
              continue;
          else
            continue;
        else if (*(cache_0 + pixel[8]) < c_b)
          if (*(cache_0 + pixel[11]) > cb)
            if (*(cache_0 + pixel[2]) > cb)
              if (*(cache_0 + pixel[15]) > cb)
                if (*(cache_0 + pixel[1]) > cb)
                  if (*(cache_0 + pixel[14]) > cb)
                    if (*(cache_0 + pixel[13]) > cb)
                      if (*(cache_0 + pixel[3]) > cb)
                        if (*(cache_0 + -3) > cb)
                          if (*(cache_0 + 3) > cb)
                            goto success;
                          else if (*(cache_0 + pixel[10]) > cb)
                            goto success;
                          else
                            continue;
                        else if (*(cache_0 + 3) > cb)
                          if (*(cache_0 + pixel[5]) > cb)
                            if (*(cache_0 + pixel[6]) > cb)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else if (*(cache_0 + pixel[9]) > cb)
                        if (*(cache_0 + pixel[10]) > cb)
                          if (*(cache_0 + -3) > cb)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else if (*(cache_0 + pixel[3]) > cb)
                      if (*(cache_0 + 3) > cb)
                        if (*(cache_0 + pixel[5]) > cb)
                          if (*(cache_0 + pixel[6]) > cb)
                            if (*(cache_0 + pixel[7]) > cb)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else if (*(cache_0 + pixel[2]) < c_b)
              if (*(cache_0 + pixel[1]) < c_b)
                if (*(cache_0 + pixel[3]) < c_b)
                  if (*(cache_0 + 3) < c_b)
                    if (*(cache_0 + pixel[5]) < c_b)
                      if (*(cache_0 + pixel[6]) < c_b)
                        if (*(cache_0 + pixel[7]) < c_b)
                          if (*(cache_0 + pixel[9]) < c_b)
                            if (*(cache_0 + pixel[10]) < c_b)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else
              continue;
          else if (*(cache_0 + pixel[11]) < c_b)
            if (*(cache_0 + pixel[6]) > cb)
              if (*(cache_0 + pixel[14]) > cb)
                if (*(cache_0 + pixel[3]) > cb)
                  if (*(cache_0 + pixel[1]) > cb)
                    if (*(cache_0 + pixel[2]) > cb)
                      if (*(cache_0 + 3) > cb)
                        if (*(cache_0 + pixel[5]) > cb)
                          if (*(cache_0 + pixel[15]) > cb)
                            if (*(cache_0 + pixel[7]) > cb)
                              goto success;
                            else if (*(cache_0 + pixel[13]) > cb)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else if (*(cache_0 + pixel[6]) < c_b)
              if (*(cache_0 + pixel[10]) > cb)
                if (*(cache_0 + pixel[1]) > cb)
                  if (*(cache_0 + pixel[2]) > cb)
                    if (*(cache_0 + pixel[3]) > cb)
                      if (*(cache_0 + 3) > cb)
                        if (*(cache_0 + pixel[5]) > cb)
                          if (*(cache_0 + -3) > cb)
                            if (*(cache_0 + pixel[13]) > cb)
                              if (*(cache_0 + pixel[14]) > cb)
                                if (*(cache_0 + pixel[15]) > cb)
                                  goto success;
                                else
                                  continue;
                              else
                                continue;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else if (*(cache_0 + pixel[10]) < c_b)
                if (*(cache_0 + pixel[5]) > cb)
                  if (*(cache_0 + pixel[7]) > cb)
                    if (*(cache_0 + pixel[1]) > cb)
                      if (*(cache_0 + pixel[2]) > cb)
                        if (*(cache_0 + pixel[3]) > cb)
                          if (*(cache_0 + 3) > cb)
                            if (*(cache_0 + -3) > cb)
                              if (*(cache_0 + pixel[13]) > cb)
                                if (*(cache_0 + pixel[14]) > cb)
                                  if (*(cache_0 + pixel[15]) > cb)
                                    goto success;
                                  else
                                    continue;
                                else
                                  continue;
                              else
                                continue;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else if (*(cache_0 + pixel[7]) < c_b)
                    if (*(cache_0 + pixel[14]) > cb)
                      if (*(cache_0 + -3) > cb)
                        if (*(cache_0 + pixel[1]) > cb)
                          if (*(cache_0 + pixel[2]) > cb)
                            if (*(cache_0 + pixel[3]) > cb)
                              if (*(cache_0 + 3) > cb)
                                if (*(cache_0 + pixel[13]) > cb)
                                  if (*(cache_0 + pixel[15]) > cb)
                                    goto success;
                                  else
                                    continue;
                                else
                                  continue;
                              else
                                continue;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else if (*(cache_0 + pixel[14]) < c_b)
                      if (*(cache_0 + pixel[9]) < c_b)
                        if (*(cache_0 + -3) < c_b)
                          if (*(cache_0 + pixel[13]) < c_b)
                            if (*(cache_0 + pixel[15]) < c_b)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else if (*(cache_0 + -3) > cb)
                    if (*(cache_0 + pixel[1]) > cb)
                      if (*(cache_0 + pixel[2]) > cb)
                        if (*(cache_0 + pixel[3]) > cb)
                          if (*(cache_0 + 3) > cb)
                            if (*(cache_0 + pixel[13]) > cb)
                              if (*(cache_0 + pixel[14]) > cb)
                                if (*(cache_0 + pixel[15]) > cb)
                                  goto success;
                                else
                                  continue;
                              else
                                continue;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else if (*(cache_0 + pixel[5]) < c_b)
                  if (*(cache_0 + -3) > cb)
                    if (*(cache_0 + pixel[2]) < c_b)
                      if (*(cache_0 + pixel[3]) < c_b)
                        if (*(cache_0 + 3) < c_b)
                          if (*(cache_0 + pixel[7]) < c_b)
                            if (*(cache_0 + pixel[9]) < c_b)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else if (*(cache_0 + -3) < c_b)
                    if (*(cache_0 + pixel[9]) < c_b)
                      if (*(cache_0 + 3) > cb)
                        if (*(cache_0 + pixel[7]) < c_b)
                          if (*(cache_0 + pixel[13]) < c_b)
                            if (*(cache_0 + pixel[14]) < c_b)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else if (*(cache_0 + 3) < c_b)
                        if (*(cache_0 + pixel[7]) < c_b)
                          if (*(cache_0 + pixel[13]) < c_b)
                            goto success;
                          else if (*(cache_0 + pixel[3]) < c_b)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else if (*(cache_0 + pixel[14]) < c_b)
                        if (*(cache_0 + pixel[13]) < c_b)
                          if (*(cache_0 + pixel[7]) < c_b)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else if (*(cache_0 + pixel[2]) < c_b)
                    if (*(cache_0 + pixel[7]) < c_b)
                      if (*(cache_0 + pixel[3]) < c_b)
                        if (*(cache_0 + pixel[9]) < c_b)
                          if (*(cache_0 + 3) < c_b)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else if (*(cache_0 + pixel[15]) < c_b)
                  if (*(cache_0 + pixel[14]) < c_b)
                    if (*(cache_0 + pixel[7]) < c_b)
                      if (*(cache_0 + pixel[9]) < c_b)
                        if (*(cache_0 + -3) < c_b)
                          if (*(cache_0 + pixel[13]) < c_b)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else if (*(cache_0 + -3) > cb)
                if (*(cache_0 + pixel[1]) > cb)
                  if (*(cache_0 + pixel[2]) > cb)
                    if (*(cache_0 + pixel[3]) > cb)
                      if (*(cache_0 + 3) > cb)
                        if (*(cache_0 + pixel[5]) > cb)
                          if (*(cache_0 + pixel[13]) > cb)
                            if (*(cache_0 + pixel[14]) > cb)
                              if (*(cache_0 + pixel[15]) > cb)
                                goto success;
                              else
                                continue;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else if (*(cache_0 + -3) > cb)
              if (*(cache_0 + pixel[3]) > cb)
                if (*(cache_0 + pixel[1]) > cb)
                  if (*(cache_0 + pixel[2]) > cb)
                    if (*(cache_0 + 3) > cb)
                      if (*(cache_0 + pixel[5]) > cb)
                        if (*(cache_0 + pixel[13]) > cb)
                          if (*(cache_0 + pixel[14]) > cb)
                            if (*(cache_0 + pixel[15]) > cb)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else
              continue;
          else if (*(cache_0 + pixel[3]) > cb)
            if (*(cache_0 + pixel[5]) > cb)
              if (*(cache_0 + pixel[14]) > cb)
                if (*(cache_0 + pixel[15]) > cb)
                  if (*(cache_0 + pixel[13]) > cb)
                    if (*(cache_0 + pixel[1]) > cb)
                      if (*(cache_0 + pixel[2]) > cb)
                        if (*(cache_0 + 3) > cb)
                          if (*(cache_0 + pixel[6]) > cb)
                            goto success;
                          else if (*(cache_0 + -3) > cb)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else if (*(cache_0 + pixel[13]) < c_b)
                    if (*(cache_0 + pixel[6]) > cb)
                      if (*(cache_0 + pixel[1]) > cb)
                        if (*(cache_0 + pixel[2]) > cb)
                          if (*(cache_0 + 3) > cb)
                            if (*(cache_0 + pixel[7]) > cb)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else if (*(cache_0 + pixel[7]) > cb)
                    if (*(cache_0 + pixel[1]) > cb)
                      if (*(cache_0 + pixel[2]) > cb)
                        if (*(cache_0 + 3) > cb)
                          if (*(cache_0 + pixel[6]) > cb)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else
              continue;
          else if (*(cache_0 + pixel[3]) < c_b)
            if (*(cache_0 + pixel[1]) < c_b)
              if (*(cache_0 + pixel[10]) < c_b)
                if (*(cache_0 + pixel[2]) < c_b)
                  if (*(cache_0 + 3) < c_b)
                    if (*(cache_0 + pixel[5]) < c_b)
                      if (*(cache_0 + pixel[6]) < c_b)
                        if (*(cache_0 + pixel[7]) < c_b)
                          if (*(cache_0 + pixel[9]) < c_b)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else
              continue;
          else
            continue;
        else if (*(cache_0 + pixel[3]) > cb)
          if (*(cache_0 + pixel[14]) > cb)
            if (*(cache_0 + -3) > cb)
              if (*(cache_0 + pixel[2]) > cb)
                if (*(cache_0 + 3) > cb)
                  if (*(cache_0 + pixel[15]) > cb)
                    if (*(cache_0 + pixel[1]) > cb)
                      if (*(cache_0 + pixel[13]) > cb)
                        if (*(cache_0 + pixel[11]) > cb)
                          goto success;
                        else if (*(cache_0 + pixel[5]) > cb)
                          goto success;
                        else
                          continue;
                      else if (*(cache_0 + pixel[13]) < c_b)
                        if (*(cache_0 + pixel[5]) > cb)
                          if (*(cache_0 + pixel[6]) > cb)
                            if (*(cache_0 + pixel[7]) > cb)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else if (*(cache_0 + pixel[7]) > cb)
                        if (*(cache_0 + pixel[5]) > cb)
                          if (*(cache_0 + pixel[6]) > cb)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else if (*(cache_0 + 3) < c_b)
                  if (*(cache_0 + pixel[1]) > cb)
                    if (*(cache_0 + pixel[10]) > cb)
                      if (*(cache_0 + pixel[11]) > cb)
                        if (*(cache_0 + pixel[13]) > cb)
                          if (*(cache_0 + pixel[15]) > cb)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else if (*(cache_0 + pixel[10]) > cb)
                  if (*(cache_0 + pixel[13]) > cb)
                    if (*(cache_0 + pixel[11]) > cb)
                      if (*(cache_0 + pixel[15]) > cb)
                        if (*(cache_0 + pixel[1]) > cb)
                          goto success;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else if (*(cache_0 + -3) < c_b)
              if (*(cache_0 + pixel[6]) > cb)
                if (*(cache_0 + pixel[1]) > cb)
                  if (*(cache_0 + pixel[2]) > cb)
                    if (*(cache_0 + 3) > cb)
                      if (*(cache_0 + pixel[5]) > cb)
                        if (*(cache_0 + pixel[15]) > cb)
                          if (*(cache_0 + pixel[7]) > cb)
                            goto success;
                          else if (*(cache_0 + pixel[13]) > cb)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else if (*(cache_0 + pixel[6]) > cb)
              if (*(cache_0 + pixel[2]) > cb)
                if (*(cache_0 + pixel[5]) > cb)
                  if (*(cache_0 + pixel[13]) > cb)
                    if (*(cache_0 + pixel[15]) > cb)
                      if (*(cache_0 + 3) > cb)
                        if (*(cache_0 + pixel[1]) > cb)
                          goto success;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else if (*(cache_0 + pixel[13]) < c_b)
                    if (*(cache_0 + pixel[1]) > cb)
                      if (*(cache_0 + 3) > cb)
                        if (*(cache_0 + pixel[7]) > cb)
                          if (*(cache_0 + pixel[15]) > cb)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else if (*(cache_0 + pixel[7]) > cb)
                    if (*(cache_0 + pixel[15]) > cb)
                      if (*(cache_0 + 3) > cb)
                        if (*(cache_0 + pixel[1]) > cb)
                          goto success;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else
              continue;
          else
            continue;
        else if (*(cache_0 + pixel[3]) < c_b)
          if (*(cache_0 + pixel[2]) > cb)
            if (*(cache_0 + pixel[9]) > cb)
              if (*(cache_0 + pixel[1]) > cb)
                if (*(cache_0 + pixel[10]) > cb)
                  if (*(cache_0 + pixel[11]) > cb)
                    if (*(cache_0 + -3) > cb)
                      if (*(cache_0 + pixel[13]) > cb)
                        if (*(cache_0 + pixel[14]) > cb)
                          if (*(cache_0 + pixel[15]) > cb)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else
              continue;
          else
            continue;
        else if (*(cache_0 + pixel[9]) > cb)
          if (*(cache_0 + pixel[2]) > cb)
            if (*(cache_0 + -3) > cb)
              if (*(cache_0 + pixel[14]) > cb)
                if (*(cache_0 + pixel[11]) > cb)
                  if (*(cache_0 + pixel[13]) > cb)
                    if (*(cache_0 + pixel[15]) > cb)
                      if (*(cache_0 + pixel[10]) > cb)
                        if (*(cache_0 + pixel[1]) > cb)
                          goto success;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else
              continue;
          else
            continue;
        else
          continue;
      else if (*(cache_0 + pixel[0]) < c_b)
        if (*(cache_0 + pixel[8]) > cb)
          if (*(cache_0 + pixel[2]) > cb)
            if (*(cache_0 + pixel[10]) > cb)
              if (*(cache_0 + pixel[6]) > cb)
                if (*(cache_0 + pixel[7]) > cb)
                  if (*(cache_0 + pixel[9]) > cb)
                    if (*(cache_0 + pixel[5]) > cb)
                      if (*(cache_0 + pixel[11]) > cb)
                        if (*(cache_0 + 3) > cb)
                          if (*(cache_0 + pixel[3]) > cb)
                            goto success;
                          else if (*(cache_0 + -3) > cb)
                            if (*(cache_0 + pixel[13]) > cb)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else if (*(cache_0 + -3) > cb)
                          if (*(cache_0 + pixel[13]) > cb)
                            if (*(cache_0 + pixel[14]) > cb)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else if (*(cache_0 + pixel[1]) > cb)
                        if (*(cache_0 + pixel[3]) > cb)
                          if (*(cache_0 + 3) > cb)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else if (*(cache_0 + pixel[5]) < c_b)
                      if (*(cache_0 + pixel[11]) > cb)
                        if (*(cache_0 + -3) > cb)
                          if (*(cache_0 + pixel[13]) > cb)
                            if (*(cache_0 + pixel[14]) > cb)
                              if (*(cache_0 + pixel[15]) > cb)
                                goto success;
                              else
                                continue;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else if (*(cache_0 + pixel[13]) > cb)
                      if (*(cache_0 + pixel[11]) > cb)
                        if (*(cache_0 + -3) > cb)
                          if (*(cache_0 + pixel[14]) > cb)
                            if (*(cache_0 + pixel[15]) > cb)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else
              continue;
          else if (*(cache_0 + pixel[2]) < c_b)
            if (*(cache_0 + pixel[13]) > cb)
              if (*(cache_0 + pixel[6]) > cb)
                if (*(cache_0 + pixel[11]) > cb)
                  if (*(cache_0 + pixel[9]) > cb)
                    if (*(cache_0 + pixel[7]) > cb)
                      if (*(cache_0 + pixel[10]) > cb)
                        if (*(cache_0 + pixel[5]) > cb)
                          if (*(cache_0 + -3) > cb)
                            if (*(cache_0 + 3) > cb)
                              goto success;
                            else if (*(cache_0 + pixel[14]) > cb)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else if (*(cache_0 + pixel[15]) > cb)
                          if (*(cache_0 + -3) > cb)
                            if (*(cache_0 + pixel[14]) > cb)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else if (*(cache_0 + pixel[6]) < c_b)
                if (*(cache_0 + pixel[7]) < c_b)
                  if (*(cache_0 + pixel[1]) < c_b)
                    if (*(cache_0 + pixel[3]) < c_b)
                      if (*(cache_0 + 3) < c_b)
                        if (*(cache_0 + pixel[5]) < c_b)
                          if (*(cache_0 + pixel[14]) < c_b)
                            if (*(cache_0 + pixel[15]) < c_b)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else if (*(cache_0 + pixel[13]) < c_b)
              if (*(cache_0 + pixel[3]) > cb)
                if (*(cache_0 + pixel[10]) > cb)
                  if (*(cache_0 + pixel[7]) > cb)
                    if (*(cache_0 + 3) > cb)
                      if (*(cache_0 + pixel[5]) > cb)
                        if (*(cache_0 + pixel[6]) > cb)
                          if (*(cache_0 + pixel[9]) > cb)
                            if (*(cache_0 + pixel[11]) > cb)
                              if (*(cache_0 + -3) > cb)
                                goto success;
                              else
                                continue;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else if (*(cache_0 + pixel[10]) < c_b)
                  if (*(cache_0 + pixel[9]) < c_b)
                    if (*(cache_0 + pixel[1]) < c_b)
                      if (*(cache_0 + pixel[11]) < c_b)
                        if (*(cache_0 + -3) < c_b)
                          if (*(cache_0 + pixel[14]) < c_b)
                            if (*(cache_0 + pixel[15]) < c_b)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else if (*(cache_0 + pixel[3]) < c_b)
                if (*(cache_0 + pixel[15]) < c_b)
                  if (*(cache_0 + pixel[1]) < c_b)
                    if (*(cache_0 + pixel[5]) > cb)
                      if (*(cache_0 + pixel[10]) < c_b)
                        if (*(cache_0 + pixel[14]) < c_b)
                          if (*(cache_0 + pixel[11]) < c_b)
                            if (*(cache_0 + -3) < c_b)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else if (*(cache_0 + 3) < c_b)
                        if (*(cache_0 + pixel[11]) < c_b)
                          if (*(cache_0 + -3) < c_b)
                            if (*(cache_0 + pixel[14]) < c_b)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else if (*(cache_0 + pixel[5]) < c_b)
                      if (*(cache_0 + 3) < c_b)
                        if (*(cache_0 + pixel[6]) < c_b)
                          if (*(cache_0 + pixel[14]) < c_b)
                            goto success;
                          else
                            continue;
                        else if (*(cache_0 + -3) < c_b)
                          if (*(cache_0 + pixel[14]) < c_b)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else if (*(cache_0 + pixel[10]) < c_b)
                        if (*(cache_0 + pixel[11]) < c_b)
                          if (*(cache_0 + -3) < c_b)
                            if (*(cache_0 + pixel[14]) < c_b)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else if (*(cache_0 + pixel[11]) < c_b)
                      if (*(cache_0 + pixel[10]) > cb)
                        if (*(cache_0 + 3) < c_b)
                          if (*(cache_0 + -3) < c_b)
                            if (*(cache_0 + pixel[14]) < c_b)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else if (*(cache_0 + pixel[10]) < c_b)
                        if (*(cache_0 + pixel[14]) < c_b)
                          if (*(cache_0 + -3) < c_b)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else if (*(cache_0 + 3) < c_b)
                        if (*(cache_0 + pixel[14]) < c_b)
                          if (*(cache_0 + -3) < c_b)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else if (*(cache_0 + pixel[9]) < c_b)
                if (*(cache_0 + pixel[11]) < c_b)
                  if (*(cache_0 + pixel[1]) < c_b)
                    if (*(cache_0 + pixel[10]) < c_b)
                      if (*(cache_0 + -3) < c_b)
                        if (*(cache_0 + pixel[14]) < c_b)
                          if (*(cache_0 + pixel[15]) < c_b)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else if (*(cache_0 + pixel[7]) > cb)
              if (*(cache_0 + pixel[3]) > cb)
                if (*(cache_0 + pixel[10]) > cb)
                  if (*(cache_0 + 3) > cb)
                    if (*(cache_0 + pixel[5]) > cb)
                      if (*(cache_0 + pixel[6]) > cb)
                        if (*(cache_0 + pixel[9]) > cb)
                          if (*(cache_0 + pixel[11]) > cb)
                            if (*(cache_0 + -3) > cb)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else if (*(cache_0 + pixel[7]) < c_b)
              if (*(cache_0 + pixel[1]) < c_b)
                if (*(cache_0 + pixel[3]) < c_b)
                  if (*(cache_0 + 3) < c_b)
                    if (*(cache_0 + pixel[5]) < c_b)
                      if (*(cache_0 + pixel[6]) < c_b)
                        if (*(cache_0 + pixel[14]) < c_b)
                          if (*(cache_0 + pixel[15]) < c_b)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else
              continue;
          else if (*(cache_0 + -3) > cb)
            if (*(cache_0 + pixel[6]) > cb)
              if (*(cache_0 + pixel[11]) > cb)
                if (*(cache_0 + pixel[9]) > cb)
                  if (*(cache_0 + pixel[10]) > cb)
                    if (*(cache_0 + pixel[13]) > cb)
                      if (*(cache_0 + pixel[7]) > cb)
                        if (*(cache_0 + pixel[5]) > cb)
                          if (*(cache_0 + 3) > cb)
                            goto success;
                          else if (*(cache_0 + pixel[14]) > cb)
                            goto success;
                          else
                            continue;
                        else if (*(cache_0 + pixel[15]) > cb)
                          if (*(cache_0 + pixel[14]) > cb)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else if (*(cache_0 + pixel[3]) > cb)
                      if (*(cache_0 + 3) > cb)
                        if (*(cache_0 + pixel[5]) > cb)
                          if (*(cache_0 + pixel[7]) > cb)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else
              continue;
          else
            continue;
        else if (*(cache_0 + pixel[8]) < c_b)
          if (*(cache_0 + 3) > cb)
            if (*(cache_0 + -3) < c_b)
              if (*(cache_0 + pixel[10]) < c_b)
                if (*(cache_0 + pixel[14]) < c_b)
                  if (*(cache_0 + pixel[15]) < c_b)
                    if (*(cache_0 + pixel[13]) < c_b)
                      if (*(cache_0 + pixel[1]) < c_b)
                        if (*(cache_0 + pixel[11]) < c_b)
                          if (*(cache_0 + pixel[9]) > cb)
                            if (*(cache_0 + pixel[2]) < c_b)
                              if (*(cache_0 + pixel[3]) < c_b)
                                goto success;
                              else
                                continue;
                            else
                              continue;
                          else if (*(cache_0 + pixel[9]) < c_b)
                            goto success;
                          else if (*(cache_0 + pixel[3]) < c_b)
                            if (*(cache_0 + pixel[2]) < c_b)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else if (*(cache_0 + pixel[7]) < c_b)
                        if (*(cache_0 + pixel[9]) < c_b)
                          if (*(cache_0 + pixel[11]) < c_b)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else if (*(cache_0 + pixel[5]) < c_b)
                    if (*(cache_0 + pixel[6]) < c_b)
                      if (*(cache_0 + pixel[7]) < c_b)
                        if (*(cache_0 + pixel[9]) < c_b)
                          if (*(cache_0 + pixel[11]) < c_b)
                            if (*(cache_0 + pixel[13]) < c_b)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else
              continue;
          else if (*(cache_0 + 3) < c_b)
            if (*(cache_0 + pixel[2]) > cb)
              if (*(cache_0 + pixel[10]) < c_b)
                if (*(cache_0 + -3) < c_b)
                  if (*(cache_0 + pixel[11]) < c_b)
                    if (*(cache_0 + pixel[9]) < c_b)
                      if (*(cache_0 + pixel[13]) < c_b)
                        if (*(cache_0 + pixel[14]) < c_b)
                          if (*(cache_0 + pixel[7]) < c_b)
                            if (*(cache_0 + pixel[15]) > cb)
                              if (*(cache_0 + pixel[5]) < c_b)
                                if (*(cache_0 + pixel[6]) < c_b)
                                  goto success;
                                else
                                  continue;
                              else
                                continue;
                            else if (*(cache_0 + pixel[15]) < c_b)
                              goto success;
                            else if (*(cache_0 + pixel[6]) < c_b)
                              if (*(cache_0 + pixel[5]) < c_b)
                                goto success;
                              else
                                continue;
                            else
                              continue;
                          else if (*(cache_0 + pixel[1]) < c_b)
                            if (*(cache_0 + pixel[15]) < c_b)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else if (*(cache_0 + pixel[5]) < c_b)
                          if (*(cache_0 + pixel[6]) < c_b)
                            if (*(cache_0 + pixel[7]) < c_b)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else if (*(cache_0 + pixel[3]) < c_b)
                        if (*(cache_0 + pixel[5]) < c_b)
                          if (*(cache_0 + pixel[6]) < c_b)
                            if (*(cache_0 + pixel[7]) < c_b)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else if (*(cache_0 + pixel[2]) < c_b)
              if (*(cache_0 + pixel[6]) > cb)
                if (*(cache_0 + pixel[13]) < c_b)
                  if (*(cache_0 + pixel[14]) < c_b)
                    if (*(cache_0 + pixel[15]) < c_b)
                      if (*(cache_0 + -3) < c_b)
                        if (*(cache_0 + pixel[1]) < c_b)
                          if (*(cache_0 + pixel[3]) < c_b)
                            if (*(cache_0 + pixel[11]) < c_b)
                              goto success;
                            else if (*(cache_0 + pixel[5]) < c_b)
                              goto success;
                            else
                              continue;
                          else if (*(cache_0 + pixel[9]) < c_b)
                            if (*(cache_0 + pixel[10]) < c_b)
                              if (*(cache_0 + pixel[11]) < c_b)
                                goto success;
                              else
                                continue;
                            else
                              continue;
                          else
                            continue;
                        else if (*(cache_0 + pixel[7]) < c_b)
                          if (*(cache_0 + pixel[9]) < c_b)
                            if (*(cache_0 + pixel[10]) < c_b)
                              if (*(cache_0 + pixel[11]) < c_b)
                                goto success;
                              else
                                continue;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else if (*(cache_0 + pixel[6]) < c_b)
                if (*(cache_0 + pixel[3]) > cb)
                  if (*(cache_0 + pixel[9]) < c_b)
                    if (*(cache_0 + pixel[10]) < c_b)
                      if (*(cache_0 + pixel[11]) < c_b)
                        if (*(cache_0 + -3) < c_b)
                          if (*(cache_0 + pixel[13]) < c_b)
                            if (*(cache_0 + pixel[7]) < c_b)
                              if (*(cache_0 + pixel[5]) < c_b)
                                goto success;
                              else if (*(cache_0 + pixel[14]) < c_b)
                                if (*(cache_0 + pixel[15]) < c_b)
                                  goto success;
                                else
                                  continue;
                              else
                                continue;
                            else if (*(cache_0 + pixel[1]) < c_b)
                              if (*(cache_0 + pixel[14]) < c_b)
                                if (*(cache_0 + pixel[15]) < c_b)
                                  goto success;
                                else
                                  continue;
                              else
                                continue;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else if (*(cache_0 + pixel[3]) < c_b)
                  if (*(cache_0 + pixel[5]) > cb)
                    if (*(cache_0 + pixel[11]) < c_b)
                      if (*(cache_0 + -3) < c_b)
                        if (*(cache_0 + pixel[13]) < c_b)
                          if (*(cache_0 + pixel[14]) < c_b)
                            if (*(cache_0 + pixel[15]) < c_b)
                              if (*(cache_0 + pixel[1]) < c_b)
                                goto success;
                              else if (*(cache_0 + pixel[7]) < c_b)
                                if (*(cache_0 + pixel[9]) < c_b)
                                  if (*(cache_0 + pixel[10]) < c_b)
                                    goto success;
                                  else
                                    continue;
                                else
                                  continue;
                              else
                                continue;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else if (*(cache_0 + pixel[5]) < c_b)
                    if (*(cache_0 + pixel[7]) > cb)
                      if (*(cache_0 + pixel[1]) < c_b)
                        if (*(cache_0 + pixel[13]) < c_b)
                          if (*(cache_0 + pixel[14]) < c_b)
                            if (*(cache_0 + pixel[15]) < c_b)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else if (*(cache_0 + pixel[7]) < c_b)
                      if (*(cache_0 + pixel[1]) > cb)
                        if (*(cache_0 + pixel[9]) < c_b)
                          if (*(cache_0 + pixel[10]) < c_b)
                            if (*(cache_0 + pixel[11]) < c_b)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else if (*(cache_0 + pixel[1]) < c_b)
                        if (*(cache_0 + pixel[9]) < c_b)
                          goto success;
                        else if (*(cache_0 + pixel[15]) < c_b)
                          goto success;
                        else
                          continue;
                      else if (*(cache_0 + pixel[11]) < c_b)
                        if (*(cache_0 + pixel[10]) < c_b)
                          if (*(cache_0 + pixel[9]) < c_b)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else if (*(cache_0 + pixel[13]) < c_b)
                      if (*(cache_0 + pixel[15]) < c_b)
                        if (*(cache_0 + pixel[14]) < c_b)
                          if (*(cache_0 + pixel[1]) < c_b)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else if (*(cache_0 + -3) < c_b)
                    if (*(cache_0 + pixel[14]) < c_b)
                      if (*(cache_0 + pixel[11]) < c_b)
                        if (*(cache_0 + pixel[13]) < c_b)
                          if (*(cache_0 + pixel[15]) < c_b)
                            if (*(cache_0 + pixel[1]) > cb)
                              if (*(cache_0 + pixel[7]) < c_b)
                                if (*(cache_0 + pixel[9]) < c_b)
                                  if (*(cache_0 + pixel[10]) < c_b)
                                    goto success;
                                  else
                                    continue;
                                else
                                  continue;
                              else
                                continue;
                            else if (*(cache_0 + pixel[1]) < c_b)
                              goto success;
                            else if (*(cache_0 + pixel[9]) < c_b)
                              if (*(cache_0 + pixel[7]) < c_b)
                                if (*(cache_0 + pixel[10]) < c_b)
                                  goto success;
                                else
                                  continue;
                              else
                                continue;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else if (*(cache_0 + pixel[11]) < c_b)
                  if (*(cache_0 + pixel[13]) < c_b)
                    if (*(cache_0 + pixel[10]) < c_b)
                      if (*(cache_0 + pixel[9]) < c_b)
                        if (*(cache_0 + -3) < c_b)
                          if (*(cache_0 + pixel[7]) > cb)
                            if (*(cache_0 + pixel[1]) < c_b)
                              if (*(cache_0 + pixel[14]) < c_b)
                                if (*(cache_0 + pixel[15]) < c_b)
                                  goto success;
                                else
                                  continue;
                              else
                                continue;
                            else
                              continue;
                          else if (*(cache_0 + pixel[7]) < c_b)
                            if (*(cache_0 + pixel[5]) < c_b)
                              goto success;
                            else if (*(cache_0 + pixel[14]) < c_b)
                              if (*(cache_0 + pixel[15]) < c_b)
                                goto success;
                              else
                                continue;
                            else
                              continue;
                          else if (*(cache_0 + pixel[15]) < c_b)
                            if (*(cache_0 + pixel[1]) < c_b)
                              if (*(cache_0 + pixel[14]) < c_b)
                                goto success;
                              else
                                continue;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else if (*(cache_0 + -3) < c_b)
                if (*(cache_0 + pixel[14]) < c_b)
                  if (*(cache_0 + pixel[15]) < c_b)
                    if (*(cache_0 + pixel[13]) < c_b)
                      if (*(cache_0 + pixel[11]) > cb)
                        if (*(cache_0 + pixel[1]) < c_b)
                          if (*(cache_0 + pixel[3]) < c_b)
                            if (*(cache_0 + pixel[5]) < c_b)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else if (*(cache_0 + pixel[11]) < c_b)
                        if (*(cache_0 + pixel[1]) > cb)
                          if (*(cache_0 + pixel[7]) < c_b)
                            if (*(cache_0 + pixel[9]) < c_b)
                              if (*(cache_0 + pixel[10]) < c_b)
                                goto success;
                              else
                                continue;
                            else
                              continue;
                          else
                            continue;
                        else if (*(cache_0 + pixel[1]) < c_b)
                          if (*(cache_0 + pixel[3]) > cb)
                            if (*(cache_0 + pixel[9]) < c_b)
                              if (*(cache_0 + pixel[10]) < c_b)
                                goto success;
                              else
                                continue;
                            else
                              continue;
                          else if (*(cache_0 + pixel[3]) < c_b)
                            goto success;
                          else if (*(cache_0 + pixel[10]) < c_b)
                            if (*(cache_0 + pixel[9]) < c_b)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else if (*(cache_0 + pixel[7]) < c_b)
                          if (*(cache_0 + pixel[10]) < c_b)
                            if (*(cache_0 + pixel[9]) < c_b)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else if (*(cache_0 + pixel[5]) < c_b)
                        if (*(cache_0 + pixel[3]) < c_b)
                          if (*(cache_0 + pixel[1]) < c_b)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else if (*(cache_0 + pixel[11]) < c_b)
              if (*(cache_0 + pixel[10]) < c_b)
                if (*(cache_0 + -3) < c_b)
                  if (*(cache_0 + pixel[9]) < c_b)
                    if (*(cache_0 + pixel[13]) > cb)
                      if (*(cache_0 + pixel[3]) < c_b)
                        if (*(cache_0 + pixel[5]) < c_b)
                          if (*(cache_0 + pixel[6]) < c_b)
                            if (*(cache_0 + pixel[7]) < c_b)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else if (*(cache_0 + pixel[13]) < c_b)
                      if (*(cache_0 + pixel[7]) < c_b)
                        if (*(cache_0 + pixel[6]) < c_b)
                          if (*(cache_0 + pixel[5]) < c_b)
                            goto success;
                          else if (*(cache_0 + pixel[14]) < c_b)
                            if (*(cache_0 + pixel[15]) < c_b)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else if (*(cache_0 + pixel[14]) < c_b)
                          if (*(cache_0 + pixel[15]) < c_b)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else if (*(cache_0 + pixel[1]) < c_b)
                        if (*(cache_0 + pixel[14]) < c_b)
                          if (*(cache_0 + pixel[15]) < c_b)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else if (*(cache_0 + pixel[3]) < c_b)
                      if (*(cache_0 + pixel[6]) < c_b)
                        if (*(cache_0 + pixel[7]) < c_b)
                          if (*(cache_0 + pixel[5]) < c_b)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else
              continue;
          else if (*(cache_0 + -3) < c_b)
            if (*(cache_0 + pixel[10]) < c_b)
              if (*(cache_0 + pixel[14]) < c_b)
                if (*(cache_0 + pixel[11]) < c_b)
                  if (*(cache_0 + pixel[13]) < c_b)
                    if (*(cache_0 + pixel[15]) < c_b)
                      if (*(cache_0 + pixel[9]) > cb)
                        if (*(cache_0 + pixel[1]) < c_b)
                          if (*(cache_0 + pixel[2]) < c_b)
                            if (*(cache_0 + pixel[3]) < c_b)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else if (*(cache_0 + pixel[9]) < c_b)
                        if (*(cache_0 + pixel[1]) < c_b)
                          goto success;
                        else if (*(cache_0 + pixel[7]) < c_b)
                          goto success;
                        else
                          continue;
                      else if (*(cache_0 + pixel[3]) < c_b)
                        if (*(cache_0 + pixel[2]) < c_b)
                          if (*(cache_0 + pixel[1]) < c_b)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else if (*(cache_0 + pixel[5]) < c_b)
                      if (*(cache_0 + pixel[6]) < c_b)
                        if (*(cache_0 + pixel[7]) < c_b)
                          if (*(cache_0 + pixel[9]) < c_b)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else
              continue;
          else
            continue;
        else if (*(cache_0 + pixel[2]) < c_b)
          if (*(cache_0 + -3) > cb)
            if (*(cache_0 + pixel[6]) < c_b)
              if (*(cache_0 + pixel[14]) < c_b)
                if (*(cache_0 + pixel[7]) > cb)
                  if (*(cache_0 + pixel[1]) < c_b)
                    if (*(cache_0 + pixel[3]) < c_b)
                      if (*(cache_0 + 3) < c_b)
                        if (*(cache_0 + pixel[5]) < c_b)
                          if (*(cache_0 + pixel[13]) < c_b)
                            if (*(cache_0 + pixel[15]) < c_b)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else if (*(cache_0 + pixel[7]) < c_b)
                  if (*(cache_0 + 3) < c_b)
                    if (*(cache_0 + pixel[5]) < c_b)
                      if (*(cache_0 + pixel[1]) < c_b)
                        if (*(cache_0 + pixel[3]) < c_b)
                          if (*(cache_0 + pixel[15]) < c_b)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else if (*(cache_0 + pixel[13]) < c_b)
                  if (*(cache_0 + pixel[1]) < c_b)
                    if (*(cache_0 + pixel[3]) < c_b)
                      if (*(cache_0 + 3) < c_b)
                        if (*(cache_0 + pixel[5]) < c_b)
                          if (*(cache_0 + pixel[15]) < c_b)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else
              continue;
          else if (*(cache_0 + -3) < c_b)
            if (*(cache_0 + pixel[3]) > cb)
              if (*(cache_0 + pixel[9]) < c_b)
                if (*(cache_0 + pixel[11]) < c_b)
                  if (*(cache_0 + pixel[14]) < c_b)
                    if (*(cache_0 + pixel[13]) < c_b)
                      if (*(cache_0 + pixel[15]) < c_b)
                        if (*(cache_0 + pixel[1]) < c_b)
                          if (*(cache_0 + pixel[10]) < c_b)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else if (*(cache_0 + pixel[3]) < c_b)
              if (*(cache_0 + pixel[14]) < c_b)
                if (*(cache_0 + 3) > cb)
                  if (*(cache_0 + pixel[10]) < c_b)
                    if (*(cache_0 + pixel[15]) < c_b)
                      if (*(cache_0 + pixel[1]) < c_b)
                        if (*(cache_0 + pixel[11]) < c_b)
                          if (*(cache_0 + pixel[13]) < c_b)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else if (*(cache_0 + 3) < c_b)
                  if (*(cache_0 + pixel[15]) < c_b)
                    if (*(cache_0 + pixel[1]) < c_b)
                      if (*(cache_0 + pixel[13]) > cb)
                        if (*(cache_0 + pixel[5]) < c_b)
                          if (*(cache_0 + pixel[6]) < c_b)
                            if (*(cache_0 + pixel[7]) < c_b)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else if (*(cache_0 + pixel[13]) < c_b)
                        if (*(cache_0 + pixel[5]) < c_b)
                          goto success;
                        else if (*(cache_0 + pixel[11]) < c_b)
                          goto success;
                        else
                          continue;
                      else if (*(cache_0 + pixel[7]) < c_b)
                        if (*(cache_0 + pixel[6]) < c_b)
                          if (*(cache_0 + pixel[5]) < c_b)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else if (*(cache_0 + pixel[10]) < c_b)
                  if (*(cache_0 + pixel[11]) < c_b)
                    if (*(cache_0 + pixel[15]) < c_b)
                      if (*(cache_0 + pixel[13]) < c_b)
                        if (*(cache_0 + pixel[1]) < c_b)
                          goto success;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else if (*(cache_0 + pixel[9]) < c_b)
              if (*(cache_0 + pixel[10]) < c_b)
                if (*(cache_0 + pixel[14]) < c_b)
                  if (*(cache_0 + pixel[11]) < c_b)
                    if (*(cache_0 + pixel[15]) < c_b)
                      if (*(cache_0 + pixel[1]) < c_b)
                        if (*(cache_0 + pixel[13]) < c_b)
                          goto success;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else
              continue;
          else if (*(cache_0 + pixel[6]) < c_b)
            if (*(cache_0 + pixel[14]) < c_b)
              if (*(cache_0 + 3) < c_b)
                if (*(cache_0 + pixel[13]) > cb)
                  if (*(cache_0 + pixel[7]) < c_b)
                    if (*(cache_0 + pixel[3]) < c_b)
                      if (*(cache_0 + pixel[1]) < c_b)
                        if (*(cache_0 + pixel[5]) < c_b)
                          if (*(cache_0 + pixel[15]) < c_b)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else if (*(cache_0 + pixel[13]) < c_b)
                  if (*(cache_0 + pixel[5]) < c_b)
                    if (*(cache_0 + pixel[15]) < c_b)
                      if (*(cache_0 + pixel[1]) < c_b)
                        if (*(cache_0 + pixel[3]) < c_b)
                          goto success;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else if (*(cache_0 + pixel[7]) < c_b)
                  if (*(cache_0 + pixel[15]) < c_b)
                    if (*(cache_0 + pixel[3]) < c_b)
                      if (*(cache_0 + pixel[5]) < c_b)
                        if (*(cache_0 + pixel[1]) < c_b)
                          goto success;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else
              continue;
          else
            continue;
        else
          continue;
      else if (*(cache_0 + pixel[8]) > cb)
        if (*(cache_0 + pixel[10]) > cb)
          if (*(cache_0 + 3) > cb)
            if (*(cache_0 + pixel[2]) > cb)
              if (*(cache_0 + pixel[6]) > cb)
                if (*(cache_0 + pixel[7]) > cb)
                  if (*(cache_0 + pixel[11]) > cb)
                    if (*(cache_0 + pixel[9]) > cb)
                      if (*(cache_0 + pixel[5]) > cb)
                        if (*(cache_0 + pixel[3]) > cb)
                          goto success;
                        else if (*(cache_0 + pixel[3]) < c_b)
                          if (*(cache_0 + -3) > cb)
                            if (*(cache_0 + pixel[13]) > cb)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else if (*(cache_0 + pixel[13]) > cb)
                          if (*(cache_0 + -3) > cb)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else if (*(cache_0 + pixel[5]) < c_b)
                        if (*(cache_0 + -3) > cb)
                          if (*(cache_0 + pixel[13]) > cb)
                            if (*(cache_0 + pixel[14]) > cb)
                              if (*(cache_0 + pixel[15]) > cb)
                                goto success;
                              else
                                continue;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else if (*(cache_0 + pixel[15]) > cb)
                        if (*(cache_0 + pixel[14]) > cb)
                          if (*(cache_0 + -3) > cb)
                            if (*(cache_0 + pixel[13]) > cb)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else if (*(cache_0 + pixel[1]) > cb)
                    if (*(cache_0 + pixel[3]) > cb)
                      if (*(cache_0 + pixel[5]) > cb)
                        if (*(cache_0 + pixel[9]) > cb)
                          goto success;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else if (*(cache_0 + pixel[2]) < c_b)
              if (*(cache_0 + pixel[11]) > cb)
                if (*(cache_0 + -3) > cb)
                  if (*(cache_0 + pixel[9]) > cb)
                    if (*(cache_0 + pixel[6]) > cb)
                      if (*(cache_0 + pixel[7]) > cb)
                        if (*(cache_0 + pixel[13]) > cb)
                          if (*(cache_0 + pixel[5]) > cb)
                            goto success;
                          else if (*(cache_0 + pixel[14]) > cb)
                            if (*(cache_0 + pixel[15]) > cb)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else if (*(cache_0 + pixel[3]) > cb)
                          if (*(cache_0 + pixel[5]) > cb)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else if (*(cache_0 + -3) > cb)
              if (*(cache_0 + pixel[6]) > cb)
                if (*(cache_0 + pixel[11]) > cb)
                  if (*(cache_0 + pixel[13]) > cb)
                    if (*(cache_0 + pixel[7]) > cb)
                      if (*(cache_0 + pixel[9]) > cb)
                        if (*(cache_0 + pixel[5]) > cb)
                          goto success;
                        else if (*(cache_0 + pixel[5]) < c_b)
                          if (*(cache_0 + pixel[14]) > cb)
                            if (*(cache_0 + pixel[15]) > cb)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else if (*(cache_0 + pixel[15]) > cb)
                          if (*(cache_0 + pixel[14]) > cb)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else if (*(cache_0 + pixel[13]) < c_b)
                    if (*(cache_0 + pixel[3]) > cb)
                      if (*(cache_0 + pixel[5]) > cb)
                        if (*(cache_0 + pixel[7]) > cb)
                          if (*(cache_0 + pixel[9]) > cb)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else if (*(cache_0 + pixel[3]) > cb)
                    if (*(cache_0 + pixel[7]) > cb)
                      if (*(cache_0 + pixel[9]) > cb)
                        if (*(cache_0 + pixel[5]) > cb)
                          goto success;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else
              continue;
          else if (*(cache_0 + 3) < c_b)
            if (*(cache_0 + pixel[6]) > cb)
              if (*(cache_0 + pixel[14]) > cb)
                if (*(cache_0 + pixel[13]) > cb)
                  if (*(cache_0 + pixel[7]) > cb)
                    if (*(cache_0 + pixel[15]) > cb)
                      if (*(cache_0 + pixel[9]) > cb)
                        if (*(cache_0 + pixel[11]) > cb)
                          if (*(cache_0 + -3) > cb)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else if (*(cache_0 + pixel[5]) > cb)
                      if (*(cache_0 + pixel[9]) > cb)
                        if (*(cache_0 + pixel[11]) > cb)
                          if (*(cache_0 + -3) > cb)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else
              continue;
          else if (*(cache_0 + pixel[14]) > cb)
            if (*(cache_0 + pixel[6]) > cb)
              if (*(cache_0 + -3) > cb)
                if (*(cache_0 + pixel[5]) > cb)
                  if (*(cache_0 + pixel[11]) > cb)
                    if (*(cache_0 + pixel[9]) > cb)
                      if (*(cache_0 + pixel[7]) > cb)
                        if (*(cache_0 + pixel[13]) > cb)
                          goto success;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else if (*(cache_0 + pixel[5]) < c_b)
                  if (*(cache_0 + pixel[15]) > cb)
                    if (*(cache_0 + pixel[7]) > cb)
                      if (*(cache_0 + pixel[9]) > cb)
                        if (*(cache_0 + pixel[11]) > cb)
                          if (*(cache_0 + pixel[13]) > cb)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else if (*(cache_0 + pixel[15]) > cb)
                  if (*(cache_0 + pixel[11]) > cb)
                    if (*(cache_0 + pixel[9]) > cb)
                      if (*(cache_0 + pixel[13]) > cb)
                        if (*(cache_0 + pixel[7]) > cb)
                          goto success;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else
              continue;
          else
            continue;
        else
          continue;
      else if (*(cache_0 + pixel[8]) < c_b)
        if (*(cache_0 + pixel[10]) < c_b)
          if (*(cache_0 + 3) > cb)
            if (*(cache_0 + pixel[14]) < c_b)
              if (*(cache_0 + pixel[6]) < c_b)
                if (*(cache_0 + -3) < c_b)
                  if (*(cache_0 + pixel[9]) < c_b)
                    if (*(cache_0 + pixel[11]) < c_b)
                      if (*(cache_0 + pixel[15]) < c_b)
                        if (*(cache_0 + pixel[13]) < c_b)
                          if (*(cache_0 + pixel[7]) < c_b)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else if (*(cache_0 + pixel[5]) < c_b)
                        if (*(cache_0 + pixel[7]) < c_b)
                          if (*(cache_0 + pixel[13]) < c_b)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else
              continue;
          else if (*(cache_0 + 3) < c_b)
            if (*(cache_0 + pixel[6]) < c_b)
              if (*(cache_0 + -3) > cb)
                if (*(cache_0 + pixel[2]) < c_b)
                  if (*(cache_0 + pixel[1]) > cb)
                    if (*(cache_0 + pixel[3]) < c_b)
                      if (*(cache_0 + pixel[5]) < c_b)
                        if (*(cache_0 + pixel[7]) < c_b)
                          if (*(cache_0 + pixel[9]) < c_b)
                            if (*(cache_0 + pixel[11]) < c_b)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else if (*(cache_0 + pixel[1]) < c_b)
                    if (*(cache_0 + pixel[5]) < c_b)
                      if (*(cache_0 + pixel[9]) < c_b)
                        if (*(cache_0 + pixel[3]) < c_b)
                          if (*(cache_0 + pixel[7]) < c_b)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else if (*(cache_0 + pixel[11]) < c_b)
                    if (*(cache_0 + pixel[3]) < c_b)
                      if (*(cache_0 + pixel[5]) < c_b)
                        if (*(cache_0 + pixel[7]) < c_b)
                          if (*(cache_0 + pixel[9]) < c_b)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else if (*(cache_0 + -3) < c_b)
                if (*(cache_0 + pixel[7]) < c_b)
                  if (*(cache_0 + pixel[11]) > cb)
                    if (*(cache_0 + pixel[1]) < c_b)
                      if (*(cache_0 + pixel[2]) < c_b)
                        if (*(cache_0 + pixel[3]) < c_b)
                          if (*(cache_0 + pixel[5]) < c_b)
                            if (*(cache_0 + pixel[9]) < c_b)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else if (*(cache_0 + pixel[11]) < c_b)
                    if (*(cache_0 + pixel[9]) < c_b)
                      if (*(cache_0 + pixel[5]) > cb)
                        if (*(cache_0 + pixel[13]) < c_b)
                          if (*(cache_0 + pixel[14]) < c_b)
                            if (*(cache_0 + pixel[15]) < c_b)
                              goto success;
                            else
                              continue;
                          else
                            continue;
                        else
                          continue;
                      else if (*(cache_0 + pixel[5]) < c_b)
                        if (*(cache_0 + pixel[13]) < c_b)
                          goto success;
                        else if (*(cache_0 + pixel[3]) < c_b)
                          goto success;
                        else
                          continue;
                      else if (*(cache_0 + pixel[15]) < c_b)
                        if (*(cache_0 + pixel[14]) < c_b)
                          if (*(cache_0 + pixel[13]) < c_b)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else if (*(cache_0 + pixel[1]) < c_b)
                    if (*(cache_0 + pixel[2]) < c_b)
                      if (*(cache_0 + pixel[9]) < c_b)
                        if (*(cache_0 + pixel[3]) < c_b)
                          if (*(cache_0 + pixel[5]) < c_b)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else if (*(cache_0 + pixel[2]) < c_b)
                if (*(cache_0 + pixel[1]) < c_b)
                  if (*(cache_0 + pixel[3]) < c_b)
                    if (*(cache_0 + pixel[7]) < c_b)
                      if (*(cache_0 + pixel[9]) < c_b)
                        if (*(cache_0 + pixel[5]) < c_b)
                          goto success;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else if (*(cache_0 + pixel[11]) < c_b)
                  if (*(cache_0 + pixel[3]) < c_b)
                    if (*(cache_0 + pixel[5]) < c_b)
                      if (*(cache_0 + pixel[7]) < c_b)
                        if (*(cache_0 + pixel[9]) < c_b)
                          goto success;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else
              continue;
          else if (*(cache_0 + pixel[14]) < c_b)
            if (*(cache_0 + pixel[6]) < c_b)
              if (*(cache_0 + -3) < c_b)
                if (*(cache_0 + pixel[5]) > cb)
                  if (*(cache_0 + pixel[9]) < c_b)
                    if (*(cache_0 + pixel[7]) < c_b)
                      if (*(cache_0 + pixel[11]) < c_b)
                        if (*(cache_0 + pixel[13]) < c_b)
                          if (*(cache_0 + pixel[15]) < c_b)
                            goto success;
                          else
                            continue;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else if (*(cache_0 + pixel[5]) < c_b)
                  if (*(cache_0 + pixel[13]) < c_b)
                    if (*(cache_0 + pixel[11]) < c_b)
                      if (*(cache_0 + pixel[7]) < c_b)
                        if (*(cache_0 + pixel[9]) < c_b)
                          goto success;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else if (*(cache_0 + pixel[15]) < c_b)
                  if (*(cache_0 + pixel[13]) < c_b)
                    if (*(cache_0 + pixel[7]) < c_b)
                      if (*(cache_0 + pixel[9]) < c_b)
                        if (*(cache_0 + pixel[11]) < c_b)
                          goto success;
                        else
                          continue;
                      else
                        continue;
                    else
                      continue;
                  else
                    continue;
                else
                  continue;
              else
                continue;
            else
              continue;
          else
            continue;
        else
          continue;
      else
        continue;

      success: corners->push_back(Eigen::Vector2i(cache_0 - line_min, y));
    }
  }
}

void FastDetector::NonMaxSuppression(const vector<Eigen::Vector2i> &corners, const vector<int> &scores, vector<Eigen::Vector2i> *nonmax_corners) {
  nonmax_corners->clear();
  nonmax_corners->reserve(corners.size());

  if (corners.size() < 1)
    return;

  // Find where each row begins
  // (the corners are output in raster scan order). A beginning of -1 signifies
  // that there are no corners on that row.
  int last_row = corners.back()(1);
  vector<int> row_start(last_row + 1, -1);

  int prev_row = -1;
  for (unsigned int i = 0; i < corners.size(); i++)
    if (corners[i](1) != prev_row) {
      row_start[corners[i](1)] = i;
      prev_row = corners[i](1);
    }

  // Point above points (roughly) to the pixel above the one of interest, if there
  // is a feature there.
  int point_above = 0;
  int point_below = 0;

  const int sz = static_cast<int>(corners.size());

  for (int i = 0; i < sz; i++) {
    int score = scores[i];
    Eigen::Vector2i pos(corners[i](0), corners[i](1));

    // Check left
    if (i > 0)
      if (corners[i - 1](0) == (pos(0) - 1) && corners[i - 1](1) == pos(1) && scores[i - 1] > score)
        continue;

    // Check right
    if (i < (sz - 1))
      if (corners[i + 1](0) == (pos(0) + 1) && corners[i - 1](1) == pos(1) && scores[i + 1] > score)
        continue;

    // Check above (if there is a valid row above)
    if (pos(1) != 0 && row_start[pos(1) - 1] != -1) {
      // Make sure that current point_above is one row above
      if (corners[point_above](1) < pos(1) - 1)
        point_above = row_start[pos(1) - 1];

      // Make point_above point to the first of the pixels above the current point, if it exists.
      for (; corners[point_above](1) < pos(1) && corners[point_above](0) < pos(0) - 1; point_above++) {
      }

      for (int i = point_above; corners[i](1) < pos(1) && corners[i](0) <= pos(0) + 1; i++) {
        int x = corners[i](0);
        if ((x == pos(0) - 1 || x == pos(0) || x == pos(0) + 1) && scores[i] > score)
          goto cont;
      }
    }

    // Check below (if there is anything below)
    if (pos(1) != last_row && row_start[pos(1) + 1] != -1 && point_below < sz) { // Nothing below
      if (corners[point_below](1) < pos(1) + 1)
        point_below = row_start[pos(1) + 1];

      // Make point below point to one of the pixels belowthe current point, if it exists.
      for (; point_below < sz && corners[point_below](1) == pos(1) + 1 && corners[point_below](0) < pos(0) - 1; point_below++) {
      }

      for (int i = point_below; i < sz && corners[i](1) == pos(1) + 1 && corners[i](0) <= pos(0) + 1; i++) {
        int x = corners[i](0);
        if ((x == pos(0) - 1 || x == pos(0) || x == pos(0) + 1) && scores[i] > score)
          goto cont;
      }
    }

    nonmax_corners->push_back(Eigen::Vector2i(corners[i](0), corners[i](1)));

    cont: ;
  }
}

void FastDetector::ComputeFastScore(const cv::Mat &src, const vector<Eigen::Vector2i> &corners, int threshold, vector<int> *scores) {
  int imgstep = src.step;

  int pointer_dir[16] = {
    0 + imgstep * 3,
    1 + imgstep * 3,
    2 + imgstep * 2,
    3 + imgstep * 1,
    3 + imgstep * 0,
    3 + imgstep * -1,
    2 + imgstep * -2,
    1 + imgstep * -3,
    0 + imgstep * -3,
    -1 + imgstep * -3,
    -2 + imgstep * -2,
    -3 + imgstep * -1,
    -3 + imgstep * 0,
    -3 + imgstep * 1,
    -2 + imgstep * 2,
    -1 + imgstep * 3,
  };

  scores->resize(corners.size());

  for (unsigned int i = 0; i < corners.size(); i++)
    (*scores)[i] = CornerScore(src, corners[i], pointer_dir, threshold);
}

int FastDetector::CornerScore(const cv::Mat &src, const Eigen::Vector2i &c, const int *pointer_dir, int threshold) {
  // The score for a positive feature is sum of the difference between the pixels
  // and the threshold if the difference is positive. Negative is similar.
  // The score is the max of those two.
  //
  // B = {x | x = points on the Bresenham circle around c}
  // Sp = { I(x) - t | x E B , I(x) - t > 0 }
  // Sn = { t - I(x) | x E B, t - I(x) > 0}
  //
  // Score = max sum(Sp), sum(Sn)

  const unsigned char* imp = src.data + c(1) * src.step + c(0);

  int cb = *imp + threshold;
  int c_b = *imp - threshold;
  int sp = 0, sn = 0;

  for (int i = 0; i < 16; i++) {
    int p = *(imp + pointer_dir[i]);

    if (p > cb)
      sp += p - cb;
    else if (p < c_b)
      sn += c_b - p;
  }

  if (sp > sn)
    return sp;
  else
    return sn;
}

}  // namespace sdvl



