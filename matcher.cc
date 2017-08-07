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

#include "./matcher.h"
#include "./config.h"
#include "extra/utils.h"
#include "extra/timer.h"

using std::shared_ptr;
using std::vector;
using std::cerr;
using std::endl;

namespace sdvl {

Matcher::Matcher(int size) {
  patch_size_ = size;
  patch_ = new uint8_t[patch_size_*patch_size_];
  border_patch_ = new uint8_t[(patch_size_+2)*(patch_size_+2)];
}

Matcher::~Matcher() {
  delete patch_;
  delete border_patch_;
}

bool Matcher::SearchPoint(const shared_ptr<Frame> &frame, const shared_ptr<Feature> &feature,
                             double idepth, double idepth_std, bool fixed, Eigen::Vector2d *px, int *flevel) {
  Eigen::Matrix2d affine_matrix;
  double range, zmin, zmax;
  int slevel, level = feature->GetLevel();
  Eigen::Vector2d pxa, pxb;

  shared_ptr<Frame> ref_frame = feature->GetFrame();
  assert(ref_frame);

  SE3 pose = frame->GetPose() * ref_frame->GetPose().Inverse();

  // Check projection
  if (fixed) {
    zmin = 1.0/(idepth + 2.0*idepth_std);
    Eigen::Vector3d p3d_min = ref_frame->GetWorldPose() * (zmin*feature->GetVector());

    if (!frame->Project(p3d_min, &pxa))
      return false;
  } else {
    // Restrict epipolar search to a narrow depth range
    zmin = 1.0/(idepth + 2.0*idepth_std);
    zmax = 1.0/(std::max(idepth - 2.0*idepth_std, 0.00000001));

    // Get 2d epipolar line extremes
    Eigen::Vector3d p3d_min = ref_frame->GetWorldPose() * (zmin*feature->GetVector());
    Eigen::Vector3d p3d_max = ref_frame->GetWorldPose() * (zmax*feature->GetVector());

    if (!frame->Project(p3d_min, &pxa))
      return false;
    if (!frame->Project(p3d_max, &pxb))
      return false;
  }

  if (Config::UseORB())
    assert(feature->HasDescriptor());

  // Check margins in sublevel where feature was found to create patch
  if (!ref_frame->GetCamera()->IsInsideImage(feature->GetLevelPosition().cast<int>(), patch_size_/2+2, level))
    return false;

  // Warp affine and create path
  cv::Mat& img = ref_frame->GetPyramid()[level];
  WarpMatrixAffine(frame->GetCamera(), feature->GetPosition(), feature->GetVector(), 1.0/idepth, pose, level, &affine_matrix);
  slevel = GetSearchLevel(affine_matrix);
  CreatePatch(affine_matrix, img, feature->GetPosition(), level, slevel, border_patch_, patch_);

  range = Config::SearchSize();
  for (int i=1; i<= slevel; i++)
    range *= 1.2;

  vector<int> indices;

  if (fixed) {
    // Search in feature
    Eigen::Vector2d cpos = *px;

    // Get features in range
    GetCornersInRange(frame, cpos, level, range, &indices);
  } else {
    // Get features close to epipolar line
    GetCornersInRange(frame, pxa, pxb, level, range, &indices);
  }

  if (!SearchFeatures(frame, indices, patch_, px, feature->GetDescriptor()))
    return false;

  // Subpixel refinament
  Eigen::Vector2d px_scaled(*px/(1 << slevel));
  if (AlignPatch(frame->GetPyramid()[slevel], border_patch_, patch_, &px_scaled)) {
    *px = px_scaled*(1 << slevel);
    *flevel = slevel;
    return true;
  }

  return false;
}

void Matcher::GetCornersInRange(const std::shared_ptr<Frame> &frame, const Eigen::Vector2d& pxa,
                                const Eigen::Vector2d& pxb, int level, double range, vector<int> *indices) {
  Eigen::Vector2d eline, vnormal;
  double dist, u, dx, dy;
  double normdist, xdiff, ydiff, vline;
  double range2 = range*range;
  int index, clevel, margin;

  if (Config::UseORB())
    margin = 4+Config::ORBSize()/2;
  else
    margin = 1+patch_size_/2;

  vector<Eigen::Vector3i>& ccorners = frame->GetCorners();

  // Epipolar line
  eline = pxa - pxb;
  eline.normalize();

  // Calculate margins
  vnormal << eline(1), -eline(0);
  normdist = pxa.dot(vnormal);
  xdiff = pxb(0)-pxa(0);
  ydiff = pxb(1)-pxa(1);
  vline = (xdiff)*(xdiff) + (ydiff)*(ydiff);

  // Check each corner
  index = 0;
  for (auto it = ccorners.begin(); it != ccorners.end(); it++, index++) {
    clevel = (*it)(2);

    // Same level or previous/next
    if (abs(clevel - level) > 1)
      continue;

    // Check borders
    if ((*it)(0)-margin < 0 || (*it)(1)-margin < 0)
      continue;

    if ((*it)(1)+margin >= frame->GetPyramid()[clevel].rows || (*it)(0)+margin >= frame->GetPyramid()[clevel].cols)
      continue;

    Eigen::Vector2d pos((*it)(0)*(1 << clevel), (*it)(1)*(1 << clevel));

    // Check distance from feature to epipolar line
    dist = normdist - pos.dot(vnormal);
    if (abs(dist) > range)
      continue;

    // Get the u parameter in the equation P = A+u(B-A)
    u = ((pos(0)-pxa(0))*xdiff+(pos(1)-pxa(1))*ydiff)/vline;
    if (u > 1) {
      // Check distance to b
      dx = pos(0)-pxb(0);
      dy = pos(1)-pxb(1);
      if ((dx*dx+dy*dy) > range2)
        continue;
    }

    if (u < 0) {
      // Check distance to a
      dx = pos(0)-pxa(0);
      dy = pos(1)-pxa(1);
      if ((dx*dx+dy*dy) > range2)
        continue;
    }

    indices->push_back(index);
  }
}

void Matcher::GetCornersInRange(const std::shared_ptr<Frame> &frame, const Eigen::Vector2d& cpos, 
                                int level, double range, vector<int> *indices) {
  double range2 = range*range;
  int index, clevel, margin;

  if (Config::UseORB())
    margin = 4+Config::ORBSize()/2;
  else
    margin = 1+patch_size_/2;

  vector<Eigen::Vector3i>& ccorners = frame->GetCorners();

  // Check each corner
  index = 0;
  for (auto it = ccorners.begin(); it != ccorners.end(); it++, index++) {
    clevel = (*it)(2);

    // Same level or previous/next
    if (abs(clevel - level) > 1)
      continue;

    // Check borders
    if ((*it)(0)-margin < 0 || (*it)(1)-margin < 0)
      continue;

    if ((*it)(1)+margin >= frame->GetPyramid()[clevel].rows || (*it)(0)+margin >= frame->GetPyramid()[clevel].cols)
      continue;

    Eigen::Vector2d pos((*it)(0)*(1 << clevel), (*it)(1)*(1 << clevel));

    // Reject features outside circle
    if ((cpos - pos).squaredNorm() > range2)
      continue;

    indices->push_back(index);
  }
}

bool Matcher::SearchFeatures(const shared_ptr<Frame> &frame, const vector<int> &indices, uint8_t* patch,
                             Eigen::Vector2d *px, const vector<uchar> &desc) {
  int sumA = 0, sumAA = 0, sumB, sumBB, sumAB;
  Eigen::Vector2d best_px;
  int index, threshold, best_score, score, level;
  Eigen::Vector3i corner;
  Eigen::Vector2i p2d;

  vector<vector<uchar>>& descriptors = frame->GetDescriptors();
  vector<Eigen::Vector3i>& corners = frame->GetCorners();

  if (Config::UseORB())
    threshold = MIN_ORB_THRESHOLD;
  else
    threshold = patch_size_*patch_size_*MAX_SSD_PER_PIXEL;

  best_score = threshold + 1;

  if (!Config::UseORB()) {
    // Compute ZMSSD with corners close to selected feature
    GetZMSSDScore(patch, &sumA, &sumAA);
  }

  // Check each corner
  for (auto it = indices.begin(); it != indices.end(); it++) {
    index = *it;
    corner = corners[index];
    p2d << corner(0), corner(1);
    level = corner(2);

    cv::Mat &cimg = frame->GetPyramid()[level];

    if (Config::UseORB()) {
      // Check if it already exists
      if (descriptors[index].empty()) {
        descriptors[index].resize(32);
        detector_.GetDescriptor(cimg, p2d, &descriptors[index]);
      }

      // Compare descriptors
      score = detector_.Distance(desc, descriptors[index]);
    } else {
      // Compare patch

      uint8_t* cur_patch = cimg.data + (corner(1)-patch_size_/2)*cimg.cols + (corner(0)-patch_size_/2);
      score = CompareZMSSDScore(patch, cur_patch, sumA, sumAA, cimg.cols, &sumB, &sumBB, &sumAB);
    }

    if (score < best_score) {
      best_score = score;
      best_px << p2d(0)*(1 << level), p2d(1)*(1 << level);
    }
  }

  if (best_score >= threshold)
      return false;

  *px = best_px;
  return true;
}

void Matcher::WarpMatrixAffine(Camera * camera, const Eigen::Vector2d &px, const Eigen::Vector3d &v,
                                    double depth, const SE3 &pose, int level, Eigen::Matrix2d *res_matrix) {
  const int half_size = 5;
  Eigen::Vector3d p3d, xyz_du, xyz_dv;
  Eigen::Vector2d px_cur, px_du, px_dv;

  p3d = v*depth;
  camera->Unproject(px + Eigen::Vector2d(half_size, 0)*(1 << level), &xyz_du);
  camera->Unproject(px + Eigen::Vector2d(0, half_size)*(1 << level), &xyz_dv);

  xyz_du *= p3d(2)/xyz_du(2);
  xyz_dv *= p3d(2)/xyz_dv(2);

  camera->Project(pose*(p3d), &px_cur);
  camera->Project(pose*(xyz_du), &px_du);
  camera->Project(pose*(xyz_dv), &px_dv);

  res_matrix->col(0) = (px_du - px_cur)/half_size;
  res_matrix->col(1) = (px_dv - px_cur)/half_size;
}

int Matcher::GetSearchLevel(const Eigen::Matrix2d &matrix) {
  int search_level = 0;
  double det = matrix.determinant();
  int max = Config::MaxFastLevels()-1;
  while (det > 3.0 && search_level < max) {
    search_level += 1;
    det *= 0.25;
  }
  return search_level;
}

void Matcher::CreatePatch(const Eigen::Matrix2d& matrix, const cv::Mat& img, const Eigen::Vector2d& px,
                               int level, int search_level, uint8_t* border_patch, uint8_t* patch) {
  Eigen::Vector2d px_patch, p;
  const int bpatch_size = patch_size_+2;  // Save border
  const int half_size = bpatch_size/2;
  const Eigen::Matrix2d matrix_inv = matrix.inverse();
  if (std::isnan(matrix_inv(0, 0))) {
    cerr << "[ERROR] Coudn't invert affine matrix" << endl;
    return;
  }

  // Perform the warp on both patches
  uint8_t* patch_ptr = patch;
  uint8_t* bpatch_ptr = border_patch;
  const Eigen::Vector2d px_pyr(px/(1 << level));
  for (int y=0; y < bpatch_size; y++) {
    for (int x=0; x < bpatch_size; x++, bpatch_ptr++) {
      px_patch << x-half_size, y-half_size;
      px_patch *= (1 << search_level);
      p = matrix_inv*px_patch + px_pyr;
      if (p(0) < 0 || p(1) < 0 || p(0) >= img.cols-1 || p(1) >= img.rows-1)
        *bpatch_ptr = 0;
      else
        *bpatch_ptr = (uint8_t) Interpolate8U(img, static_cast<float>(p(0)), static_cast<float>(p(1)));

      // Skip borders
      if (y >= 1 && y < bpatch_size-1 && x >= 1 && x < bpatch_size-1) {
        *patch_ptr = *bpatch_ptr;
        patch_ptr++;
      }
    }
  }
}

bool Matcher::AlignPatch(const cv::Mat& img, uint8_t* border_patch, uint8_t* patch, Eigen::Vector2d *px) {
  Eigen::Matrix3f H, Hinv;
  Eigen::Vector3f update;
  const int half_size = patch_size_/2;
  const int patch_area = patch_size_*patch_size_;
  bool converged = false;

  // Compute derivative of template and prepare inverse compositional
  float __attribute__((__aligned__(16))) patch_dx[patch_area];
  float __attribute__((__aligned__(16))) patch_dy[patch_area];

  H.setZero();
  update.setZero();

  // Compute gradient and hessian
  const int ref_step = patch_size_+2;
  float* it_dx = patch_dx;
  float* it_dy = patch_dy;
  for (int y=0; y < patch_size_; y++) {
    uint8_t* it = border_patch + (y+1)*ref_step + 1;
    for (int x=0; x < patch_size_; x++, it++, it_dx++, it_dy++) {
      Eigen::Vector3f J;
      J[0] = 0.5 * (it[1] - it[-1]);
      J[1] = 0.5 * (it[ref_step] - it[-ref_step]);
      J[2] = 1;
      *it_dx = J[0];
      *it_dy = J[1];
      H += J*J.transpose();
    }
  }
  Hinv = H.inverse();
  float mean_diff = 0;

  // Compute pixel location in new image:
  float u = px->x();
  float v = px->y();

  // Termination condition
  float min_update_squared = 0.03*0.03;
  int cur_step = img.step.p[0];
  for (int iter = 0; iter < Config::MaxAlignIts(); iter++) {
    int u_r = floor(u);
    int v_r = floor(v);
    if (u_r < half_size || v_r < half_size || u_r >= img.cols-half_size || v_r >= img.rows-half_size)
      break;

    if (std::isnan(u) || std::isnan(v))
      return false;

    // compute interpolation weights
    float subpix_x = u-u_r;
    float subpix_y = v-v_r;
    float wTL = (1.0-subpix_x)*(1.0-subpix_y);
    float wTR = subpix_x * (1.0-subpix_y);
    float wBL = (1.0-subpix_x)*subpix_y;
    float wBR = subpix_x * subpix_y;

    // loop through search_patch, interpolate
    uint8_t* it_ref = patch;
    float* it_ref_dx = patch_dx;
    float* it_ref_dy = patch_dy;
    Eigen::Vector3f Jres; Jres.setZero();
    for (int y=0; y < patch_size_; y++) {
      uint8_t* it = reinterpret_cast<uint8_t*>(img.data + (v_r+y-half_size)*cur_step + u_r-half_size);
      for (int x=0; x < patch_size_; x++, it++, it_ref++, it_ref_dx++, it_ref_dy++) {
        float search_pixel = wTL*it[0] + wTR*it[1] + wBL*it[cur_step] + wBR*it[cur_step+1];
        float res = search_pixel - *it_ref + mean_diff;
        Jres[0] -= res*(*it_ref_dx);
        Jres[1] -= res*(*it_ref_dy);
        Jres[2] -= res;
      }
    }

    update = Hinv * Jres;
    u += update[0];
    v += update[1];
    mean_diff += update[2];

    if (update[0]*update[0]+update[1]*update[1] < min_update_squared) {
      converged = true;
      break;
    }
  }

  *px << u, v;
  return converged;
}

void Matcher::GetZMSSDScore(uint8_t *patch, int *sumA, int *sumAA) {
  uint32_t sumA_uint = 0, sumAA_uint = 0;
  int parea = patch_size_*patch_size_;
  for (int r = 0; r < parea; r++) {
    uint8_t n = patch[r];
    sumA_uint += n;
    sumAA_uint += n*n;
  }
  *sumA = sumA_uint;
  *sumAA = sumAA_uint;
}

double Matcher::CompareZMSSDScore(uint8_t *ref_patch, uint8_t *patch, int sumA, int sumAA, int cols, int *sumB, int *sumBB, int *sumAB) {
  uint32_t sumB_uint = 0;
  uint32_t sumBB_uint = 0;
  uint32_t sumAB_uint = 0;
  for (int y=0, r=0; y < patch_size_; y++) {
    uint8_t* patch_ptr = patch + y*cols;
    for (int x=0; x < patch_size_; x++, r++) {
      const uint8_t pixel = patch_ptr[x];
      sumB_uint  += pixel;
      sumBB_uint += pixel*pixel;
      sumAB_uint += pixel * ref_patch[r];
    }
  }
  *sumB = sumB_uint;
  *sumBB = sumBB_uint;
  *sumAB = sumAB_uint;
  return sumAA - 2*(*sumAB) + (*sumBB) - (sumA*sumA - 2*sumA*(*sumB) + (*sumB)*(*sumB))/(patch_size_*patch_size_);
}

}  // namespace sdvl
