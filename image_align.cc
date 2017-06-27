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

#include "./image_align.h"
#include "./feature.h"
#include "./config.h"
#include "extra/se3.h"

using std::shared_ptr;
using std::vector;
using std::cout;
using std::cerr;
using std::endl;

namespace sdvl {

ImageAlign::ImageAlign() {
  stop_ = false;
  chi2_ = 1e10;
  frame1_ = nullptr;
  frame2_ = nullptr;
  error_ = 1e10;
}

ImageAlign::~ImageAlign() {
}

int ImageAlign::ComputePose(const shared_ptr<Frame> &frame1, const shared_ptr<Frame> &frame2, bool fast) {
  int size, path_area;

  frame1_ = frame1;
  frame2_ = frame2;

  assert(static_cast<int>(frame1_->GetPyramid().size()) >= Config::MaxAlignLevel());

  size = frame1_->GetFeatures().size();
  if (size == 0) {
    cerr << "[ERROR] No points to track!" << endl;
    return 0;
  }

  path_area = Config::AlignPatchSize()*Config::AlignPatchSize();
  patch_cache_ = cv::Mat(size, path_area, CV_32F);
  jacobian_cache_.resize(Eigen::NoChange, patch_cache_.rows*path_area);
  visible_fts_.resize(patch_cache_.rows, false);

  // Initial displacement between frames.
  SE3 current_se3 = frame2_->GetPose() * frame1_->GetPose().Inverse();

  for (int level=Config::MaxAlignLevel(); level >= Config::MinAlignLevel(); level--) {
    jacobian_cache_.setZero();
    Optimize(&current_se3, level);

    // High error in max level means frames are not close, skip other levels
    if (fast && error_ > 0.01) {
      error_ = 1e10;
      break;
    }
  }

  frame2_->SetPose(current_se3 * frame1_->GetPose());

  frame1_ = nullptr;
  frame2_ = nullptr;
  return n_meas_/path_area;
}

void ImageAlign::Optimize(SE3 *se3, int level) {
  Eigen::Matrix<double, 6, 1>  x;
  SE3 se3_bk = *se3;

  // Perform iterative estimation
  for (int i = 0; i < Config::MaxImgAlignIts(); i++) {
    H_.setZero();
    Jres_.setZero();

    // compute initial error
    n_meas_ = 0;
    double new_chi2 = ComputeResiduals(*se3, level, true, i == 0);
    if (n_meas_ == 0)
      stop_ = true;

    // Solve linear system
    x = H_.ldlt().solve(Jres_);
    if (static_cast<bool>(std::isnan(static_cast<double>(x[0])))) {
      // Matrix was singular and could not be computed
      stop_ = true;
    }

    // Check if error increased since last iteration
    if ((i > 0 && new_chi2 > chi2_) || stop_) {
      *se3 = se3_bk;  // rollback
      break;
    }

    // Update se3
    se3_bk = *se3;
    *se3 = (*se3) * SE3::Exp(-x);

    chi2_ = new_chi2;

    // Stop when converged
    error_ = AbsMax(x);
    if (error_ <= 1e-10)
      break;
  }
}

double ImageAlign::ComputeResiduals(const SE3 &se3, int level, bool linearize, bool patches) {
  const int half_patch = Config::AlignPatchSize()/2;
  const int path_area = Config::AlignPatchSize()*Config::AlignPatchSize();

  // Warp last img so that it aligns with the first image
  const cv::Mat& last_img = frame2_->GetPyramid().at(level);

  // Compute patches only the first time
  if (patches)
    PrecomputePatches(level);

  // Compute weights on first iteration
  const int stride = last_img.cols;
  const int border = half_patch+1;
  const float scale = 1.0f/(1 << level);
  const Eigen::Vector3d first_pos = frame1_->GetWorldPosition();
  float chi2 = 0.0;
  size_t counter = 0;
  vector<bool>::iterator vit = visible_fts_.begin();
  vector<shared_ptr<Feature>> &features = frame1_->GetFeatures();
  for (auto it=features.begin(); it != features.end(); it++, counter++, vit++) {
    shared_ptr<Feature> feature = *it;

    // check if point is within image
    if (!*vit)
      continue;

    assert(feature->GetPoint());
    if (feature->GetPoint()->ToDelete())
      continue;

    // Compute pixel location in last img
    const double depth((feature->GetPoint()->GetPosition() - first_pos).norm());
    const Eigen::Vector3d xyz_ref(feature->GetVector()*depth);
    const Eigen::Vector3d xyz_cur(se3 * xyz_ref);
    Eigen::Vector2d proj_xyz;
    frame2_->GetCamera()->Project(xyz_cur, &proj_xyz);
    const Eigen::Vector2d uv_last_pyr = proj_xyz*scale;
    const float u_cur = uv_last_pyr(0);
    const float v_cur = uv_last_pyr(1);
    const int u_last_i = floorf(u_cur);
    const int v_last_i = floorf(v_cur);

    // check if projection is within the image
    if (u_last_i < 0 || v_last_i < 0 || u_last_i-border < 0 || v_last_i-border < 0 || u_last_i+border >= last_img.cols || v_last_i+border >= last_img.rows)
      continue;

    // compute bilateral interpolation weights for the current image
    const float subpix_u_cur = u_cur-u_last_i;
    const float subpix_v_cur = v_cur-v_last_i;
    const float w_last_tl = (1.0-subpix_u_cur) * (1.0-subpix_v_cur);
    const float w_last_tr = subpix_u_cur * (1.0-subpix_v_cur);
    const float w_last_bl = (1.0-subpix_u_cur) * subpix_v_cur;
    const float w_last_br = subpix_u_cur * subpix_v_cur;
    float* patch_cache_ptr = reinterpret_cast<float*>(patch_cache_.data) + path_area*counter;
    size_t pixel_counter = 0;  // is used to compute the index of the cached jacobian
    for (int y=0; y < Config::AlignPatchSize(); y++) {
      uint8_t* last_img_ptr = reinterpret_cast<uint8_t*>(last_img.data + (v_last_i+y-half_patch)*stride + (u_last_i-half_patch));

      for (int x=0; x < Config::AlignPatchSize(); x++, pixel_counter++, last_img_ptr++, patch_cache_ptr++) {
        // compute residual
        const float intensity_cur = w_last_tl*last_img_ptr[0] + w_last_tr*last_img_ptr[1] + w_last_bl*last_img_ptr[stride] + w_last_br*last_img_ptr[stride+1];
        const float res = intensity_cur - (*patch_cache_ptr);

        float weight = 1.0;
        chi2 += res*res*weight;
        n_meas_++;

        if (linearize) {
          // compute Jacobian, weighted Hessian and weighted "steepest descend images" (times error)
          const Eigen::Matrix<double, 6, 1> J = jacobian_cache_.col(counter*path_area + pixel_counter);
          H_.noalias() += J*J.transpose()*weight;
          Jres_.noalias() -= J*res*weight;
        }
      }
    }
  }

  return chi2/n_meas_;
}

void ImageAlign::PrecomputePatches(int level) {
  const int half_patch = Config::AlignPatchSize()/2;
  const int path_area = Config::AlignPatchSize()*Config::AlignPatchSize();
  const int border = half_patch+1;
  const cv::Mat& first_img = frame1_->GetPyramid().at(level);
  const int stride = first_img.cols;
  const float scale = 1.0f/(1 << level);
  const Eigen::Vector3d first_pos = frame1_->GetWorldPosition();
  const double focal_length = frame1_->GetCamera()->GetFx();
  size_t counter = 0;
  Eigen::Matrix<double, 2, 6> frame_jac;
  vector<bool>::iterator vit = visible_fts_.begin();
  vector<shared_ptr<Feature>> &features = frame1_->GetFeatures();
  for (auto it=features.begin(); it != features.end(); it++, counter++, vit++) {
    shared_ptr<Feature> feature = *it;

    // Check if patch fits within image
    const float u_ref = feature->GetPosition()(0)*scale;
    const float v_ref = feature->GetPosition()(1)*scale;
    const int u_first_i = floorf(u_ref);
    const int v_first_i = floorf(v_ref);
    if (!feature->GetPoint() || feature->GetPoint()->ToDelete() || u_first_i-border < 0 || v_first_i-border < 0 || u_first_i+border >= first_img.cols || v_first_i+border >= first_img.rows)
      continue;
    *vit = true;

    // Get 3d point from camera
    const double depth((feature->GetPoint()->GetPosition() - first_pos).norm());
    const Eigen::Vector3d xyz_ref(feature->GetVector()*depth);

    // Evaluate projection jacobian
    Jacobian3DToPlane(xyz_ref, &frame_jac);

    // compute bilateral interpolation weights for reference image
    const float subpix_u_ref = u_ref-u_first_i;
    const float subpix_v_ref = v_ref-v_first_i;
    const float w_first_tl = (1.0-subpix_u_ref) * (1.0-subpix_v_ref);
    const float w_first_tr = subpix_u_ref * (1.0-subpix_v_ref);
    const float w_first_bl = (1.0-subpix_u_ref) * subpix_v_ref;
    const float w_first_br = subpix_u_ref * subpix_v_ref;
    size_t pixel_counter = 0;
    float* cache_ptr = reinterpret_cast<float*>(patch_cache_.data) + path_area*counter;
    for (int y=0; y < Config::AlignPatchSize(); y++) {
      uint8_t* first_img_ptr = reinterpret_cast<uint8_t*>(first_img.data + (v_first_i+y-half_patch)*stride + (u_first_i-half_patch));
      for (int x=0; x < Config::AlignPatchSize(); x++, first_img_ptr++, cache_ptr++, pixel_counter++) {
        // precompute interpolated reference patch color
        *cache_ptr = w_first_tl*first_img_ptr[0] + w_first_tr*first_img_ptr[1] + w_first_bl*first_img_ptr[stride] + w_first_br*first_img_ptr[stride+1];

        // we use the inverse compositional: thereby we can take the gradient always at the same position
        // get gradient of warped image (~gradient at warped position)
        float dx = 0.5f * ((w_first_tl*first_img_ptr[1] + w_first_tr*first_img_ptr[2] + w_first_bl*first_img_ptr[stride+1] + w_first_br*first_img_ptr[stride+2])
                          -(w_first_tl*first_img_ptr[-1] + w_first_tr*first_img_ptr[0] + w_first_bl*first_img_ptr[stride-1] + w_first_br*first_img_ptr[stride]));
        float dy = 0.5f * ((w_first_tl*first_img_ptr[stride] + w_first_tr*first_img_ptr[1+stride] + w_first_bl*first_img_ptr[stride*2] + w_first_br*first_img_ptr[stride*2+1])
                          -(w_first_tl*first_img_ptr[-stride] + w_first_tr*first_img_ptr[1-stride] + w_first_bl*first_img_ptr[0] + w_first_br*first_img_ptr[1]));

        // cache the jacobian
        jacobian_cache_.col(counter*path_area + pixel_counter) = (dx*frame_jac.row(0) + dy*frame_jac.row(1))*(focal_length / (1 << level));
      }
    }
  }
}

}  // namespace sdvl
