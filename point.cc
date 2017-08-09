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

#include "./point.h"
#include "./config.h"
#include "extra/utils.h"

using std::shared_ptr;

namespace sdvl {

int Point::counter_ = 0;

Point::Point() {
  id_ = counter_;
  last_frame_ = -1;
  last_ba_ = -1;
  n_failed_ = 0;
  n_successful_ = 0;
  delete_ = false;
  fixed_ = false;
  p3d_.Zero();

  counter_++;
}

Point::~Point() {
}

void Point::InitCandidate(const shared_ptr<Feature> &p, double depth) {
  feature_ = p;

  // Set default parameters
  a_ = 10;
  b_ = 10;
  rho_ = 1.0/depth;
  sigma2_ = 1.0;
  z_range_ = sqrt(sigma2_*36);

  // Frame info
  cos_alpha_ = 1.0;
  last_distance_ = 1.0/rho_;
}

void Point::Update(const shared_ptr<Frame> &frame, double depth, double px_error_angle) {
  assert(feature_);
  assert(feature_->GetFrame());

  SE3 pose = feature_->GetFrame()->GetPose()*frame->GetPose().Inverse();
  double tau = ComputeTau(pose, feature_->GetVector(), depth, px_error_angle);
  double tau_inverse = 0.5 * (1.0/std::max(0.0000001, depth-tau) - 1.0/(depth+tau));

  double tau2 = tau_inverse*tau_inverse;
  double x = 1./depth;
  double norm_scale = sqrt(sigma2_ + tau2);
  if (std::isnan(norm_scale))
    return;

  double s2 = 1./(1./sigma2_ + 1./tau2);
  double m = s2*(rho_/sigma2_ + x/tau2);
  double C1 = a_/(a_+b_) * PDFNormal(rho_, norm_scale, x);
  double C2 = b_/(a_+b_) * 1./z_range_;
  double normalization_constant = C1 + C2;
  C1 /= normalization_constant;
  C2 /= normalization_constant;
  double f = C1*(a_+1.)/(a_+b_+1.) + C2*a_/(a_+b_+1.);
  double e = C1*(a_+1.)*(a_+2.)/((a_+b_+1.)*(a_+b_+2.))
          + C2*a_*(a_+1.0f)/((a_+b_+1.0f)*(a_+b_+2.0f));

  // Update parameters
  double rho_new = C1*m+C2*rho_;
  sigma2_ = C1*(s2 + m*m) + C2*(sigma2_ + rho_*rho_) - rho_new*rho_new;
  rho_ = rho_new;
  a_ = (e-f)/(f-e/f);
  b_ = a_*(1.0f-f)/f;

  // Get parallax and distance to last frame
  Eigen::Vector3d pos = GetPosition();
  cos_alpha_ = GetParallax(feature_->GetFrame()->GetWorldPosition(), frame->GetWorldPosition(), pos);
  last_distance_ = frame->DistanceTo(pos);
  n_failed_ = 0;  // If it is updated, it means point was found
}


bool Point::Promote() {
  n_successful_++;
  n_failed_ = 0;
  return true;
}

bool Point::Unpromote() {
  n_failed_++;
  b_++;
  if (n_failed_ > Config::MaxFailed())
    return true;

  return false;
}

void Point::DeleteFeature(const std::shared_ptr<Feature> &feature) {
  // At the most there will be one feature
  for (auto it=features_.begin(); it != features_.end(); it++) {
    if (*it == feature) {
      features_.erase(it);
      break;
    }
  }
}

Eigen::Vector3d Point::GetPosition() const {
  Eigen::Vector3d pos;

  assert(feature_);
  assert(feature_->GetFrame());

  // Point has converged, return calculated xyz
  if (fixed_)
    return p3d_;

  // Transform from inverse depth to xyz
  SE3 se3 = feature_->GetFrame()->GetWorldPose();
  pos = se3*(1.0/rho_ * feature_->GetVector());
  return pos;
}

void Point::SetPosition(const Eigen::Vector3d &pos) {
  if (fixed_) {
    p3d_ = pos;
  } else {
    std::shared_ptr<Frame> frame = feature_->GetFrame();

    // Update 3d vector
    Eigen::Vector2d p2d;
    frame->Project(pos, &p2d);
    Eigen::Vector3d v3d = frame->GetCamera()->Unproject(p2d);
    feature_->SetVector(v3d);

    // Update inverse depth
    double depth = frame->DistanceTo(pos);
    rho_ = 1.0/depth;
  }
}

bool Point::HasConverged() {
  if (fixed_)
    return true;

  double std_d = sqrt(sigma2_)/(rho_*rho_);
  double l = 4*std_d*cos_alpha_/last_distance_;
  bool converged = (l < 0.1);
  if (converged) {
    // Get current position with inverse depth
    p3d_ = GetPosition();
    fixed_ = true;
    return true;
  }
  return false;
}

bool Point::SeenFrom(const std::shared_ptr<Frame> &frame) const {
  for (auto it=features_.begin(); it != features_.end(); it++) {
    if (frame->GetID() == (*it)->GetFrame()->GetID())
      return true;
  }
  return false;
}

double Point::ComputeTau(const SE3 &pose, const Eigen::Vector3d &v, double depth, double px_error_angle) {
  const double PI = 3.14159265;
  Eigen::Vector3d t(pose.GetTranslation());
  Eigen::Vector3d a = v*depth-t;
  double t_norm = t.norm();
  double a_norm = a.norm();
  double alpha = acos(v.dot(t)/t_norm);  // dot product
  double beta = acos(a.dot(-t)/(t_norm*a_norm));  // dot product
  double beta_plus = beta + px_error_angle;
  double gamma_plus = PI-alpha-beta_plus;  // triangle angles sum to PI
  double depth_plus = t_norm*sin(beta_plus)/sin(gamma_plus);  // law of sines
  return (depth_plus - depth);  // tau
}

double Point::PDFNormal(double mean, double sd, double x) {
  const double PI = 3.14159265;
  double result = 0.0;

  if (sd <= 0)
    return result;

  // pdf = e^(-(x-m)^2/(2s^2))/(s*sqrt(2*pi)
  double exponent = x - mean;
  exponent *= -exponent;
  exponent /= 2 * sd * sd;

  result = exp(exponent);
  result /= sd * sqrt(2.0 * PI);
  return result;
}


}  // namespace sdvl
