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

#include "./ui.h"
#include "../extra/se3.h"

using std::vector;
using std::cerr;
using std::endl;

namespace sdvl {

UI::UI(Camera* camera, SDVL * handler, bool show) {
  camera_ = camera;
  handler_ = handler;
  visible_ = true;

  pthread_mutex_init(&mutex_display_, NULL);
  pthread_mutex_init(&mutex_handler_, NULL);

  // Create elements
  drawscene_ = new DrawScene(camera_);
  drawimage_ = new DrawImage(camera_);
}

UI::~UI() {
}

void UI::SetHandler(SDVL * handler) {
  pthread_mutex_lock(&mutex_handler_);
  handler_ = handler;
  pthread_mutex_unlock(&mutex_handler_);
}

void UI::Update(const cv::Mat &image) {
  vector<std::pair<sdvl::SE3, bool>> camera_trail;
  vector<Eigen::Vector3d> points;
  vector<Eigen::Vector3i> features;
  sdvl::SE3 pose;
  SDVL::TrackingQuality quality;

  // Save image
  pthread_mutex_lock(&mutex_display_);
  drawimage_->SetBackground(image);
  pthread_mutex_unlock(&mutex_display_);

  if (handler_ == nullptr)
    return;

  pthread_mutex_lock(&mutex_handler_);
  handler_->GetCameraTrail(&camera_trail);
  handler_->GetPoints(&points);
  handler_->GetLastFeatures(&features);
  pose = handler_->GetPose();
  quality = handler_->GetTrackingQuality();
  pthread_mutex_unlock(&mutex_handler_);

  // Save parameters
  pthread_mutex_lock(&mutex_display_);
  drawscene_->SetCameraTrail(camera_trail);
  drawscene_->SetPoints(points);
  drawscene_->SetCurrentPose(pose);
  drawimage_->SetFeatures(features);
  drawimage_->SetCurrentPose(pose.Inverse());
  drawimage_->SetTrackingQuality(quality);
  pthread_mutex_unlock(&mutex_display_);
}

bool UI::IsVisible() {
  return visible_;
}

void UI::Display() {
  // Update
  pthread_mutex_lock(&mutex_display_);
  drawscene_->ShowScene();
  drawimage_->ShowImage();
  pthread_mutex_unlock(&mutex_display_);
}

}  // namespace sdvl
