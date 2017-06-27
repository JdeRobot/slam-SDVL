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

UI::UI(Camera* camera, SDVL * handler, bool show) : gtkmain_(0, 0) {
  camera_ = camera;
  handler_ = handler;

  pthread_mutex_init(&mutex_display_, NULL);
  pthread_mutex_init(&mutex_handler_, NULL);

  // Init OpenGL
  if (!Gtk::GL::init_check(NULL, NULL)) {
    cerr << "Couldn't initialize GL" << endl;
    exit(1);
  }

  // Create widgets
  mainwindow_ = new Gtk::Window();
  main_table_ = new Gtk::Table(1, 2, true);
  drawarea_ = new DrawArea(camera_);
  drawimage_ = new DrawImage(camera_);
  mainwindow_->add(*main_table_);
  main_table_->attach(*drawimage_, 0, 1, 0, 1);
  main_table_->attach(*drawarea_, 1, 2, 0, 1);

  if (show) {
    // Show window
    mainwindow_->show();
    main_table_->show();
    drawarea_->show();
    drawimage_->show();
  }
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
  drawarea_->SetCameraTrail(camera_trail);
  drawarea_->SetPoints(points);
  drawarea_->SetCurrentPose(pose);
  drawimage_->SetFeatures(features);
  drawimage_->SetCurrentPose(pose.Inverse());
  drawimage_->SetTrackingQuality(quality);
  pthread_mutex_unlock(&mutex_display_);
}

bool UI::IsVisible() {
  return mainwindow_->is_visible();
}

void UI::Display() {
  // Show window
  pthread_mutex_lock(&mutex_display_);
  mainwindow_->resize(1, 1);
  while (gtkmain_.events_pending())
    gtkmain_.iteration();
  pthread_mutex_unlock(&mutex_display_);
}

}  // namespace sdvl
