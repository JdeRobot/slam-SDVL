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

#include <iostream>
#include <pthread.h>
#include "./sdvl.h"
#include "./camera.h"
#include "./video_source.h"
#include "./config.h"
#include "ui/ui.h"
#include "extra/timer.h"

using std::cout;
using std::cerr;
using std::endl;

void* callback_ui(void* obj);

class UIThread {
 public:
  UIThread() {
    running_ = false;
  }

  int main() {
    // Create user interface
    ui_ = new sdvl::UI(camera_, handler_, true);
    running_ = true;

    while (running_) {
      // Show UI
      if (ui_->IsVisible())
        ui_->Display();

      usleep(100 * 1000);
    }

    return 0;
  }

  void start(sdvl::Camera * camera, sdvl::SDVL * handler) {
    camera_ = camera;
    handler_ = handler;
    pthread_create(&thread_, 0, &callback_ui, this);
  }

  void stop() {
    running_ = false;
  }

  void Update(const cv::Mat &img) {
    if (running_) {
      ui_->Update(img);
    }
  }

  void SetHandler(sdvl::SDVL * handler) {
    handler_ = handler;
    ui_->SetHandler(handler_);
  }

 private:
  sdvl::UI * ui_;
  sdvl::Camera * camera_;
  sdvl::SDVL * handler_;

  pthread_t thread_;
  bool running_;
};

void* callback_ui(void* obj) {
  static_cast<UIThread*>(obj)->main();
  return (0);
}

int main(int argc, char** argv) {
  UIThread ui;
  cv::Mat img, imgu;
  sdvl::VideoSource * video;
  sdvl::SDVL * handler = nullptr;
  sdvl::Camera * camera;
  bool sequential = false;
  bool exit = false;
  double FPS = 30.0;
  double it_time = 1000.0*(1.0/FPS);

  // Read parameters
  sdvl::Config &config = sdvl::Config::GetInstance();
  if (!config.ReadParameters("../config.cfg")) {
    cerr << "[ERROR] Config file contains errors" << endl;
    return -1;
  }

  video = new sdvl::VideoSource();
  camera = new sdvl::Camera();

  // Start UI
  ui.start(camera, nullptr);
  usleep(1000 * 1000);

  // Start algorithm
  handler = new sdvl::SDVL(camera);
  if (!sequential)
    handler->Start();
  ui.SetHandler(handler);

  while (!exit) {
    // Get frame
    video->GetFrame(&img);
    if (img.empty()) {
      exit = true;
      continue;
    }
    camera->UndistortImage(img, &imgu);

    // Handle current frame
    sdvl::Timer timer(true);
    handler->HandleFrame(imgu);
    timer.Stop();

    // Get Pose
    sdvl::SE3 pose = handler->GetPose();
    Eigen::Vector3d t = pose.GetTranslation();
    Eigen::Quaterniond q = pose.GetQuaternion();
    cout << "[INFO] Frame pose is: [" << t(0) << " " << t(1) << " " << t(2) << "]";
    cout << "[" << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "]" << endl;
    cout << "[INFO] Tracking time is " << timer.GetMsTime() << "ms" << endl;

    if (sequential)
      handler->Mapping();

    // Update UI
    ui.Update(imgu);

    // Sleep until next frame
    if (!sequential && timer.GetMsTime() < it_time)
      usleep((it_time-timer.GetMsTime())*1000);
  }

  cout << "Closing SDVL..." << endl;

  // Stop threads
  if (handler != nullptr && !sequential)
    handler->Stop();
  ui.stop();

  return 0;
}
