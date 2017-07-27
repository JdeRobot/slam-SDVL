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
#include <thread>
#include "./sdvl.h"
#include "./camera.h"
#include "./video_source.h"
#include "./config.h"
#include "extra/timer.h"
#ifdef USE_GUI
#include "ui/ui.h"
#endif

using std::cout;
using std::cerr;
using std::endl;

#ifdef USE_GUI
class UIThread {
 public:
  UIThread() {
    running_ = false;
  }

  void Run() {
    // Create user interface
    ui_ = new sdvl::UI(camera_, handler_, true);
    running_ = true;

    while (running_) {
      // Show UI
      if (ui_->IsVisible())
        ui_->Display();

      usleep(100 * 1000);
    }
  }

  void Start(sdvl::Camera * camera, sdvl::SDVL * handler) {
    camera_ = camera;
    handler_ = handler;
    thread_ = new std::thread(&UIThread::Run,this);
  }

  void Stop() {
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

  std::thread* thread_;
  bool running_;
};
#endif

int main(int argc, char** argv) {
  #ifdef USE_GUI
  UIThread ui;
  #endif
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

  #ifdef USE_GUI
  // Start UI
  ui.Start(camera, nullptr);
  usleep(1000 * 1000);
  #endif

  // Start algorithm
  handler = new sdvl::SDVL(camera);
  if (!sequential)
    handler->Start();
  #ifdef USE_GUI
  ui.SetHandler(handler);
  #endif

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

    #ifdef USE_GUI
    // Update UI
    ui.Update(imgu);
    #endif

    // Sleep until next frame
    if (!sequential && timer.GetMsTime() < it_time)
      usleep((it_time-timer.GetMsTime())*1000);
  }

  cout << "Closing SDVL..." << endl;

  // Stop threads
  if (handler != nullptr && !sequential)
    handler->Stop();
  #ifdef USE_GUI
  ui.Stop();
  #endif

  return 0;
}
