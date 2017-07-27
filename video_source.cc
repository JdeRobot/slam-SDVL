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

#include "./video_source.h"
#include "./config.h"

using std::cout;
using std::cerr;
using std::endl;

namespace sdvl {

VideoSource::VideoSource() {
  VideoParameters &params = Config::GetVideoParameters();
  use_real_ = (params.type == 0);

  if (use_real_) {
    cout << "[INFO] VideoSource_Linux: Opening video source /dev/video" << params.device << "..." << endl;
    cap_ = new cv::VideoCapture(params.device);
    cap_->set(CV_CAP_PROP_FRAME_WIDTH, params.width);
    cap_->set(CV_CAP_PROP_FRAME_HEIGHT, params.height);
    cap_->set(CV_CAP_PROP_FPS, params.fps);
    if (!cap_->isOpened()) {
      cerr << "[ERROR] Couldn't open video device" << endl;
      exit(-1);
    }
    cout << "[INFO] ... got video source." << endl;
  } else {
    path_ = params.path;
    filename_ = params.filename;

    // Open file
    std::stringstream ss;
    ss << path_ << filename_;
    file_.open(ss.str());
    if (!file_.is_open())
      cerr << "[ERROR] File not found: " << ss.str() << endl;
  }
}

void VideoSource::GetFrame(cv::Mat *img) {
  if (use_real_) {
    cv::Mat frame;
    *cap_ >> frame;
    cv::cvtColor(frame, *img, CV_RGB2GRAY);
  } else {
    if (file_.is_open()) {
      std::string line;
      std::stringstream ss;

      if (getline(file_, line)) {
        ss << path_ << line;
        cout << "[INFO] Reading Frame " << ss.str() << endl;

        // Read image in gray scale
        *img = cv::imread(ss.str(), CV_LOAD_IMAGE_GRAYSCALE);
      } else {
        *img = cv::Mat();
        file_.close();
      }
    }
  }
}

}  // namespace sdvl
