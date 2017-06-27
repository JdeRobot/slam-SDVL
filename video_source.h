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

#ifndef SDVL_VIDEO_SOURCE_H_
#define SDVL_VIDEO_SOURCE_H_

#include <fstream>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <string>

namespace sdvl {

class VideoSource {
 public:
  VideoSource();

  // Get a new frame from video source
  void GetFrame(cv::Mat *img);

 private:
  bool use_real_;
  cv::VideoCapture * cap_;

  std::string path_;
  std::string filename_;
  std::ifstream file_;
};

}  // namespace sdvl

#endif  // SDVL_VIDEO_SOURCE_H_
