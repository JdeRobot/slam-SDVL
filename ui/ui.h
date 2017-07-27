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

#ifndef SDVL_UI_UI_H_
#define SDVL_UI_UI_H_

#include <string>
#include <iostream>
#include <mutex>
#include "../camera.h"
#include "../sdvl.h"
#include "./drawscene.h"
#include "./drawimage.h"

namespace sdvl {

class UI {
 public:
  UI(Camera * camera, SDVL * handler, bool show);
  virtual ~UI();

  void SetHandler(SDVL * handler);

  // Return true if UI is visible
  bool IsVisible();

  // Update information to display
  void Update(const cv::Mat &image);

  // Display window
  void Display();

 private:
  DrawScene * drawscene_;
  DrawImage * drawimage_;
  Camera * camera_;
  SDVL * handler_;
  bool visible_;

  std::mutex mutex_display_;
  std::mutex mutex_handler_;
};

}  // namespace sdvl

#endif  // SDVL_UI_UI_H_
