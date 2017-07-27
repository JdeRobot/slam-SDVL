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

#ifndef SDVL_UI_DRAWSCENE_H_
#define SDVL_UI_DRAWSCENE_H_

#include <string>
#include <iostream>
#include <utility>
#include <vector>
#include <mutex>
#include <pangolin/pangolin.h>
#include "../camera.h"
#include "../extra/se3.h"

namespace sdvl {

class DrawScene {
 public:
  explicit DrawScene(Camera * camera);
  virtual ~DrawScene();

  // Save camera positions and points
  void SetCameraTrail(const std::vector<std::pair<sdvl::SE3, bool>> &trail);
  void SetPoints(const std::vector<Eigen::Vector3d> &points);
  inline void SetCurrentPose(const sdvl::SE3 &pose) { pose_ = pose; }

  void ShowScene();

 protected:
  // Convert SE3 to OpenGL matrix
  void GetCameraMatrix(const SE3 &se3, pangolin::OpenGlMatrix *m);

  // Draw elements in OpenGL
  void DrawFrustrum(double depth, bool tail);
  void DrawCamera(const SE3 &se3, bool slim, bool colored);
  void DrawPoint(const Eigen::Vector3d &pos);

  std::mutex mutex_3D_;

  Camera * camera_;
  std::vector<std::pair<sdvl::SE3, bool>> camera_trail_;    // Camera poses
  std::vector<Eigen::Vector3d> points_;                     // Points positions
  sdvl::SE3 pose_;

  // Pangolin parameters
  pangolin::OpenGlRenderState *s_cam_;
  pangolin::View d_cam_;
  pangolin::Var<bool> *follow_;
  pangolin::Var<bool> *show_lines_;
};

}  // namespace sdvl

#endif  // SDVL_UI_DRAWSCENE_H_
