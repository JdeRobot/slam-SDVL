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

#ifndef SDVL_UI_DRAWAREA_H_
#define SDVL_UI_DRAWAREA_H_

#include <string>
#include <iostream>
#include <utility>
#include <vector>
#include <pthread.h>
#include <gtkmm.h>
#include <gtkglmm.h>
#include <gdkglmm.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include "../camera.h"
#include "../extra/se3.h"

namespace sdvl {

class DrawArea: public Gtk::DrawingArea, public Gtk::GL::Widget<DrawArea> {
 public:
  explicit DrawArea(Camera * camera);
  virtual ~DrawArea();

  // Save camera positions and points
  void SetCameraTrail(const std::vector<std::pair<sdvl::SE3, bool>> &trail);
  void SetPoints(const std::vector<Eigen::Vector3d> &points);
  inline void SetCurrentPose(const sdvl::SE3 &pose) { pose_ = pose; }

  inline void Lock() { pthread_mutex_lock(&mutex_3D); }
  inline void UnLock() { pthread_mutex_unlock(&mutex_3D); }

 protected:
  // Override default signal handler
  virtual bool on_expose_event(GdkEventExpose* event);
  virtual bool on_motion_notify(GdkEventMotion* event);
  virtual bool on_drawarea_scroll(GdkEventScroll * event);

  void glMultMatrix(const SE3 &se3);
  bool OnTimeout();

  void DrawWorld();
  void DrawMap();
  void DrawFrustrum(double depth);
  void DrawCamera(const SE3 &se3, bool slim, bool colored);
  void DrawPoint(const Eigen::Vector3d &pos);

  pthread_mutex_t mutex_3D;
  int refresh_time_;
  int width_;
  int height_;

  double radius_;
  double latitud_;
  double longitud_;
  double old_x_;
  double old_y_;

  Camera * camera_;
  double glcam_posX_;
  double glcam_posY_;
  double glcam_posZ_;
  double glcam_foaX_;
  double glcam_foaY_;
  double glcam_foaZ_;

  std::vector<std::pair<sdvl::SE3, bool>> camera_trail_;    // Camera poses
  std::vector<Eigen::Vector3d> points_;                     // Points positions
  sdvl::SE3 pose_;
};

}  // namespace sdvl

#endif  // SDVL_UI_DRAWAREA_H_
