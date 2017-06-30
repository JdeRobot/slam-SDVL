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

#include "./drawarea.h"

using std::vector;
using std::cerr;
using std::endl;

namespace sdvl {

DrawArea::DrawArea(Camera * camera) : Gtk::DrawingArea(), Gtk::GL::Widget<DrawArea>() {
  width_ = 640;
  height_ = 480;
  refresh_time_ = 100;  // ms
  camera_ = camera;
  follow_ = true;

  // Resize drawing area
  set_size_request(640, 480);

  Glib::RefPtr<Gdk::GL::Config> glconfig = Gdk::GL::Config::create(Gdk::GL::MODE_RGB | Gdk::GL::MODE_DEPTH | Gdk::GL::MODE_DOUBLE);
  if (!glconfig) {
    cerr << "*** Cannot find the double-buffered visual.\n" << "*** Trying single-buffered visual" << endl;

    // Try single-buffered visual
    glconfig = Gdk::GL::Config::create(Gdk::GL::MODE_RGB | Gdk::GL::MODE_DEPTH);
    if (!glconfig) {
      cerr << "*** Cannot find any OpenGL-capable visual" << endl;
      exit(1);
    }
  }

  // Set OpenGL-capability to the widget.
  unrealize();
  if (!set_gl_capability(glconfig) || !is_gl_capable()) {
    cerr << "No Gl capability" << endl;
    exit(1);
  }
  realize();

  // Add events
  add_events(
      Gdk::BUTTON1_MOTION_MASK | Gdk::BUTTON2_MOTION_MASK | Gdk::BUTTON3_MOTION_MASK | Gdk::BUTTON_PRESS_MASK | Gdk::BUTTON_RELEASE_MASK
          | Gdk::VISIBILITY_NOTIFY_MASK);

  signal_motion_notify_event().connect(sigc::mem_fun(this, &DrawArea::on_motion_notify));
  signal_scroll_event().connect(sigc::mem_fun(this, &DrawArea::on_drawarea_scroll));

  // Call to expose_event
  Glib::signal_timeout().connect(sigc::mem_fun(*this, &DrawArea::OnTimeout), refresh_time_);

  // Init Glut
  int val_init = 0;
  glutInit(&val_init, NULL);
  glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);

  // GL Camera Position and FOA
  glcam_posX_ = 0.0;
  glcam_posY_ = 0.0;
  glcam_posZ_ = 2.5;
  glcam_foaX_ = 0.0;
  glcam_foaY_ = 0.0;
  glcam_foaZ_ = 0.0;
  glcam_upX_ = 0.0;
  glcam_upY_ = 1.0;
  glcam_upZ_ = 0.0;

  radius_ = 2.0;
  latitud_ = 0.2;
  longitud_ = -1.0;
  old_x_ = 0.0;
  old_y_ = 0.0;

  pthread_mutex_init(&mutex_3D, NULL);
}

DrawArea::~DrawArea() {
}

void DrawArea::SetCameraTrail(const vector<std::pair<sdvl::SE3, bool>> &trail) {
  Lock();
  camera_trail_.clear();

  for (auto it=trail.begin(); it != trail.end(); it++)
    camera_trail_.push_back(std::make_pair(it->first, it->second));
  UnLock();
}

void DrawArea::SetPoints(const vector<Eigen::Vector3d> &points) {
  Lock();
  points_.clear();

  for (vector<Eigen::Vector3d>::const_iterator it=points.begin(); it != points.end(); it++)
    points_.push_back((*it));

  UnLock();
}

bool DrawArea::on_expose_event(GdkEventExpose* event) {
  Gtk::Allocation allocation = get_allocation();
  GLdouble width, height;

  Glib::RefPtr<Gdk::GL::Window> glwindow = get_gl_window();
  glwindow->gl_begin(get_gl_context());

  glDrawBuffer(GL_BACK);
  glClearColor(1.0, 1.0, 1.0, 1.0);

  glClearDepth(1.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  width = allocation.get_width();
  height = allocation.get_height();

  glEnable(GL_DEPTH_TEST);
  glViewport(0, 0, width, height);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  // Angle, ratio, znear, zfar
  gluPerspective(50.0, width / height, 0.1, 5000.0);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  // Calc camera pose
  SetCameraPose(pose_);

  // Draw map
  glPushMatrix();
  DrawMap();
  glPopMatrix();

  // Swap buffers
  if (glwindow->is_double_buffered())
    glwindow->swap_buffers();
  else
    glFlush();

  glwindow->gl_end();

  return true;
}

void DrawArea::DrawMap() {
  bool showlines = false;

  Lock();

  // Draw camera trail
  for (auto it=camera_trail_.begin(); it != camera_trail_.end(); it++)
    DrawCamera(it->first, true, it->second);

  // Draw current position
  DrawCamera(pose_, false, false);

  glPointSize(2);
  glLineWidth(1);
  for (auto it=points_.begin(); it != points_.end(); it+=2) {
    Eigen::Vector3d diff = *it - *(it+1);
    if (diff.dot(diff) < 1e-10) {
      // Draw point in space
      glColor3f(0.5, 0.0, 1.0);
      glBegin(GL_POINTS);
      DrawPoint(*it);
      glEnd();
    } else {
      if (showlines) {
        // Draw line
        glColor3f(1.0, 0.0, 0.2);
        glBegin(GL_LINES);
        DrawPoint(*it);
        DrawPoint(*(it+1));
        glEnd();
      }
    }
  }

  glColor3f(0, 0, 0);
  glLineWidth(1);

  UnLock();
}

void DrawArea::DrawCamera(const SE3 &se3, bool slim, bool colored) {
  glPushMatrix();
  glMultMatrix(se3);

  if (colored)
    glColor3f(0.9, 0.0, 0.0);
  else
    glColor3f(0.0, 0.0, 0.0);

  if (slim) {
    glLineWidth(1);
    DrawFrustrum(0.1);
  } else {
    glLineWidth(2);
    glutWireCube(0.05);
    DrawFrustrum(0.1);
  }

  glPopMatrix();
}

void DrawArea::DrawPoint(const Eigen::Vector3d &pos) {
  glVertex3d(pos(0), pos(1), pos(2));
}

void DrawArea::DrawFrustrum(double depth) {
  double left = (depth / camera_->GetFx()) * camera_->GetU0();
  double right = (depth / camera_->GetFx()) * (camera_->GetU0() - camera_->GetWidth());
  double top = (depth / camera_->GetFy()) * camera_->GetV0();
  double bottom = (depth / camera_->GetFy()) * (camera_->GetV0() - camera_->GetHeight());

  // Left triangle
  glBegin(GL_LINE_STRIP);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(left, top, depth);
  glVertex3f(left, bottom, depth);
  glEnd();

  // Bottom triangle
  glBegin(GL_LINE_STRIP);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(left, bottom, depth);
  glVertex3f(right, bottom, depth);
  glEnd();

  // Right triangle
  glBegin(GL_LINE_STRIP);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(right, bottom, depth);
  glVertex3f(right, top, depth);
  glEnd();

  // Upper triangle
  glBegin(GL_LINE_STRIP);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(right, top, depth);
  glVertex3f(left, top, depth);
  glEnd();
}

void DrawArea::SetCameraPose(const SE3 &se3) {
  Eigen::Matrix3d rot = se3.GetRotation();
  Eigen::Vector3d t = se3.GetTranslation();
 
  if(follow_) {
    glcam_foaX_ = t(0);
    glcam_foaY_ = t(1);
    glcam_foaZ_ = t(2);

    Eigen::Matrix4d rt;
    Eigen::Vector4d dist(0, 0, -1.5, 1);
    rt << rot(0, 0), rot(0, 1), rot(0, 2), t(0), 
          rot(1, 0), rot(1, 1), rot(1, 2), t(1),
          rot(2, 0), rot(2, 1), rot(2, 2), t(2),
          0, 0, 0, 1;

    Eigen::MatrixXd res = rt*dist;

    glcam_posX_ = res(0);
    glcam_posY_ = res(1);
    glcam_posZ_ = res(2);
  }

  // Cam position, central point, vector up
  gluLookAt(glcam_posX_, glcam_posY_, glcam_posZ_, glcam_foaX_, glcam_foaY_, glcam_foaZ_, glcam_upX_, glcam_upY_, glcam_upZ_);
}

void DrawArea::glMultMatrix(const SE3 &se3) {
  Eigen::Matrix3d rot = se3.GetRotation();
  Eigen::Vector3d t = se3.GetTranslation();

  glTranslatef(static_cast<float>(t(0)), static_cast<float>(t(1)), static_cast<float>(t(2)));
  GLdouble mGL[16];
  mGL[0] = rot(0, 0);
  mGL[1] = rot(1, 0);
  mGL[2] = rot(2, 0);
  mGL[3] = 0;

  mGL[4] = rot(0, 1);
  mGL[5] = rot(1, 1);
  mGL[6] = rot(2, 1);
  mGL[7] = 0;

  mGL[8] = rot(0, 2);
  mGL[9] = rot(1, 2);
  mGL[10] = rot(2, 2);
  mGL[11] = 0;

  mGL[12] = 0;
  mGL[13] = 0;
  mGL[14] = 0;
  mGL[15] = 1;

  glMultMatrixd(mGL);
}

bool DrawArea::OnTimeout() {
  // Force our program to redraw
  Glib::RefPtr<Gdk::Window> win = get_window();
  if (win) {
    Gdk::Rectangle r(0, 0, get_allocation().get_width(), get_allocation().get_height());
    win->invalidate_rect(r, false);
  }
  return true;
}

bool DrawArea::on_motion_notify(GdkEventMotion* event) {
  double desp = 0.01;
  double x = event->x;
  double y = event->y;

  // if left mouse button is toggled
  if (event->state & GDK_BUTTON1_MASK) {
    if ((x - old_x_) > 0.0)
      longitud_ -= desp;
    else if ((x - old_x_) < 0.0)
      longitud_ += desp;

    if ((y - old_y_) > 0.0)
      latitud_ += desp;
    else if ((y - old_y_) < 0.0)
      latitud_ -= desp;

    glcam_posX_ = radius_ * cosf(latitud_) * sinf(longitud_) + glcam_foaX_;
    glcam_posY_ = radius_ * sinf(latitud_) + glcam_foaY_;
    glcam_posZ_ = radius_ * cosf(latitud_) * cosf(longitud_) + glcam_foaZ_;
  }
  // if right mouse button is toggled
  if (event->state & GDK_BUTTON3_MASK) {
    if ((x - old_x_) > 0.0)
      longitud_ -= desp;
    else if ((x - old_x_) < 0.0)
      longitud_ += desp;

    if ((y - old_y_) > 0.0)
      latitud_ += desp;
    else if ((y - old_y_) < 0.0)
      latitud_ -= desp;

    glcam_foaX_ = -radius_ * cosf(latitud_) * sinf(longitud_) + glcam_posX_;
    glcam_foaY_ = -radius_ * sinf(latitud_) + glcam_posY_;
    glcam_foaZ_ = -radius_ * cosf(latitud_) * cosf(longitud_) + glcam_posZ_;
  }

  old_x_ = x;
  old_y_ = y;
  follow_ = false;

  return true;
}

bool DrawArea::on_drawarea_scroll(GdkEventScroll * event) {
  double vx, vy, vz;

  vx = (glcam_foaX_ - glcam_posX_) / (radius_ * 3);
  vy = (glcam_foaY_ - glcam_posY_) / (radius_ * 3);
  vz = (glcam_foaZ_ - glcam_posZ_) / (radius_ * 3);

  if (event->direction == GDK_SCROLL_UP) {
    glcam_foaX_ = glcam_foaX_ + vx;
    glcam_foaY_ = glcam_foaY_ + vy;
    glcam_foaZ_ = glcam_foaZ_ + vz;

    glcam_posX_ = glcam_posX_ + vx;
    glcam_posY_ = glcam_posY_ + vy;
    glcam_posZ_ = glcam_posZ_ + vz;
  }

  if (event->direction == GDK_SCROLL_DOWN) {
    glcam_foaX_ = glcam_foaX_ - vx;
    glcam_foaY_ = glcam_foaY_ - vy;
    glcam_foaZ_ = glcam_foaZ_ - vz;

    glcam_posX_ = glcam_posX_ - vx;
    glcam_posY_ = glcam_posY_ - vy;
    glcam_posZ_ = glcam_posZ_ - vz;
  }
  follow_ = false;

  return true;
}

}  // namespace sdvl

