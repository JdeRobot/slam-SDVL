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

#include "./drawimage.h"
#include "../point.h"
#include "../config.h"

using std::vector;
using std::cerr;
using std::endl;

namespace sdvl {

DrawImage::DrawImage(Camera * camera) : Gtk::DrawingArea(), Gtk::GL::Widget<DrawImage>() {
  width_ = 640;
  height_ = 480;
  refresh_time_ = 100;  // ms
  camera_ = camera;

  // Resize drawing area
  set_size_request(width_, height_);

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
  add_events(Gdk::VISIBILITY_NOTIFY_MASK);

  // Call to expose_event
  Glib::signal_timeout().connect(sigc::mem_fun(*this, &DrawImage::OnTimeout), refresh_time_);

  // Set 3D
  pthread_mutex_init(&mutex_3D_, NULL);

  // Image
  image_ = cv::Mat(Config::GetCameraParameters().width, Config::GetCameraParameters().height, CV_8UC3);
}

DrawImage::~DrawImage() {
}

void DrawImage::SetBackground(const cv::Mat &src) {
  // Convert to RGB
  Lock();
  cv::cvtColor(src, image_, CV_GRAY2RGB);
  Unlock();
}

void DrawImage::SetFeatures(const vector<Eigen::Vector3i> &features) {
  CvScalar color;

  Lock();
  for (vector<Eigen::Vector3i>::const_iterator it=features.begin(); it != features.end(); it++) {
    if ((*it)(2) == Point::P_FOUND)
      color = CV_RGB(0, 255, 0);
    else if ((*it)(2) == Point::P_NOT_FOUND || (*it)(2) == Point::P_OUTLIER)
      color = CV_RGB(255, 0, 0);
    else
      color = CV_RGB(255, 255, 0);
    cv::circle(image_, cvPoint((*it)(0), (*it)(1)), 3, color, 1, CV_AA, 0);
  }
  Unlock();
}

bool DrawImage::on_expose_event(GdkEventExpose* event) {
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

  glViewport(0, 0, width, height);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  // Angle, ratio, znear, zfar
  gluPerspective(50.0, width / height, 0.1, 5000.0);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  // Cam position, central point, vector up
  gluLookAt(0.0, 0.0, 0.51, 0.0, 0.0, 0.0, 0., 0.1, 0.);

  Lock();

  // Draw image texture
  glEnable(GL_TEXTURE_2D);
  DrawBackground(width, height);
  glDisable(GL_TEXTURE_2D);
  glEnable(GL_DEPTH_TEST);

  Unlock();

  // Swap buffers
  if (glwindow->is_double_buffered())
    glwindow->swap_buffers();
  else
    glFlush();

  glwindow->gl_end();

  return true;
}

bool DrawImage::OnTimeout() {
  // Force our program to redraw
  Glib::RefPtr<Gdk::Window> win = get_window();
  if (win) {
    Gdk::Rectangle r(0, 0, get_allocation().get_width(), get_allocation().get_height());
    win->invalidate_rect(r, false);
  }
  return true;
}

void DrawImage::DrawBackground(double width, double height) {
  // Resize image
  cv::Mat resized;
  cv::Size size(static_cast<int>(width), static_cast<int>(height));
  cv::resize(image_, resized, size);

  // Check quality
  if (quality_ == SDVL::TRACKING_BAD)
    SetChannel(&resized, 2, 200);
  if (quality_ == SDVL::TRACKING_INSUFFICIENT)
    SetChannel(&resized, 0, 200);

  // Create texture and bind it
  GLuint tex = MatToTexture(resized);
  glBindTexture(GL_TEXTURE_2D, tex);

  double tw = width / 1000.0;
  double th = height / 1000.0;

  glColor3f(1.0f, 1.0f, 1.0f);
  glBegin(GL_QUADS);
  glTexCoord2d(0.0, 0.0);
  glVertex2d(-tw / 2, th / 2);
  glTexCoord2d(1.0, 0.0);
  glVertex2d(tw / 2, th / 2);
  glTexCoord2d(1.0, 1.0);
  glVertex2d(tw / 2, -th / 2);
  glTexCoord2d(0.0, 1.0);
  glVertex2d(-tw / 2, -th / 2);
  glEnd();
  glDeleteTextures(1, &tex);
}

GLuint DrawImage::MatToTexture(const cv::Mat &mat) {
  // Generate a number for our textureID's unique handle
  GLuint textureID;
  glGenTextures(1, &textureID);

  // Set texture interpolation methods for minification and magnification
  glBindTexture(GL_TEXTURE_2D, textureID);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

  // Create the texture
  glTexImage2D(GL_TEXTURE_2D,     // Type of texture
      0,                 // Pyramid level (for mip-mapping) - 0 is the top level
      GL_RGB,            // Internal colour format to convert to
      mat.cols,          // Image width  i.e. 640 in standard mode
      mat.rows,          // Image height i.e. 480 in standard mode
      0,                 // Border width in pixels (can either be 1 or 0)
      GL_BGR,            // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
      GL_UNSIGNED_BYTE,  // Image data type
      mat.ptr());        // The actual image data itself

  return textureID;
}

void DrawImage::SetChannel(cv::Mat *mat, unsigned int channel, unsigned char value) {
  const int cols = mat->cols;
  const int step = mat->channels();
  const int rows = mat->rows;
  for (int y = 0; y < rows; y++) {
    // get pointer to the first byte to be changed in this row
    unsigned char *p_row = mat->ptr(y) + channel;
    unsigned char *row_end = p_row + cols*step;
    for (; p_row != row_end; p_row += step)
      *p_row = value;
  }
}

bool DrawImage::Project(double x, double y, double z, cv::Point *p) {
  Eigen::Vector3d p3D, rel_p;
  Eigen::Vector2d p2D;
  int limit = 2000;

  p3D << x, y, z;
  rel_p = pose_ * p3D;
  if (rel_p(2) < 0.0)
    return false;

  camera_->Project(rel_p, &p2D);
  p->x = p2D(0);
  p->y = p2D(1);
  if (p->x < -limit || p->x > limit)
    return false;
  if (p->y < -limit || p->y > limit)
    return false;
  return true;
}

}  // namespace sdvl

