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

#include "./drawscene.h"

using std::vector;
using std::cerr;
using std::endl;

namespace sdvl {

DrawScene::DrawScene(Camera * camera) {
  camera_ = camera;

  // Create new pangolin window
  pangolin::CreateWindowAndBind("SDVL Map", 1024, 768);

  // Init OpenGL
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Add menu to pangolin window
  pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
  follow_ = new pangolin::Var<bool>("menu.Follow Camera", true, true);
  show_lines_ = new pangolin::Var<bool>("menu.Show Depth Lines", false, true);
  // pangolin::Var<bool> menuReset("menu.Reset", false, false);

  // Create OpenGL area with pangolin
  s_cam_ = new pangolin::OpenGlRenderState(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.7, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));

  d_cam_ = pangolin::CreateDisplay()
          .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
          .SetHandler(new pangolin::Handler3D(*s_cam_));
}

DrawScene::~DrawScene() {
}

void DrawScene::SetCameraTrail(const vector<std::pair<sdvl::SE3, bool>> &trail) {
  std::unique_lock<std::mutex> lock(mutex_3D_);
  camera_trail_.clear();

  for (auto it=trail.begin(); it != trail.end(); it++)
    camera_trail_.push_back(std::make_pair(it->first, it->second));
}

void DrawScene::SetPoints(const vector<Eigen::Vector3d> &points) {
  std::unique_lock<std::mutex> lock(mutex_3D_);
  points_.clear();

  for (vector<Eigen::Vector3d>::const_iterator it=points.begin(); it != points.end(); it++)
    points_.push_back((*it));
}

void DrawScene::ShowScene() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Get camera pose
  pangolin::OpenGlMatrix cpose;
  GetCameraMatrix(pose_, &cpose);

  // Follow camera
  if (*follow_)
    s_cam_->Follow(cpose);

  d_cam_.Activate(*s_cam_);
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

  // Draw saved data
  {
    std::unique_lock<std::mutex> lock(mutex_3D_);

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
        if (*show_lines_) {
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
  }

  pangolin::FinishFrame();
}

void DrawScene::DrawCamera(const SE3 &se3, bool slim, bool colored) {
  glPushMatrix();

  pangolin::OpenGlMatrix cpose;
  GetCameraMatrix(se3, &cpose);
  glMultMatrixd(cpose.m);

  if (slim) {
    if (colored)
      glColor3f(0.9, 0.0, 0.0);
    else
      glColor3f(0.5, 0.5, 0.5);
    glLineWidth(1);
    DrawFrustrum(0.1, false);
  } else {
    glColor3f(0.0, 0.0, 0.0);
    glLineWidth(3);
    DrawFrustrum(0.1, true);
  }
  glPopMatrix();
}

void DrawScene::DrawPoint(const Eigen::Vector3d &pos) {
  glVertex3d(pos(0), pos(1), pos(2));
}

void DrawScene::DrawFrustrum(double depth, bool tail) {
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

  if (tail) {
    double mdepth = depth*0.25;
    // Draw cube
    glBegin(GL_LINE_STRIP);
    glVertex3f(mdepth, -mdepth, mdepth);
    glVertex3f(mdepth, mdepth, mdepth);
    glVertex3f(-mdepth, mdepth, mdepth);
    glVertex3f(-mdepth, -mdepth, mdepth);
    glVertex3f(mdepth, -mdepth, mdepth);
    glEnd();

    glBegin(GL_LINE_STRIP);
    glVertex3f(mdepth, -mdepth, -mdepth);
    glVertex3f(mdepth, mdepth, -mdepth);
    glVertex3f(-mdepth, mdepth, -mdepth);
    glVertex3f(-mdepth, -mdepth, -mdepth);
    glVertex3f(mdepth, -mdepth, -mdepth);
    glEnd();

    glBegin(GL_LINE_STRIP);
    glVertex3f(mdepth, mdepth, -mdepth);
    glVertex3f(mdepth, mdepth, mdepth);
    glVertex3f(-mdepth, mdepth, mdepth);
    glVertex3f(-mdepth, mdepth, -mdepth);
    glVertex3f(mdepth, mdepth, -mdepth);
    glEnd();

    glBegin(GL_LINE_STRIP);
    glVertex3f(mdepth, -mdepth, -mdepth);
    glVertex3f(mdepth, -mdepth, mdepth);
    glVertex3f(-mdepth, -mdepth, mdepth);
    glVertex3f(-mdepth, -mdepth, -mdepth);
    glVertex3f(mdepth, -mdepth, -mdepth);
    glEnd();
  }
}

void DrawScene::GetCameraMatrix(const SE3 &se3, pangolin::OpenGlMatrix *m) {
  Eigen::Matrix3d rot = se3.GetRotation();
  Eigen::Vector3d t = se3.GetTranslation();

  m->m[0] = rot(0, 0);
  m->m[1] = rot(1, 0);
  m->m[2] = rot(2, 0);
  m->m[3]  = 0.0;

  m->m[4] = rot(0, 1);
  m->m[5] = rot(1, 1);
  m->m[6] = rot(2, 1);
  m->m[7]  = 0.0;

  m->m[8] = rot(0, 2);
  m->m[9] = rot(1, 2);
  m->m[10] = rot(2, 2);
  m->m[11]  = 0.0;

  m->m[12] = t(0);
  m->m[13] = t(1);
  m->m[14] = t(2);
  m->m[15]  = 1.0;
}

}  // namespace sdvl

