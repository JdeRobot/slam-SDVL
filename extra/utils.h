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

#ifndef SDVL_EXTRA_UTILS_H_
#define SDVL_EXTRA_UTILS_H_

#include <vector>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include "./se3.h"

namespace sdvl {

// Get absolute max value of a vector
double AbsMax(const Eigen::VectorXd &v);

// Interpolate a pixel in image
float Interpolate8U(const cv::Mat& mat, float u, float v);

// Calculate Shi Tomasi score for a feature
double FindShiTomasiScoreAtPoint(const cv::Mat &img, int px, int py);

// Jacobian of 3D point projection in frame coordinates to unit plane coordinates
void Jacobian3DToPlane(const Eigen::Vector3d &p, Eigen::Matrix<double, 2, 6> *J);

// Jacobian of point projection on unit plane in frame
void Jacobian3DToPlane(const Eigen::Vector3d &p, const Eigen::Matrix3d &R, Eigen::Matrix<double, 2, 3> *J);

// Triangulate two points in relative coordinates
Eigen::Vector3d Triangulate(const SE3 &pose, const Eigen::Vector3d &vector1, const Eigen::Vector3d &vector2);

// Linear triangulation in 3D space
bool Triangulate(const SE3 &pose1, const SE3 &pose2, const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, Eigen::Vector3d *p3d);

// Calculate depth triangulating two 3D vectors
bool GetDepthFromTriangulation(const SE3 &pose, const Eigen::Vector3d &v_ref, const Eigen::Vector3d &v_cur, double *depth);

// Parallax between 2 3D positions and a 3D point. Return cos alpha
double GetParallax(const Eigen::Vector3d &src1, const Eigen::Vector3d &src2, const Eigen::Vector3d &p3d);

// Get median element of a vector
double GetMedianVector(std::vector<double>* v);

// Distance between two 2D vectors
double Distance2D(const Eigen::Vector2d &v1, const Eigen::Vector2d &v2);

}  // namespace sdvl

#endif  // SDVL_EXTRA_UTILS_H_
