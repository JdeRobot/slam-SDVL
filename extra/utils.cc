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

#include "./utils.h"

using std::vector;

namespace sdvl {

double AbsMax(const Eigen::VectorXd &v) {
  double max, abs;
  int size;

  size = v.size();
  max = -1;

  for (int i=0; i < size; i++) {
    abs = fabs(v(i));
    if (abs > max) {
      max = abs;
    }
  }
  return max;
}

float Interpolate8U(const cv::Mat& mat, float u, float v) {
  assert(mat.type() == CV_8U);
  int x = floor(u);
  int y = floor(v);
  float subpix_x = u-x;
  float subpix_y = v-y;

  float w00 = (1.0f-subpix_x)*(1.0f-subpix_y);
  float w01 = (1.0f-subpix_x)*subpix_y;
  float w10 = subpix_x*(1.0f-subpix_y);
  float w11 = 1.0f - w00 - w01 - w10;

  const int stride = mat.step.p[0];
  unsigned char* ptr = mat.data + y*stride + x;
  return w00*ptr[0] + w01*ptr[stride] + w10*ptr[1] + w11*ptr[stride+1];
}

double FindShiTomasiScoreAtPoint(const cv::Mat& img, int px, int py) {
  float dXX = 0.0;
  float dYY = 0.0;
  float dXY = 0.0;
  const int halfbox_size = 4;
  const int box_size = 2*halfbox_size;
  const int box_area = box_size*box_size;
  const int x_min = px-halfbox_size;
  const int x_max = px+halfbox_size;
  const int y_min = py-halfbox_size;
  const int y_max = py+halfbox_size;

  // Check borders
  if (x_min < 1 || x_max >= img.cols-1 || y_min < 1 || y_max >= img.rows-1)
    return 0.0;

  const int stride = img.step.p[0];
  for (int y=y_min; y < y_max; y++) {
    const uint8_t* ptr_left   = img.data + stride*y + x_min - 1;
    const uint8_t* ptr_right  = img.data + stride*y + x_min + 1;
    const uint8_t* ptr_top    = img.data + stride*(y-1) + x_min;
    const uint8_t* ptr_bottom = img.data + stride*(y+1) + x_min;
    for (int x = 0; x < box_size; x++, ptr_left++, ptr_right++, ptr_top++, ptr_bottom++) {
      float dx = *ptr_right - *ptr_left;
      float dy = *ptr_bottom - *ptr_top;
      dXX += dx*dx;
      dYY += dy*dy;
      dXY += dx*dy;
    }
  }

  // Find and return smaller eigenvalue:
  dXX = dXX / (2.0 * box_area);
  dYY = dYY / (2.0 * box_area);
  dXY = dXY / (2.0 * box_area);
  return 0.5 * (dXX + dYY - sqrt( (dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY) ));
}

void Jacobian3DToPlane(const Eigen::Vector3d &p, Eigen::Matrix<double, 2, 6> *J) {
  const double x = p(0);
  const double y = p(1);
  const double z_inv = 1./p(2);
  const double z_inv_2 = z_inv*z_inv;

  (*J)(0, 0) = -z_inv;                  // -1/z
  (*J)(0, 1) = 0.0;                     // 0
  (*J)(0, 2) = x*z_inv_2;               // x/z^2
  (*J)(0, 3) = y*(*J)(0, 2);            // x*y/z^2
  (*J)(0, 4) = -(1.0 + x*(*J)(0, 2));   // -(1.0 + x^2/z^2)
  (*J)(0, 5) = y*z_inv;                 // y/z

  (*J)(1, 0) = 0.0;                   // 0
  (*J)(1, 1) = -z_inv;                // -1/z
  (*J)(1, 2) = y*z_inv_2;             // y/z^2
  (*J)(1, 3) = 1.0 + y*(*J)(1, 2);    // 1.0 + y^2/z^2
  (*J)(1, 4) = -(*J)(0, 3);           // -x*y/z^2
  (*J)(1, 5) = -x*z_inv;              // x/z
}

void Jacobian3DToPlane(const Eigen::Vector3d &p, const Eigen::Matrix3d &R, Eigen::Matrix<double, 2, 3> *J) {
  const double z_inv = 1./p(2);
  const double z_inv_2 = z_inv*z_inv;

  (*J)(0, 0) = z_inv;
  (*J)(0, 1) = 0.0;
  (*J)(0, 2) = -p(0) * z_inv_2;
  (*J)(1, 0) = 0.0;
  (*J)(1, 1) = z_inv;
  (*J)(1, 2) = -p(1) * z_inv_2;
  (*J) = -(*J) * R;
}

Eigen::Vector3d Triangulate(const SE3 &pose, const Eigen::Vector3d &vector1, const Eigen::Vector3d &vector2) {
  Eigen::Vector3d res, xm, xn;
  Eigen::Vector2d b, lambda;
  Eigen::Vector3d v1, v2, T;
  Eigen::Matrix2d A;
  Eigen::Matrix3d R;

  T = pose.GetTranslation();
  R = pose.GetRotation();
  v1 = vector1;
  v2 = vector2;

  // Calculate intersection
  v2 = R * v2;
  b[0] = T.dot(v1);
  b[1] = T.dot(v2);
  A(0, 0) = v1.dot(v1);
  A(1, 0) = v1.dot(v2);
  A(0, 1) = -A(1, 0);
  A(1, 1) = -v2.dot(v2);
  lambda = A.inverse() * b;
  xm = lambda[0] * v1;
  xn = T + lambda[1] * v2;
  res = (xm + xn)/2;

  return res;
}

bool Triangulate(const SE3 &pose1, const SE3 &pose2, const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, Eigen::Vector3d *p3d) {
  Eigen::Matrix3d R1 = pose1.GetRotation();
  Eigen::Matrix3d R2 = pose2.GetRotation();
  Eigen::Vector3d T1 = pose1.GetTranslation();
  Eigen::Vector3d T2 = pose2.GetTranslation();

  cv::Mat RT1 = (cv::Mat_<float>(3, 4) << R1(0, 0), R1(0, 1), R1(0, 2), T1(0), R1(1, 0), R1(1, 1), R1(1, 2), T1(1), R1(2, 0), R1(2, 1), R1(2, 2), T1(2));
  cv::Mat RT2 = (cv::Mat_<float>(3, 4) << R2(0, 0), R2(0, 1), R2(0, 2), T2(0), R2(1, 0), R2(1, 1), R2(1, 2), T2(1), R2(2, 0), R2(2, 1), R2(2, 2), T2(2));
  cv::Mat v1m = (cv::Mat_<float>(3, 1) << v1(0), v1(1), v1(2));
  cv::Mat v2m = (cv::Mat_<float>(3, 1) << v2(0), v2(1), v2(2));
  v1m = v1m/v1m.at<float>(2);
  v2m = v2m/v2m.at<float>(2);

  cv::Mat w, u, vt;
  cv::Mat A(4, 4, CV_32F);
  A.row(0) = v1m.at<float>(0)*RT1.row(2)-RT1.row(0);
  A.row(1) = v1m.at<float>(1)*RT1.row(2)-RT1.row(1);
  A.row(2) = v2m.at<float>(0)*RT2.row(2)-RT2.row(0);
  A.row(3) = v2m.at<float>(1)*RT2.row(2)-RT2.row(1);

  cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A|cv::SVD::FULL_UV);

  cv::Mat x3D = vt.row(3).t();
  if (x3D.at<float>(3) == 0)
    return false;

  // Euclidean coordinates
  x3D = x3D.rowRange(0, 3)/x3D.at<float>(3);
  *p3d << x3D.at<float>(0), x3D.at<float>(1), x3D.at<float>(2);
  return true;
}

bool GetDepthFromTriangulation(const SE3 &pose, const Eigen::Vector3d &v_ref, const Eigen::Vector3d &v_cur, double *depth) {
  Eigen::Matrix2d AtA;
  Eigen::Matrix<double, 3, 2> A;

  A << pose.GetRotation() * v_ref, v_cur;
  AtA = A.transpose()*A;
  if (AtA.determinant() < 0.000001)
    return false;

  Eigen::Vector2d depth2 = -AtA.inverse()*A.transpose()*pose.GetTranslation();
  *depth = fabs(depth2[0]);
  return true;
}

double GetParallax(const Eigen::Vector3d &src1, const Eigen::Vector3d &src2, const Eigen::Vector3d &p3d) {
  Eigen::Vector3d v1 = src1 - p3d;
  Eigen::Vector3d v2 = src2 - p3d;
  v1.normalize();
  v2.normalize();
  return v1.dot(v2);
}

double GetMedianVector(vector<double>* v) {
  assert(!v->empty());
  vector<double>::iterator it = v->begin()+floor(v->size()/2);
  nth_element(v->begin(), it, v->end());
  return *it;
}

double Distance2D(const Eigen::Vector2d &v1, const Eigen::Vector2d &v2) {
  double d1 = v1(0)-v2(0);
  double d2 = v1(1)-v2(1);
  return sqrt(d1*d1+d2*d2);
}

}  // namespace sdvl



