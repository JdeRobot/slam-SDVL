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

#include "./homography_init.h"
#include "./point.h"
#include "./feature.h"
#include "./config.h"
#include "extra/utils.h"

using std::shared_ptr;
using std::vector;
using std::cout;
using std::cerr;
using std::endl;

namespace sdvl {

HomographyInit::HomographyInit(Map * map, int min_shift) {
  map_ = map;
  min_shift_ = min_shift;
}

void HomographyInit::Reset() {
  pixels1_.clear();
  pixels2_.clear();
  vectors1_.clear();
  vectors2_.clear();
  frame1_ = nullptr;
  frame2_ = nullptr;
}

bool HomographyInit::InitFirstFrame(const shared_ptr<Frame> &frame) {
  shared_ptr<Feature> feature;
  Eigen::Vector3i corner;
  int index, scale;

  frame1_ = frame;

  // Filter corners
  frame1_->FilterCorners();
  vector<Eigen::Vector3i> &corners = frame1_->GetCorners();
  vector<int> &fcorners = frame1_->GetFilteredCorners();

  pixels1_.clear();
  vectors1_.clear();
  for (auto it=fcorners.begin(); it != fcorners.end(); it++) {
    index = *it;
    corner = corners[index];
    scale = (1 << corner(2));

    feature = std::make_shared<Feature>(frame1_, Eigen::Vector2d(corner(0)*scale, corner(1)*scale), corner(2));
    pixels1_.push_back(cv::Point2f(feature->GetPosition()[0], feature->GetPosition()[1]));
    vectors1_.push_back(feature->GetVector());
  }

  cout << "[INFO] Homography Init: Detected " << pixels1_.size() << " features" << endl;
  if (static_cast<int>(pixels1_.size()) < Config::MinInitCorners())
    return false;

  // Optical flow will search starting from pixels2_ values
  pixels2_.insert(pixels2_.begin(), pixels1_.begin(), pixels1_.end());
  return true;
}

bool HomographyInit::InitSecondFrame(const shared_ptr<Frame> &frame) {
  Camera * camera;
  vector<double> depths;
  double depth, scale;
  Eigen::Vector2d p1, p2;
  int nfeatures, dist, margin;
  shared_ptr<Feature> feature1, feature2;
  bool fixed = false;

  frame2_ = frame;
  camera = frame2_->GetCamera();
  nfeatures = 0;

  if (!TrackSecondFrame())
    return false;

  if (!ComputeHomography())
    return false;

  if (triangulations_.empty())
    return false;

  // Set margin
  if (Config::UseORB())
    margin = 4+Config::ORBSize()/2;
  else
    margin = 1+Config::PatchSize()/2;

  // Rescale map
  for (unsigned int i=0; i < triangulations_.size(); ++i)
    depths.push_back((triangulations_[i]).z());

  depth = GetMedianVector(&depths);
  scale = Config::MapScale()/depth;

  // Save pose
  frame2_->SetPose(se3_ * frame1_->GetPose());
  Eigen::Vector3d translation = frame1_->GetWorldPosition() + scale*(frame2_->GetWorldPosition() - frame1_->GetWorldPosition());
  frame2_->GetPose().SetTranslation(-frame2_->GetPose().GetRotation()*translation);

  // Save features
  for (vector<int>::iterator it=inliers_.begin(); it != inliers_.end(); it++) {
    p1 << pixels1_[*it].x, pixels1_[*it].y;
    p2 << pixels2_[*it].x, pixels2_[*it].y;

    if (!camera->IsInsideImage(p1.cast<int>(), margin) || !camera->IsInsideImage(p2.cast<int>(), margin) || triangulations_[*it](2) < 0)
      continue;

    // Get depth
    SE3 pose = frame2_->GetPose() * frame1_->GetPose().Inverse();
    if (!GetDepthFromTriangulation(pose, vectors1_[*it], vectors2_[*it], &depth))
      continue;

    // Create new point
    shared_ptr<Point> candidate = std::make_shared<Point>();
    feature1 = std::make_shared<Feature>(frame1_, candidate, p1, vectors1_[*it], 0);
    feature2 = std::make_shared<Feature>(frame2_, candidate, p2, vectors2_[*it], 0);

    if (Config::UseORB()) {
      vector<uchar> desc(32);

      // Get descriptors
      detector_.GetDescriptor(frame1_->GetPyramid()[0], p1.cast<int>(), &desc);
      feature1->SetDescriptor(desc);

      detector_.GetDescriptor(frame2_->GetPyramid()[0], p2.cast<int>(), &desc);
      feature2->SetDescriptor(desc);

      dist = detector_.Distance(feature1->GetDescriptor(), feature2->GetDescriptor());
      if (dist > MIN_ORB_THRESHOLD)
        continue;
    }

    // Link to first frame
    candidate->InitCandidate(feature1, depth);
    frame1_->AddFeature(feature1);
    candidate->AddFeature(feature1);

    // Link to second frame
    frame2_->AddFeature(feature2);
    candidate->AddFeature(feature2);

    if (fixed) {
      // Set fixed position
      Eigen::Vector3d pos = candidate->GetPosition();
      candidate->SetFixed();
      candidate->SetPosition(pos);
    } else {
      map_->AddCandidate(candidate);
    }

    nfeatures++;
  }

  // Connect frames
  frame1_->AddConnection(std::make_pair(frame2_, nfeatures));
  frame2_->AddConnection(std::make_pair(frame1_, nfeatures));

  cout << "[INFO] Initial Map created with " << nfeatures << " features" << endl;
  return nfeatures >= Config::MinInitCorners();
}

bool HomographyInit::TrackSecondFrame() {
  Camera * camera = frame2_->GetCamera();
  vector<uchar> status;
  vector<float> error;
  vector<double> shifts;
  Eigen::Vector2d pos;
  Eigen::Vector3d pos3d;
  double shift;
  int size;

  cv::TermCriteria criteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.001);
  cv::calcOpticalFlowPyrLK(frame1_->GetPyramid()[0], frame2_->GetPyramid()[0],
                           pixels1_, pixels2_, status, error,
                           cv::Size2i(30.0, 30.0), 4, criteria, cv::OPTFLOW_USE_INITIAL_FLOW);

  vector<cv::Point2f>::iterator it_pixels1 = pixels1_.begin();
  vector<cv::Point2f>::iterator it_pixels2 = pixels2_.begin();
  vector<Eigen::Vector3d>::iterator it_vectors1 = vectors1_.begin();
  vectors2_.clear();
  for (vector<uchar>::iterator it_status=status.begin(); it_status != status.end(); it_status++) {
    if (!(*it_status)) {
      it_pixels1 = pixels1_.erase(it_pixels1);
      it_pixels2 = pixels2_.erase(it_pixels2);
      it_vectors1 = vectors1_.erase(it_vectors1);
      continue;
    }

    // Save 3d vector
    pos << it_pixels2->x, it_pixels2->y;
    camera->Unproject(pos, &pos3d);
    vectors2_.push_back(pos3d);

    // Compute shift between features
    shifts.push_back(Eigen::Vector2d(it_pixels1->x - it_pixels2->x, it_pixels1->y - it_pixels2->y).norm());

    it_pixels1++;
    it_pixels2++;
    it_vectors1++;
  }

  if (shifts.empty())
    return false;

  shift = GetMedianVector(&shifts);
  size = pixels1_.size();

  cout << "[DEBUG] Homography Init: Tracked " << size << " features" << endl;
  cout << "[DEBUG] Average shift between features is " << shift << "px " << endl;

  return shift >= min_shift_ && size >= Config::MinInitCorners();
}

bool HomographyInit::ComputeHomography() {
  Eigen::Vector2d pos1, pos2;
  double focal_length = frame1_->GetCamera()->GetFx();

  assert(vectors1_.size() == vectors2_.size());

  vector<cv::Point2f> src_pts(vectors1_.size());
  vector<cv::Point2f> dst_pts(vectors1_.size());
  for (unsigned int i=0; i < vectors1_.size(); i++) {
    pos1 = Camera::SimpleProject(vectors1_[i]);
    pos2 = Camera::SimpleProject(vectors2_[i]);
    src_pts[i] = cv::Point2f(pos1(0), pos1(1));
    dst_pts[i] = cv::Point2f(pos2(0), pos2(1));
  }

  // Get homography
  cv::Mat cvH = cv::findHomography(src_pts, dst_pts, CV_RANSAC, 2./focal_length);
  bestH_(0, 0) = cvH.at<double>(0, 0);
  bestH_(0, 1) = cvH.at<double>(0, 1);
  bestH_(0, 2) = cvH.at<double>(0, 2);
  bestH_(1, 0) = cvH.at<double>(1, 0);
  bestH_(1, 1) = cvH.at<double>(1, 1);
  bestH_(1, 2) = cvH.at<double>(1, 2);
  bestH_(2, 0) = cvH.at<double>(2, 0);
  bestH_(2, 1) = cvH.at<double>(2, 1);
  bestH_(2, 2) = cvH.at<double>(2, 2);

  // Decompose homography
  if (!DecomposeHomography(bestH_))
    return false;
  CheckInliers(bestH_, src_pts, dst_pts);
  ChooseBestDecomposition(src_pts, dst_pts);

  // Save decomposition
  assert(decompositions_.size() == 1);
  se3_ = decompositions_[0].se3;

  // Check matches with calculated decomposition
  CheckDecompositionInliers();
  if (static_cast<int>(inliers_.size()) < Config::MinInitCorners()) {
    cerr << "[ERROR] Homography Init: Not enough inliers" << endl;
    return false;
  }

  return true;
}

void HomographyInit::CheckInliers(const Eigen::Matrix3d &H, const vector<cv::Point2f> &src, const vector<cv::Point2f> &dst) {
  int size;

  inliers_.clear();
  size = src.size();

  for (int i=0; i < size; i++) {
    Eigen::Vector3d projection = bestH_ * Camera::SimpleUnproject(Eigen::Vector2d(src[i].x, src[i].y));
    Eigen::Vector2d projection2d = Camera::SimpleProject(projection);
    Eigen::Vector2d v2Error = Eigen::Vector2d(dst[i].x, dst[i].y) - projection2d;
    if (v2Error.norm() <= Config::InlierErrorThreshold())
      inliers_.push_back(i);
  }
}

void HomographyInit::CheckDecompositionInliers() {
  int size;
  Camera * camera;
  Eigen::Vector3d xyz;
  double err1, err2, ratio;

  inliers_.clear();
  triangulations_.clear();
  size = vectors1_.size();
  camera = frame2_->GetCamera();
  ratio = camera->GetFx();

  Eigen::Vector3d translation = se3_.GetTranslation();
  Eigen::Matrix3d rotation = se3_.GetRotation();

  // Triangulate each match and check reprojection error
  // Note: Triangulation from frame2 point of view
  for (int i=0; i < size; i++) {
    xyz = Triangulate(se3_, vectors2_[i], vectors1_[i]);
    triangulations_.push_back(xyz);

    // Calc reprojection errors
    err1 = ReprojectionError(camera, vectors2_[i], xyz)*ratio;
    err2 = ReprojectionError(camera, rotation, translation, vectors1_[i], xyz)*ratio;

    if (err1 <= Config::InlierErrorThreshold() && err2 <= Config::InlierErrorThreshold())
      inliers_.push_back(i);
  }
}

bool HomographyInit::DecomposeHomography(const Eigen::Matrix3d &H) {
  decompositions_.clear();

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Vector3d v3Diag = svd.singularValues();

  double d1 = fabs(v3Diag(0));  // The paper suggests the square of these (e.g. the evalues of AAT)
  double d2 = fabs(v3Diag(1));  // should be used, but this is wrong. c.f. Faugeras' book.
  double d3 = fabs(v3Diag(2));

  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();

  double s = U.determinant() * V.determinant();
  double dPrime_PM = d2;

  int nCase;
  if (d1 != d2 && d2 != d3)
    nCase = 1;
  else if (d1 == d2 && d2 == d3)
    nCase = 3;
  else
    nCase = 2;

  if (nCase != 1) {
    cerr << "[ERROR] Homography Init: This motion case is not implemented or is degenerate. Try again. " << endl;
    return false;
  }

  double x1_PM;
  double x2;
  double x3_PM;

  // All below deals with the case = 1 case.
  // Case 1 implies (d1 != d3)
  { // Eq. 12
    x1_PM = sqrt((d1 * d1 - d2 * d2) / (d1 * d1 - d3 * d3));
    x2 = 0;
    x3_PM = sqrt((d2 * d2 - d3 * d3) / (d1 * d1 - d3 * d3));
  }

  double e1[4] = { 1.0, -1.0, 1.0, -1.0 };
  double e3[4] = { 1.0, 1.0, -1.0, -1.0 };

  double e1_Eigen[4] = { 1.0, -1.0, 1.0, -1.0 };
  double e3_Eigen[4] = { 1.0, 1.0, -1.0, -1.0 };

  Eigen::Vector3d v3np;
  HomographyDecomposition decomposition;

  // Case 1, d' > 0:
  decomposition.d = s * dPrime_PM;
  for (int signs = 0; signs < 4; signs++) {
    // Eq 13
    decomposition.m3Rp = Eigen::Matrix3d::Identity();

    double dSinTheta = (d1 - d3) * x1_PM * x3_PM * e1[signs] * e3[signs] / d2;
    double dCosTheta = (d1 * x3_PM * x3_PM + d3 * x1_PM * x1_PM) / d2;
    decomposition.m3Rp(0, 0) = dCosTheta;
    decomposition.m3Rp(0, 2) = -dSinTheta;
    decomposition.m3Rp(2, 0) = dSinTheta;
    decomposition.m3Rp(2, 2) = dCosTheta;

    // Eq 14
    decomposition.v3Tp(0) = (d1 - d3) * x1_PM * e1_Eigen[signs];
    decomposition.v3Tp(1) = 0.0;
    decomposition.v3Tp(2) = (d1 - d3) * -x3_PM * e3_Eigen[signs];

    v3np(0) = x1_PM * e1[signs];
    v3np(1) = x2;
    v3np(2) = x3_PM * e3[signs];

    decomposition.v3n = V * v3np;

    decompositions_.push_back(decomposition);
  }

  // Case 1, d' < 0:
  decomposition.d = s * -dPrime_PM;
  for (int signs = 0; signs < 4; signs++) {
    // Eq 15
    decomposition.m3Rp = -1 * Eigen::Matrix3d::Identity();

    double dSinPhi = (d1 + d3) * x1_PM * x3_PM * e1[signs] * e3[signs] / d2;
    double dCosPhi = (d3 * x1_PM * x1_PM - d1 * x3_PM * x3_PM) / d2;
    decomposition.m3Rp(0, 0) = dCosPhi;
    decomposition.m3Rp(0, 2) = dSinPhi;
    decomposition.m3Rp(2, 0) = dSinPhi;
    decomposition.m3Rp(2, 2) = -dCosPhi;

    // Eq 16
    decomposition.v3Tp(0) = (d1 + d3) * x1_PM * e1_Eigen[signs];
    decomposition.v3Tp(1) = 0.0;
    decomposition.v3Tp(2) = (d1 + d3) * x3_PM * e3_Eigen[signs];

    v3np(0) = x1_PM * e1[signs];
    v3np(1) = x2;
    v3np(2) = x3_PM * e3[signs];

    decomposition.v3n = V * v3np;

    decompositions_.push_back(decomposition);
  }

  // While we have the SVD results calculated here, store the decomposition R and t results as well..
  for (unsigned int i = 0; i < decompositions_.size(); i++) {
    Eigen::Matrix3d rotation = s * U * decompositions_[i].m3Rp * V.transpose();
    Eigen::Vector3d translation = U * decompositions_[i].v3Tp;

    decompositions_[i].se3.SetRotation(rotation);
    decompositions_[i].se3.SetTranslation(translation);
  }

  return true;
}

bool operator<(const HomographyDecomposition lhs, const HomographyDecomposition rhs) {
  return lhs.score < rhs.score;
}

bool HomographyInit::ChooseBestDecomposition(const vector<cv::Point2f> &src, const vector<cv::Point2f> &dst) {
  int size;

  assert(decompositions_.size() == 8);
  size = src.size();

  // Select 4 decompositions
  for (unsigned int i = 0; i < decompositions_.size(); i++) {
    HomographyDecomposition &decom = decompositions_[i];
    int nPositive = 0;
    for (vector<int>::iterator it=inliers_.begin(); it != inliers_.end(); it++) {
      Eigen::Vector2d v2(src[*it].x, src[*it].y);
      double dVisibilityTest = (bestH_(2, 0) * v2(0) + bestH_(2, 1) * v2(1) + bestH_(2, 2)) / decom.d;
      if (dVisibilityTest > 0.0)
        nPositive++;
    }
    decom.score = -nPositive;
  }

  sort(decompositions_.begin(), decompositions_.end());
  decompositions_.resize(4);

  // Select 2 decompositions
  for (unsigned int i = 0; i < decompositions_.size(); i++) {
    HomographyDecomposition &decom = decompositions_[i];
    int nPositive = 0;
    for (vector<int>::iterator it=inliers_.begin(); it != inliers_.end(); it++) {
      Eigen::Vector2d v2(src[*it].x, src[*it].y);
      Eigen::Vector3d v3 = Camera::SimpleUnproject(v2);
      double dVisibilityTest = v3.dot(decom.v3n) / decom.d;
      if (dVisibilityTest > 0.0)
        nPositive++;
    }
    decom.score = -nPositive;
  }

  sort(decompositions_.begin(), decompositions_.end());
  decompositions_.resize(2);

  // According to Faugeras and Lustman, ambiguity exists if the two scores are equal
  // but in practive, better to look at the ratio!
  double dRatio = static_cast<double>(decompositions_[1].score) / static_cast<double>(decompositions_[0].score);

  if (dRatio < 0.9) {  // no ambiguity!
    decompositions_.erase(decompositions_.begin() + 1);
  } else {             // two-way ambiguity. Resolve by sampsonus score of all points.
    double dErrorSquaredLimit = Config::InlierErrorThreshold() * Config::InlierErrorThreshold() * 4;
    double adSampsonusScores[2];
    for (int i = 0; i < 2; i++) {
      SE3 & se3 = decompositions_[i].se3;
      Eigen::Matrix3d m3Essential;
      for (int j = 0; j < 3; j++) {
        Eigen::Vector3d trans = se3.GetTranslation();
        Eigen::Matrix3d rot = se3.GetRotation();
        Eigen::Vector3d rot_T;
        rot_T(0) = rot(0, j);
        rot_T(1) = rot(1, j);
        rot_T(2) = rot(2, j);

        Eigen::Vector3d sol = trans.cross(rot_T);

        m3Essential(0, j) = sol(0);
        m3Essential(1, j) = sol(1);
        m3Essential(2, j) = sol(2);
      }

      double dSumError = 0;
      for (int m = 0; m < size; m++) {
        double d = SampsonusError(m3Essential, m, src, dst);
        if (d > dErrorSquaredLimit)
          d = dErrorSquaredLimit;
        dSumError += d;
      }

      adSampsonusScores[i] = dSumError;
    }

    if (adSampsonusScores[0] <= adSampsonusScores[1])
      decompositions_.erase(decompositions_.begin() + 1);
    else
      decompositions_.erase(decompositions_.begin());
  }

  return true;
}

double HomographyInit::SampsonusError(const Eigen::Matrix3d &m3Essential, int i,
                                      const vector<cv::Point2f> &src, const vector<cv::Point2f> &dst) {
  Eigen::Vector2d v2(src[i].x, src[i].y);
  Eigen::Vector2d v2Dash(dst[i].x, dst[i].y);

  Eigen::Vector3d v3Dash = Camera::SimpleUnproject(v2Dash);
  Eigen::Vector3d v3 = Camera::SimpleUnproject(v2);

  Eigen::Vector3d aux = (m3Essential * v3);

  double dError = aux.dot(v3Dash);

  Eigen::Vector3d fv3 = m3Essential * v3;
  Eigen::Vector3d fTv3Dash = m3Essential.transpose() * v3Dash;

  Eigen::Vector2d fv3Slice;
  fv3Slice(0) = fv3(0);
  fv3Slice(1) = fv3(1);

  Eigen::Vector2d fTv3DashSlice;
  fTv3DashSlice(0) = fTv3Dash(0);
  fTv3DashSlice(1) = fTv3Dash(1);

  return (dError * dError / (fv3Slice.dot(fv3Slice) + fTv3DashSlice.dot(fTv3DashSlice)));
}

double HomographyInit::ReprojectionError(Camera * camera, const Eigen::Vector3d vector, const Eigen::Vector3d p3d) {
  Eigen::Vector2d perror = Camera::SimpleProject(vector) - Camera::SimpleProject(p3d);
  return perror.norm();
}

double HomographyInit::ReprojectionError(Camera * camera, const Eigen::Matrix3d &R, const Eigen::Vector3d &T,
                                         const Eigen::Vector3d vector, const Eigen::Vector3d p3d) {
  Eigen::Vector3d pcamera = R.transpose()*(p3d - T);
  Eigen::Vector2d perror = Camera::SimpleProject(vector) - Camera::SimpleProject(pcamera);
  return perror.norm();
}

}  // namespace sdvl
