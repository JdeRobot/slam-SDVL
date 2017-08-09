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

#include <exception>
#include "./bundle.h"
#include "extra/g2o/g2o/core/block_solver.h"
#include "extra/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "extra/g2o/g2o/solvers/linear_solver_eigen.h"
#include "extra/g2o/g2o/types/types_six_dof_expmap.h"
#include "extra/g2o/g2o/core/robust_kernel_impl.h"
#include "extra/g2o/g2o/solvers/linear_solver_dense.h"

using std::shared_ptr;
using std::vector;
using std::list;
using std::cout;
using std::endl;

namespace sdvl {

int Bundle::counter_ = 0;

Bundle::Bundle(Map *map) {
  id_ = counter_;
  map_ = map;
  index_frame_ = 0;
  index_point_ = 0;
  index_feature_ = 0;

  counter_ += 1;
}

g2o::SE3Quat toSE3Quat(const SE3 &pose) {
  Eigen::Matrix3d R = pose.GetRotation();
  Eigen::Vector3d t = pose.GetTranslation();
  return g2o::SE3Quat(R, t);
}

SE3 toSE3(const g2o::SE3Quat &se3q) {
  SE3 se3;
  Eigen::Matrix<double, 4, 4> eigMat = se3q.to_homogeneous_matrix();
  se3.SetRotation(eigMat.block(0, 0, 3, 3));
  se3.SetTranslation(Eigen::Vector3d(eigMat(0, 3), eigMat(1, 3), eigMat(2, 3)));
  return se3;
}

void Bundle::Local(const vector<shared_ptr<Frame>> &kfs) {
  long maxID = 0;
  int size;
  vector<shared_ptr<Point>> lpoints;
  vector<shared_ptr<Frame>> fixed_kfs;

  // Get all points involved
  for (auto it=kfs.begin(); it != kfs.end(); it++) {
    (*it)->SetLastBA(id_);

    vector<shared_ptr<Feature>> &cfeatures = (*it)->GetFeatures();
    for (auto it_fts=cfeatures.begin(); it_fts != cfeatures.end(); it_fts++) {
      shared_ptr<Point> point = (*it_fts)->GetPoint();
      if (!point || point->ToDelete())
        continue;

      // Skip if it has been already saved
      if (point->GetLastBA() != id_) {
        point->SetLastBA(id_);
        lpoints.push_back(point);
      }
    }
  }

  cout << "[DEBUG] BA total points: " << lpoints.size() << endl;

  if (lpoints.empty())
    return;

  // Get keyframes that see these points and save as fixed
  for (auto it=lpoints.begin(); it != lpoints.end(); it++) {
    list<shared_ptr<Feature>>& cfeatures = (*it)->GetFeatures();

    for (auto it_fts=cfeatures.begin(); it_fts != cfeatures.end(); it_fts++) {
      shared_ptr<Frame> frame = (*it_fts)->GetFrame();

      if (!frame->ToDelete() && frame->GetLastBA() != id_) {
        frame->SetLastBA(id_);
        fixed_kfs.push_back(frame);
      }
    }
  }

  cout << "[DEBUG] BA total fixed frames: " << fixed_kfs.size() << endl;

  // Configure BA
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolver_6_3::LinearSolverType * linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
  g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);

  // Set Local KeyFrame vertices
  for (auto it=kfs.begin(); it != kfs.end(); it++) {
    int id = (*it)->GetID();
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(toSE3Quat((*it)->GetPose()));
    vSE3->setId(id);
    vSE3->setFixed(id == 0);
    optimizer.addVertex(vSE3);
    if (id > maxID)
        maxID = id;
  }

  // Set Fixed KeyFrame vertices
  for (auto it=fixed_kfs.begin(); it != fixed_kfs.end(); it++) {
    int id = (*it)->GetID();
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(toSE3Quat((*it)->GetPose()));
    vSE3->setId(id);
    vSE3->setFixed(true);
    optimizer.addVertex(vSE3);
    if (id > maxID)
        maxID = id;
  }

  // Set vertices
  size = (kfs.size()+fixed_kfs.size())*lpoints.size();
  vector<g2o::EdgeSE3ProjectXYZ*> projections;
  vector<shared_ptr<Frame>> kf_edges;
  vector<shared_ptr<Point>> points_edges;
  projections.reserve(size);
  kf_edges.reserve(size);
  points_edges.reserve(size);

  float thHuber = sqrt(5.991);
  for (auto it=lpoints.begin(); it != lpoints.end(); it++) {
    g2o::VertexSBAPointXYZ* vertex = new g2o::VertexSBAPointXYZ();
    vertex->setEstimate((*it)->GetPosition());
    int id = (*it)->GetID()+maxID+1;
    vertex->setId(id);
    vertex->setMarginalized(true);
    optimizer.addVertex(vertex);

    list<shared_ptr<Feature>>& cfeatures = (*it)->GetFeatures();
    // Set edges
    for (auto it_fts=cfeatures.begin(); it_fts != cfeatures.end(); it_fts++) {
      shared_ptr<Frame> frame = (*it_fts)->GetFrame();

      if (frame->ToDelete())
        continue;

      if(optimizer.vertex(id) == nullptr || optimizer.vertex(frame->GetID()) == nullptr)
        continue;

      g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(frame->GetID())));
      e->setMeasurement((*it_fts)->GetPosition());
      e->setInformation(Eigen::Matrix2d::Identity());

      g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
      e->setRobustKernel(rk);
      rk->setDelta(thHuber);

      e->fx = frame->GetCamera()->GetFx();
      e->fy = frame->GetCamera()->GetFy();
      e->cx = frame->GetCamera()->GetU0();
      e->cy = frame->GetCamera()->GetV0();

      optimizer.addEdge(e);
      projections.push_back(e);
      kf_edges.push_back(frame);
      points_edges.push_back(*it);
    }
  }

  optimizer.initializeOptimization();
  optimizer.optimize(5);

  // Check inliers
  for (size_t i=0, iend=projections.size(); i < iend; i++) {
      g2o::EdgeSE3ProjectXYZ* e = projections[i];
      shared_ptr<Point> point = points_edges[i];
      if (point->ToDelete())
          continue;

      if (e->chi2() > 5.991 || !e->isDepthPositive())
          e->setLevel(1);
      e->setRobustKernel(0);
  }

  // Optimize again without outliers
  optimizer.initializeOptimization(0);
  optimizer.optimize(10);

  // Save Keyframes
  for (auto it=kfs.begin(); it != kfs.end(); it++) {
    g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex((*it)->GetID()));
    g2o::SE3Quat SE3quat = vSE3->estimate();
    (*it)->SetPose(toSE3(SE3quat));
  }

  // Save Points
  for (auto it=lpoints.begin(); it != lpoints.end(); it++) {
    g2o::VertexSBAPointXYZ* vertex = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex((*it)->GetID()+maxID+1));
    Eigen::Vector3d p3d = vertex->estimate();
    (*it)->SetPosition(p3d);
  }
}

}  // namespace sdvl
