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

#include "./feature.h"

using std::shared_ptr;

namespace sdvl {

Feature::Feature(const shared_ptr<Frame> &f, const Eigen::Vector2d& p, int l) {
  frame_ = f;
  point_ = nullptr;
  p2d_ = p;
  v_ = frame_->GetCamera()->Unproject(p2d_);
  level_ = l;
  descriptor_.resize(32);
  has_descriptor_ = false;
}

Feature::Feature(const shared_ptr<Frame> &f, const shared_ptr<Point> &ft, const Eigen::Vector2d& p, int l) {
  frame_ = f;
  point_ = ft;
  p2d_ = p;
  v_ = frame_->GetCamera()->Unproject(p2d_);
  level_ = l;
  descriptor_.resize(32);
  has_descriptor_ = false;
}

Feature::Feature(const shared_ptr<Frame> &f, const shared_ptr<Point> &ft, const Eigen::Vector2d& p, const Eigen::Vector3d& v, int l) {
  frame_ = f;
  point_ = ft;
  p2d_ = p;
  v_ = v;
  level_ = l;
  descriptor_.resize(32);
  has_descriptor_ = false;
}

}  // namespace sdvl
