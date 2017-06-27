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

#ifndef SDVL_EXTRA_BUNDLE_H_
#define SDVL_EXTRA_BUNDLE_H_

#include <iostream>
#include <memory>
#include <vector>
#include "../map.h"
#include "./se3.h"

namespace sdvl {

struct BAFeature {
  std::shared_ptr<Feature> feature;
  int index_frame;
  int index_point;
};

class Bundle {
 public:
  explicit Bundle(Map *map);

  inline int GetID() const { return id_; }

  // Perform local bundle adjustment
  void Local(const std::vector<std::shared_ptr<Frame>> &kfs);

 private:
  Map * map_;

  int id_;                // BA unique id
  static int counter_;    // Counter to set BA id

  int index_frame_;
  int index_point_;
  int index_feature_;
};

}  // namespace sdvl


#endif  // SDVL_EXTRA_BUNDLE_H_
