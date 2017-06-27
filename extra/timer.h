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

#ifndef SDVL_EXTRA_TIMER_H_
#define SDVL_EXTRA_TIMER_H_

#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>

namespace sdvl {

class Timer {
 public:
  explicit Timer(bool autostart) : time_(0.0) {
    if (autostart)
      Start();
  }

  inline double GetTime() {
    return time_;
  }

  inline double GetMsTime() {
    return time_*1000.0;
  }

  inline void Start() {
    gettimeofday(&start_time_, NULL);
  }


  inline void Stop() {
    timeval end_time;
    gettimeofday(&end_time, NULL);
    long seconds  = end_time.tv_sec  - start_time_.tv_sec;
    long useconds = end_time.tv_usec - start_time_.tv_usec;
    time_ = ((seconds) + useconds*0.000001);
  }

 private:
  timeval start_time_;
  double time_;
};

}  // namespace sdvl


#endif  // SDVL_EXTRA_TIMER_H_
