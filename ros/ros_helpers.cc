// Copyright 2019 joydeepb@cs.utexas.edu
// Computer Science Department
// University of Texas at Austin
//
//
// This software is free: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3,
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
// ========================================================================

#include <string>

#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "visualization_msgs/Marker.h"

#include "ros_helpers.h"

namespace ros_helpers {

void InitRosHeader(const std::string& frame_id, std_msgs::Header* h) {
  h->seq = 0;
  h->frame_id = frame_id;
  h->stamp = ros::Time::now();
}

void ClearMarker(visualization_msgs::Marker* m) {
  m->points.clear();
  m->colors.clear();
}

}  // namespace ros_helpers
