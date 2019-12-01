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

#include "eigen3/Eigen/Core"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "ros/ros.h"
#include "std_msgs/ColorRGBA.h"
#include "std_msgs/Header.h"
#include "visualization_msgs/Marker.h"

#ifndef ROS_HELPERS_H
#define ROS_HELPERS_H

namespace ros_helpers {

void InitRosHeader(const std::string& frame_id, std_msgs::Header* h);

void ClearMarker(visualization_msgs::Marker* m);

template<typename Tr, typename Tg, typename Tb, typename Ta>
std_msgs::ColorRGBA RosColor(const Tr& r,
                             const Tg& g,
                             const Tb& b,
                             const Ta& a) {
  std_msgs::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

template<typename Tr, typename Tg, typename Tb, typename Ta, typename RosColor>
void SetRosColor(const Tr& r,
                 const Tg& g,
                 const Tb& b,
                 const Ta& a,
                 RosColor* c) {
  c->r = r;
  c->g = g;
  c->b = b;
  c->a = a;
}

template<typename Tx, typename Ty, typename Tz>
geometry_msgs::Point RosPoint(const Tx& x, const Ty& y, const Tz& z) {
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

template<typename Tx, typename Ty>
geometry_msgs::Point RosPoint(const Tx& x, const Ty& y) {
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = 0;
  return p;
}

template<typename Tx, typename Ty, typename Tz, typename RosVector>
void SetRosVector(const Tx& x, const Ty& y, const Tz& z, RosVector* v) {
  v->x = x;
  v->y = y;
  v->z = z;
}

template<typename RosVector, typename T>
void SetRosQuaternion(const T& w,
                      const T& x,
                      const T& y,
                      const T& z,
                      RosVector* q) {
  q->w = w;
  q->x = x;
  q->y = y;
  q->z = z;
}

template<typename RosVector>
void SetIdentityRosQuaternion(RosVector* q) {
  q->w = 1;
  q->x = 0;
  q->y = 0;
  q->z = 0;
}

template <typename Derived>
geometry_msgs::Point Eigen3DToRosPoint(const Eigen::DenseBase<Derived>& v) {
  geometry_msgs::Point p;
  p.x = v[0];
  p.y = v[1];
  p.z = v[2];
  return p;
}

template <typename Derived>
geometry_msgs::Point Eigen2DToRosPoint(const Eigen::DenseBase<Derived>& v) {
  geometry_msgs::Point p;
  p.x = v[0];
  p.y = v[1];
  p.z = 0;
  return p;
}

}  // namespace ros_helpers

#endif  // ROS_HELPERS_H
