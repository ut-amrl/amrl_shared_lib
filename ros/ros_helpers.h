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

#ifndef ROS_HELPERS_H
#define ROS_HELPERS_H

#include <string>
#include "eigen3/Eigen/Core"

#ifdef ROS2
  // Use ROS2 headers
  #include "rclcpp/rclcpp.hpp"
  #include "std_msgs/msg/color_rgba.hpp"
  #include "std_msgs/msg/header.hpp"
  #include "geometry_msgs/msg/point.hpp"
  #include "visualization_msgs/msg/marker.hpp"
#else
  // Use ROS1 headers
  #include "ros/ros.h"
  #include "std_msgs/ColorRGBA.h"
  #include "std_msgs/Header.h"
  #include "geometry_msgs/Point.h"
  #include "visualization_msgs/Marker.h"
#endif

namespace ros_helpers {

#ifdef ROS2
  // Aliases for ROS2 types.
  using StdColorRGBA = std_msgs::msg::ColorRGBA;
  using StdHeader     = std_msgs::msg::Header;
  using GeoPoint      = geometry_msgs::msg::Point;
  using VisMarker     = visualization_msgs::msg::Marker;
#else
  // Aliases for ROS1 types.
  using StdColorRGBA = std_msgs::ColorRGBA;
  using StdHeader     = std_msgs::Header;
  using GeoPoint      = geometry_msgs::Point;
  using VisMarker     = visualization_msgs::Marker;
#endif

// Initializes a header with the specified frame_id and current time.
inline void InitRosHeader(const std::string& frame_id, StdHeader* h) {
#ifdef ROS2
  // Use a temporary clock since we don't have a node pointer here.
  rclcpp::Clock clock;
  h->stamp = clock.now();
#else
  h->stamp = ros::Time::now();
  h->seq = 0;
#endif
  h->frame_id = frame_id;
}

inline void ClearMarker(VisMarker* m) {
  m->points.clear();
  m->colors.clear();
}

template<typename Tr, typename Tg, typename Tb, typename Ta>
StdColorRGBA RosColor(const Tr& r, const Tg& g, const Tb& b, const Ta& a) {
  StdColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

template<typename Tr, typename Tg, typename Tb, typename Ta, typename RosColor>
void SetRosColor(const Tr& r, const Tg& g, const Tb& b, const Ta& a, RosColor* c) {
  c->r = r;
  c->g = g;
  c->b = b;
  c->a = a;
}

template<typename Tx, typename Ty, typename Tz>
GeoPoint RosPoint(const Tx& x, const Ty& y, const Tz& z) {
  GeoPoint p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

template<typename Tx, typename Ty>
GeoPoint RosPoint(const Tx& x, const Ty& y) {
  GeoPoint p;
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
void SetRosQuaternion(const T& w, const T& x, const T& y, const T& z, RosVector* q) {
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
GeoPoint Eigen3DToRosPoint(const Eigen::DenseBase<Derived>& v) {
  GeoPoint p;
  p.x = v[0];
  p.y = v[1];
  p.z = v[2];
  return p;
}

template <typename Derived>
GeoPoint Eigen2DToRosPoint(const Eigen::DenseBase<Derived>& v) {
  GeoPoint p;
  p.x = v[0];
  p.y = v[1];
  p.z = 0;
  return p;
}

template <typename Derived>
void DrawEigen2DLine(const Eigen::DenseBase<Derived>& v1,
                     const Eigen::DenseBase<Derived>& v2,
                     VisMarker* msg) {
  msg->points.push_back(Eigen2DToRosPoint<Derived>(v1));
  msg->points.push_back(Eigen2DToRosPoint<Derived>(v2));
}

template <typename Derived>
void DrawEigen2DLine(const Eigen::DenseBase<Derived>& v1,
                     const Eigen::DenseBase<Derived>& v2,
                     const StdColorRGBA& c,
                     VisMarker* msg) {
  msg->points.push_back(Eigen2DToRosPoint<Derived>(v1));
  msg->points.push_back(Eigen2DToRosPoint<Derived>(v2));
  msg->colors.push_back(c);
  msg->colors.push_back(c);
}

template <typename Derived>
void DrawEigen2DLine(const Eigen::DenseBase<Derived>& v1,
                     const Eigen::DenseBase<Derived>& v2,
                     const StdColorRGBA& c1,
                     const StdColorRGBA& c2,
                     VisMarker* msg) {
  msg->points.push_back(Eigen2DToRosPoint<Derived>(v1));
  msg->points.push_back(Eigen2DToRosPoint<Derived>(v2));
  msg->colors.push_back(c1);
  msg->colors.push_back(c2);
}

template <typename Vector2>
void DrawCross(const Vector2& v,
              const float size,
              const StdColorRGBA& color,
              VisMarker* msg) {
  msg->points.push_back(Eigen2DToRosPoint(v - Vector2(size, size)));
  msg->points.push_back(Eigen2DToRosPoint(v + Vector2(size, size)));
  msg->points.push_back(Eigen2DToRosPoint(v - Vector2(size, -size)));
  msg->points.push_back(Eigen2DToRosPoint(v + Vector2(size, -size)));
  msg->colors.push_back(color);
  msg->colors.push_back(color);
  msg->colors.push_back(color);
  msg->colors.push_back(color);
}

}  // namespace ros_helpers

#endif  // ROS_HELPERS_H
