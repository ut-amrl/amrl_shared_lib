#pragma once

#ifdef ROS1
  #include <ros/ros.h>
  #define LOG_ERROR(...) ROS_ERROR(__VA_ARGS__)
  #define LOG_WARN(...)  ROS_WARN(__VA_ARGS__)
  #define LOG_INFO(...)  ROS_INFO(__VA_ARGS__)
  #define GET_TIME()     ros::Time::now()
  #define DURATION(secs) ros::Duration(secs)
#else
  #include <rclcpp/rclcpp.hpp>
  // Define a default logger. You can either hard-code a logger name...
  #define DEFAULT_LOGGER rclcpp::get_logger("default_logger")
  
  #define LOG_ERROR(...) RCLCPP_ERROR(DEFAULT_LOGGER, __VA_ARGS__)
  #define LOG_WARN(...)  RCLCPP_WARN(DEFAULT_LOGGER, __VA_ARGS__)
  #define LOG_INFO(...)  RCLCPP_INFO(DEFAULT_LOGGER, __VA_ARGS__)
  #define GET_TIME()     rclcpp::Clock().now()
  #define DURATION(secs) rclcpp::Duration::from_seconds(secs)
#endif