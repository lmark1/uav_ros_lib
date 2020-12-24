# UAV ROS lib

| Ubuntu 18.04  | Ubuntu 20.04|
|---------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------|
 [![Melodic](https://github.com/lmark1/uav_ros_lib/workflows/Melodic/badge.svg)](https://github.com/lmark1/uav_ros_lib/actions) | [![Noetic](https://github.com/lmark1/uav_ros_lib/workflows/Noetic/badge.svg)](https://github.com/lmark1/uav_ros_lib/actions) |

## Summary

A collection of useful libraries for the [uav_ros_stack](https://github.com/lmark1/uav_ros_stack).

## Description

### General
* **[param_util namespace](https://github.com/lmark1/uav_ros_lib/include/uav_ros_lib/param_util.hpp)** - Useful functions for loading values from the ROS parameter server
* **[nonlinear_filters namespace](https://github.com/lmark1/uav_ros_lib/include/uav_ros_lib/nonlinear_filters.hpp)** - Implementation of simple, commonly used nonlinear filters
* **[TopicHandler](https://github.com/lmark1/uav_ros_lib/include/uav_ros_lib/topic_handler.hpp)** - A helper class for handling ROS message subscriptions
* **[ReconfigureHandler](https://github.com/lmark1/uav_ros_lib/include/uav_ros_lib/reconfigure_handler.hpp)** - A helper class for handling ROS reconfigure server

## Estimation
* **[KalmanFilterVanilla](include/uav_ros_lib/kalman_filter_vanilla.hpp)** - A straightforward implementation of a single measurement Discrete Kalman Filter