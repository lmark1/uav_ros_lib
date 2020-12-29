# UAV ROS lib

| Ubuntu 18.04  | Ubuntu 20.04| Doxygen |
|---------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------|
 [![Melodic](https://github.com/lmark1/uav_ros_lib/workflows/Melodic/badge.svg)](https://github.com/lmark1/uav_ros_lib/actions) | [![Noetic](https://github.com/lmark1/uav_ros_lib/workflows/Noetic/badge.svg)](https://github.com/lmark1/uav_ros_lib/actions) | [![Docs](https://github.com/lmark1/uav_ros_lib/workflows/Docs/badge.svg)](https://github.com/lmark1/uav_ros_lib/actions) |

## Summary

A collection of useful libraries for the [uav_ros_stack](https://github.com/lmark1/uav_ros_stack).

## Documentation

Documentation can be found at [lmark1.github.io/uav_ros_lib](https://lmark1.github.io/uav_ros_lib/).
## General

### Namespaces
* **[param_util](https://github.com/lmark1/uav_ros_lib/tree/main/include/uav_ros_lib/param_util.hpp)** - Useful functions for loading values from the ROS parameter server
* **[nonlinear_filters](https://github.com/lmark1/uav_ros_lib/tree/main/include/uav_ros_lib/nonlinear_filters.hpp)** - Implementation of simple, commonly used nonlinear filters
* **[ros_convert](https://github.com/lmark1/uav_ros_lib/blob/main/include/uav_ros_lib/ros_convert.hpp)** - Useful functions for conversion to and from ROS messages, various attitude conversions etc.
* **[trajectory_helper](https://github.com/lmark1/uav_ros_lib/tree/main/include/uav_ros_lib/trajectory/trajectory_helper.hpp)** - Useful functions for generating simple trajectory references

### Classes
* **[Topic Handler](https://github.com/lmark1/uav_ros_lib/tree/main/include/uav_ros_lib/topic_handler.hpp)** - A helper class for handling ROS message subscriptions
* **[Reconfigure Handler](https://github.com/lmark1/uav_ros_lib/tree/main/include/uav_ros_lib/reconfigure_handler.hpp)** - A helper class for handling ROS reconfigure server
* **[Global To Local](https://github.com/lmark1/uav_ros_lib/blob/main/include/uav_ros_lib/global_to_local.hpp)** - A class used for transforming [Lat,Lon,Alt] to and from ENU coordinates


## Estimation
* **[Kalman Filter - Vanilla](https://github.com/lmark1/uav_ros_lib/tree/main/include/uav_ros_lib/estimation/kalman_filter_vanilla.hpp)** - A straightforward implementation of a single measurement Discrete Kalman Filter
* **[kf_base](https://github.com/lmark1/uav_ros_lib/tree/main/include/uav_ros_lib/estimation/kf_base.hpp)** - A class providing a template for a generic kalman filter implementation
* **[Constant Velocity LKF](https://github.com/lmark1/uav_ros_lib/tree/main/include/uav_ros_lib/estimation/constant_velocity_lkf.hpp)** - An implementation of a constant velocity linear Kalman filter using the [kf_base](https://github.com/lmark1/uav_ros_lib/tree/main/include/uav_ros_lib/estimation/kf_base.hpp) as the base class
* **[Constant Acceleration LKF](https://github.com/lmark1/uav_ros_lib/tree/main/include/uav_ros_lib/estimation/constant_acceleration_lkf.hpp)** - An implementation of a constant acceleration linear Kalman filter using the [kf_base](https://github.com/lmark1/uav_ros_lib/tree/main/include/uav_ros_lib/estimation/kf_base.hpp) as the base class
