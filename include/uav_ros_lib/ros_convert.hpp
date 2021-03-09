#ifndef ROS_CONVERT_HPP
#define ROS_CONVERT_HPP

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tuple>

namespace ros_convert {

/**
 * @brief Wrap given value to the range [min, max]
 *
 * @param x An unwrapped value
 * @param min Minimum range value
 * @param max Maximum range value
 * @return A given value wrapped in [min, max] range
 */
double wrapMinMax(double x, double min, double max);

/**
 * @brief Unwrap two consectutive angles, without jumps.
 * 
 * @param previousAngle 
 * @param newAngle 
 * @return Unwrapped angle.
 */
double unwrap(double previousAngle,double newAngle);

/**
 * @brief Calculate angle difference between 2 angles
 * 
 * @param a First Angle
 * @param b Second Angle
 * @return Angle Difference
 */
double angleDiff(double a, double b);

/**
 * @brief Calculate yaw from the given quaternion
 *
 * @param qx
 * @param qy
 * @param qz
 * @param qw
 * @return Calculated yaw from the given quaternion
 */
double calculateYaw(double qx, double qy, double qz, double qw);

/**
 * @brief Caculate yaw from the given quaternion object.
 * 
 * @param q 
 * @return double 
 */
double calculateYaw(const geometry_msgs::Quaternion& q);

/**
 * @brief Construct a new geometry msgs::Quaternion object from the given heading.
 * 
 * @param heading 
 * @return A quaternion message.
 */
geometry_msgs::Quaternion calculate_quaternion(double heading);

/**
 * @brief Get the geometry_msgs::PoseStamped from given translation and rotation.
 *
 * @param translation Given translation
 * @param rotation Given rotation
 * @return geometry_msgs::PoseStamped
 */
geometry_msgs::PoseStamped get_pose_stamped(const tf2::Vector3 &translation,
  const tf2::Quaternion &rotation);

/**
 * @brief Get tf2::Vector from the given geometry_msgs::Pose.
 *
 * @param pose Given Pose
 * @return tf2::Vector3
 */
tf2::Vector3 vector_from_pose(const geometry_msgs::Pose &pose);

/**
 * @brief Get tf2::Quaternion from the given geometry_msgs::Pose.
 *
 * @param pose Given Pose
 * @return tf2::Quaternion
 */
tf2::Quaternion quaternion_from_pose(const geometry_msgs::Pose &pose);

/**
 * @brief Get the Heading Quaternion from the given start to the end coordinates.
 *
 * @param t_xStart X coordinate of the start position
 * @param t_yStart y coordinate of the start position
 * @param t_xEnd X coordinate of the end position
 * @param t_yEnd Y coordinate of the end position
 * @return tf2::Quaternion
 */
tf2::Quaternion
  get_heading_quaternion(double t_xStart, double t_yStart, double t_xEnd, double t_yEnd);

/**
 * @brief Make a trajectory_msgs::MultiDOFJoinTrajectoryPoint from the given translational
 * and rotation information.
 *
 * @param x
 * @param y
 * @param z
 * @param qx
 * @param qy
 * @param qz
 * @param qw
 * @return trajectory_msgs::MultiDOFJointTrajectoryPoint
 */
trajectory_msgs::MultiDOFJointTrajectoryPoint to_trajectory_point(double x,
  double y,
  double z,
  double qx,
  double qy,
  double qz,
  double qw);

/**
 * @brief Make a trajectory_msgs::MultiDOFJoinTrajectoryPoint from the given translational
 * and rotation information.
 *
 * @param x
 * @param y
 * @param z
 * @param yaw
 * @return trajectory_msgs::MultiDOFJointTrajectoryPoint
 */
trajectory_msgs::MultiDOFJointTrajectoryPoint
  to_trajectory_point(double x, double y, double z, double yaw);

/**
 * @brief Extract position from the odometry message
 * 
 * @param odom 
 * @return std::tuple<double, double, double> [x, y, z] odometry position
 */
std::tuple<double, double, double> extract_odometry(const nav_msgs::Odometry &odom);

}// namespace ros_convert

#endif /* ROS_CONVERT_HPP */