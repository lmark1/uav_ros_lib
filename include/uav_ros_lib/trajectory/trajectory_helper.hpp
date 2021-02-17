#ifndef TRAJECTORY_HELPER_HPP
#define TRAJECTORY_HELPER_HPP

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <vector>

namespace trajectory_helper {

/**
 * @brief Is the current referent position close to the current odometry.
 *
 * @param ref Given referent point message
 * @param odom Given odometry message
 * @param tol Given tolerance
 * @return true Reference is close to the odometry
 * @return false Reference is not close to the odometry
 */
bool is_close_to_reference(const trajectory_msgs::MultiDOFJointTrajectoryPoint &ref,
  const nav_msgs::Odometry &odom,
  double tol = 1e-3);

/**
 * @brief Generate a circle trajectory from the current reference to the given [x,y,z]
 * coordinates.
 *
 * @param t_x Circle center - x position
 * @param t_y Circle center - y position
 * @param t_z Circle center - z position
 * @param currentPoint Current trajectory point
 * @param t_numberOfPoints Number of points in the circle
 * @param t_circleRadius Circle radius
 * @return trajectory_msgs::MultiDOFJointTrajectory
 */
trajectory_msgs::MultiDOFJointTrajectory generate_circle_trajectory(double t_x,
  double t_y,
  double t_z,
  const trajectory_msgs::MultiDOFJointTrajectoryPoint &currentPoint,
  int t_numberOfPoints = 10,
  double t_circleRadius = 1);

/**
 * @brief Interpolate points in the given vector based on the given number of points.
 *
 * @param input Input PoseStamped vector
 * @param interpolate_count Number of points added in between
 * @return std::vector<geometry_msgs::PoseStamped>
 */
std::vector<geometry_msgs::PoseStamped> interpolate_points(
  const std::vector<geometry_msgs::PoseStamped> &input,
  int interpolate_count = 50);

/**
 * @brief Interpolate trajectory points based on the given step size
 * 
 * @param input 
 * @param step_size_xy Minimum distance between two points in the xy plane
 * @param step_size_z Minimum distance between two points along the z axis
 * @return trajectory_msgs::MultiDOFJointTrajectory 
 */
trajectory_msgs::MultiDOFJointTrajectory interpolate_points(
  const trajectory_msgs::MultiDOFJointTrajectory &input,
  double step_size_xy, double step_size_z);
  
/**
 * @brief Load trajectory points from given csv file.
 *
 * @param infile CSV input file
 * @param frame_id Frame ID
 * @param verbose
 * @return trajectory_msgs::MultiDOFJointTrajectory
 */
trajectory_msgs::MultiDOFJointTrajectory trajectory_from_csv(const std::string &infile,
  const std::string &frame_id,
  bool verbose);

}// namespace trajectory_helper

#endif /* TRAJECTORY_HELPER_HPP */