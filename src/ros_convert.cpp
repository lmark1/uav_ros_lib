#include <uav_ros_lib/ros_convert.hpp>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>

double wrapMax(double x, double max)
{
  /* integer math: `(max + x % max) % max` */
  return fmod(max + fmod(x, max), max);
}

double ros_convert::wrapMinMax(double x, double min, double max)
{
  return min + wrapMax(x - min, max - min);
}

double ros_convert::calculateYaw(double qx, double qy, double qz, double qw)
{
  return atan2(2 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz);
}

double ros_convert::calculateYaw(const geometry_msgs::Quaternion &q)
{
  return calculateYaw(q.x, q.y, q.z, q.w);
}

geometry_msgs::PoseStamped ros_convert::get_pose_stamped(const tf2::Vector3 &translation,
  const tf2::Quaternion &rotation)
{
  geometry_msgs::PoseStamped p;
  p.pose.position.x = translation.x();
  p.pose.position.y = translation.y();
  p.pose.position.z = translation.z();
  p.pose.orientation.x = rotation.x();
  p.pose.orientation.y = rotation.y();
  p.pose.orientation.z = rotation.z();
  p.pose.orientation.w = rotation.w();
  return p;
}

tf2::Vector3 ros_convert::vector_from_pose(const geometry_msgs::Pose &pose)
{
  return { pose.position.x, pose.position.y, pose.position.z };
}

tf2::Quaternion ros_convert::quaternion_from_pose(const geometry_msgs::Pose &pose)
{
  return {
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
  };
}

tf2::Quaternion ros_convert::get_heading_quaternion(const double t_xStart,
  const double t_yStart,
  const double t_xEnd,
  const double t_yEnd)
{
  tf2::Quaternion q;
  double yaw = atan2(t_yEnd - t_yStart, t_xEnd - t_xStart);
  q.setRPY(0, 0, yaw);
  return q;
}

trajectory_msgs::MultiDOFJointTrajectoryPoint ros_convert::to_trajectory_point(
  const double x,
  const double y,
  const double z,
  const double qx,
  const double qy,
  const double qz,
  const double qw)
{
  trajectory_msgs::MultiDOFJointTrajectoryPoint point;
  point.transforms = std::vector<geometry_msgs::Transform>(1);
  point.velocities = std::vector<geometry_msgs::Twist>(1);
  point.accelerations = std::vector<geometry_msgs::Twist>(1);

  point.transforms[0].translation.x = x;
  point.transforms[0].translation.y = y;
  point.transforms[0].translation.z = z;

  point.transforms[0].rotation.x = qx;
  point.transforms[0].rotation.y = qy;
  point.transforms[0].rotation.z = qz;
  point.transforms[0].rotation.w = qw;

  return point;
}

trajectory_msgs::MultiDOFJointTrajectoryPoint ros_convert::to_trajectory_point(
  const double x,
  const double y,
  const double z,
  const double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return to_trajectory_point(x, y, z, q.getX(), q.getY(), q.getZ(), q.getZ());
}

std::tuple<double, double, double> ros_convert::extract_odometry(
  const nav_msgs::Odometry &odom)
{
  return {
    odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z
  };
}

// Normalize to [-180,180):
inline double constrainAngle(double x)
{
  x = fmod(x + M_PI, M_PI * 2);
  if (x < 0) { x += M_PI * 2; }
  return x - M_PI;
}
// convert to [-360,360]
inline double angleConv(double angle) { return fmod(constrainAngle(angle), M_PI * 2); }

inline double angleDiff(double a, double b)
{
  double dif = fmod(b - a + M_PI, M_PI * 2);
  if (dif < 0) { dif += M_PI * 2; }
  return dif - M_PI;
}

double ros_convert::unwrap(double newAngle, double previousAngle)
{
  return previousAngle - angleDiff(newAngle, angleConv(previousAngle));
}

geometry_msgs::Quaternion ros_convert::calculate_quaternion(double heading) 
{
  tf2::Quaternion q;
  q.setRPY(0, 0, heading);

  geometry_msgs::Quaternion q_msg;
  q_msg.x = q.getX();
  q_msg.y = q.getY();
  q_msg.z = q.getZ();
  q_msg.w = q.getW();
  return q_msg;
}