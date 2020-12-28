#ifndef GLOBAL_TO_LOCAL_H
#define GLOBAL_TO_LOCAL_H

#include <ros/ros.h>
#include <uav_ros_lib/topic_handler.hpp>
#include <Eigen/Dense>
#include <mavros_msgs/HomePosition.h>

namespace tf_util {

/**
 * @brief This class is used to transform global (Lat, Lon, Alt) to local (East, North,
 * Up) coordinates and the other way around. It sets up a subsciber to
 * mavros/global_position/home and transforms coordinates with respect to the home
 * position found on that topic.
 *
 */
class GlobalToLocal
{
public:
  /**
   * @brief Construct a new Global To Local object. Sets up a subscriber to the
   * mavros/global_position/home.
   *
   * @param nh A ROS node handle
   */
  explicit GlobalToLocal(ros::NodeHandle &nh);

  /**
   * @brief Transforms local ENU position to Global Lat,Lon,Alt with respect to the
   * obtained home position from mavros/global_position/home.
   *
   * @param t_enuX ENU position x component
   * @param t_enuY ENU position y component
   * @param t_enuZ ENU position z compoenent
   * @return Eigen::Vector3d Global position (Lat, Lon, Alt)
   */
  Eigen::Vector3d toGlobal(double t_enuX, double t_enuY, double t_enuZ);

  /**
   * @brief 
   * 
   * @param lat 
   * @param lon 
   * @param alt 
   * @param altitudeRelative 
   * @return Eigen::Vector3d 
   */
  Eigen::Vector3d
    toLocal(double lat, double lon, double alt, bool altitudeRelative = false);

private:
  ros_util::TopicHandler<mavros_msgs::HomePosition> m_homeHandler;
};
}// namespace tf_util
#endif