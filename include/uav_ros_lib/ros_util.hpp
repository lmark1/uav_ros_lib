#ifndef ROS_UTIL_H
#define ROS_UTIL_H

#include <functional>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <Eigen/Eigen>

namespace ros_util {

struct EnumClassHash
{
  template<typename T> std::size_t operator()(T t) const
  {
    return static_cast<std::size_t>(t);
  }
};

struct PairHash
{
  template<class T1, class T2> std::size_t operator()(const std::pair<T1, T2> &pair) const
  {
    return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
  }
};

template<class T> class ParamHandler
{
public:
  ParamHandler(T t_defaultConfig, std::string configName)
    : m_configServer(m_configMutex, ros::NodeHandle(std::move(configName)))
  {
    m_configServer.updateConfig(std::move(t_defaultConfig));
    auto paramCallback = boost::bind(&ParamHandler::paramCallback, this, _1, _2);
    m_configServer.setCallback(paramCallback);
  }

  const T &getData() { return m_currentConfig; }

private:
  void paramCallback(const T &cfgParams, uint32_t /* unused */)
  {
    m_currentConfig = std::move(cfgParams);
  }

  boost::recursive_mutex m_configMutex;
  dynamic_reconfigure::Server<T> m_configServer;
  T m_currentConfig;
};
}// namespace ros_util

#endif /* ROS_UTIL_H */