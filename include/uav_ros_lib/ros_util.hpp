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

template<int nRow, int nCol>
Eigen::Matrix<double, nRow, nCol> loadMatrixOrThrow(ros::NodeHandle &t_nh,
  const std::string &matrix_name)
{
  Eigen::Matrix<double, nRow, nCol> matrix;

  std::vector<double> tmp_vec;
  bool success = t_nh.getParam(matrix_name, tmp_vec);
  if (!success) {
    ROS_FATAL_STREAM("Unable to get param: [" << matrix_name << "]. Throwing...");
    throw std::runtime_error("Parameter initialization failed.");
  }

  if (static_cast<int>(tmp_vec.size()) != nRow * nCol) {
    ROS_FATAL_STREAM("Param: [" << matrix_name << "] of size " << tmp_vec.size()
                                    << ". Expected " << nRow << "x" << nCol << "!");
    throw std::runtime_error("Parameter initialization failed.");
  }

  // Matrix is the correct size, assign it
  try {
    for (int i = 0; i < nRow; i++) {
      for (int j = 0; j < nCol; j++) { matrix(i, j) = tmp_vec[i * nRow + j]; }
    }
  } catch (std::runtime_error &e) {
    ROS_FATAL_STREAM("Loading param: [" << matrix_name << "] failed.");
    throw e;
  }

  // Assume everything is correct from here
  ROS_INFO_STREAM("Got param [" << matrix_name << "]\n" << matrix);
  return matrix;
}

template<class T>
void getParamOrThrow(ros::NodeHandle &t_nh,
  const std::string &t_paramName,
  T &t_paramContainer)
{
  bool gotParam = t_nh.getParam(t_paramName, t_paramContainer);
  ROS_INFO_STREAM("Got param [" << t_paramName << "] = " << t_paramContainer);
  if (!gotParam) {
    ROS_FATAL_STREAM("Unable to get param: [" << t_paramName << "]. Throwing...");
    throw std::runtime_error("Parameter initialization failed.");
  }
}

template<class T> T getParamOrThrow(ros::NodeHandle &nh, std::string param_name)
{
  T param;
  getParamOrThrow(nh, param_name, param);
  return param;
}

template<class T> class TopicHandler
{
public:
  TopicHandler(ros::NodeHandle &t_nh,
    std::string t_topicName,
    const double t_timeoutDuration = 2)
    : m_topicName(std::move(t_topicName)), m_topicTimeout(t_timeoutDuration),
      m_lastMessgeTime(0), m_isResponsive(false), m_messageRecieved(false)
  {
    m_subT = t_nh.subscribe(m_topicName, 1, &TopicHandler::callback, this);

    if (t_timeoutDuration > 0) {
      m_watchdogTimer = t_nh.createTimer(
        ros::Duration(t_timeoutDuration), &TopicHandler::watchdog_callback, this);
    }
  }

  const T &getData() const { return m_data; }
  bool isResponsive() const { return m_isResponsive; }
  bool isMessageRecieved() const { return m_messageRecieved; }

private:
  void callback(const T &msg)
  {
    m_data = std::move(msg);
    m_lastMessgeTime = ros::Time::now().toSec();
    m_messageRecieved = true;
  }

  void watchdog_callback(const ros::TimerEvent & /* unused */)
  {
    double elapsedTime = ros::Time::now().toSec() - m_lastMessgeTime;
    if (elapsedTime > m_topicTimeout) {
      m_isResponsive = false;
      ROS_FATAL_STREAM("Topic: [" << m_topicName << "] unresponsive.");
    } else {
      m_isResponsive = true;
    }
  }

  ros::Timer m_watchdogTimer;
  double m_topicTimeout, m_lastMessgeTime;
  bool m_isResponsive, m_messageRecieved;

  ros::Subscriber m_subT;
  T m_data;
  const std::string m_topicName;
};

template<class T> class TopicHandlerWithFunction
{
public:
  TopicHandlerWithFunction(ros::NodeHandle &t_nh,
    std::string t_topicName,
    std::function<void(const T &)> t_postCallbackFunction,
    const double t_timeoutDuration = 2)
    : m_topicName(std::move(t_topicName)), m_topicTimeout(t_timeoutDuration),
      m_lastMessgeTime(0), m_isResponsive(false),
      m_postCallbackFunction(t_postCallbackFunction), m_messageRecieved(false)
  {
    m_subT = t_nh.subscribe(m_topicName, 1, &TopicHandlerWithFunction::callback, this);

    if (t_timeoutDuration > 0) {
      m_watchdogTimer = t_nh.createTimer(ros::Duration(t_timeoutDuration),
        &TopicHandlerWithFunction::watchdog_callback,
        this);
    }
  }

  const T &getData() const { return m_data; }
  bool isResponsive() const { return m_isResponsive; }
  bool isMessageRecieved() const { return m_messageRecieved; }

private:
  void callback(const T &msg)
  {
    m_data = std::move(msg);
    m_lastMessgeTime = ros::Time::now().toSec();
    m_messageRecieved = true;
    m_postCallbackFunction(m_data);
  }

  void watchdog_callback(const ros::TimerEvent &e)
  {
    double elapsedTime = ros::Time::now().toSec() - m_lastMessgeTime;
    if (elapsedTime > m_topicTimeout) {
      m_isResponsive = false;
      ROS_FATAL_STREAM("Topic: [" << m_topicName << "] unresponsive.");
    } else {
      m_isResponsive = true;
    }
  }

  ros::Timer m_watchdogTimer;
  double m_topicTimeout, m_lastMessgeTime;
  bool m_isResponsive, m_messageRecieved;

  std::function<void(const T &)> m_postCallbackFunction;
  ros::Subscriber m_subT;
  T m_data;
  const std::string m_topicName;
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