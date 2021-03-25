#ifndef TOPIC_HANDLER_HPP
#define TOPIC_HANDLER_HPP

#include <ros/ros.h>
#include <functional>
#include <mutex>
#include <memory>

namespace ros_util {

/**
 * @brief A Subscriber Topic Handler class used to setup the ROS subscriber handle and a
 * timer to check if the topic is active. ROS Timer is only activated if a timeout is
 * specified.
 *
 * @tparam T ROS Message Typename
 */
template<class T> class TopicHandler
{
public:
  using Ptr = std::unique_ptr<TopicHandler<T>>;

  /**
   * @brief Construct a new Topic Handler object. Does nothing.
   *
   */
  TopicHandler() {}

  /**
   * @brief Construct a new Topic Handler object.
   *
   * @param t_nh A ROS node handle.
   * @param t_topicName A topic name.
   * @param t_timeoutDuration Optional desired timeout duration to indicate topic
   * inactivity.
   */
  TopicHandler(ros::NodeHandle &t_nh,
    std::string t_topicName,
    const double t_timeoutDuration = 2)
    : TopicHandler(t_nh, t_topicName, nullptr, t_timeoutDuration)
  {}

  /**
   * @brief Construct a new Topic Handler object with an additional post callback
   * function argument.
   *
   * @param t_nh A ROS node handle.
   * @param t_topicName A topic name.
   * @param t_postCallbackFunction A post callback function.
   * @param t_timeoutDuration Optional desired timeout duration to indicate topic
   * inactivity.
   */
  TopicHandler(ros::NodeHandle &t_nh,
    std::string t_topicName,
    std::function<void(const typename T::ConstPtr &)> t_postCallbackFunction,
    const double t_timeoutDuration = 2)
    : m_topicName(std::move(t_topicName)), m_topicTimeout(t_timeoutDuration),
      m_lastMessgeTime(0), m_isResponsive(false),
      m_postCallbackFunction(t_postCallbackFunction), m_messageRecieved(false)
  {
    m_subT = t_nh.subscribe(m_topicName, 1, &TopicHandler::callback, this);

    if (t_timeoutDuration > 0) {
      m_watchdogTimer = t_nh.createTimer(
        ros::Duration(t_timeoutDuration), &TopicHandler::watchdog_callback, this);
    }
  }

  /**
   * @brief Construct a new Topic Handler object with an additional post callback
   * function argument that can be an object method.
   *
   * @tparam Obj Type of the object which holds the callback method
   * @param t_nh A ROS node handle.
   * @param t_topicName A topic name.
   * @param message_callback A post callback function as a class method.
   * @param object Object upon which the post callback function is called.
   * @param t_timeoutDuration Optional desired timeout duration to indicate topic
   * inactivity.
   */
  template<typename Obj>
  TopicHandler(ros::NodeHandle &t_nh,
    std::string t_topicName,
    void (Obj::*message_callback)(const typename T::ConstPtr &),
    Obj *object,
    const double t_timeoutDuration = 2)
    : TopicHandler(t_nh,
        t_topicName,
        std::bind(message_callback, object, std::placeholders::_1),
        t_timeoutDuration)
  {}

  /**
   * @brief Get the latest Data from the topic.
   *
   * @return const T&
   */
  const T &getData() const
  {
    std::scoped_lock<std::recursive_mutex> lock(m_recursive_mutex);
    return *m_data;
  }

  /**
   * @brief Check if the topic is responsive within the specified timeout.
   *
   * @return true if the topic is responsive.
   * @return false if topic is unresponsive.
   */
  bool isResponsive() const
  {
    std::scoped_lock<std::recursive_mutex> lock(m_recursive_mutex);
    return m_isResponsive;
  }

  /**
   * @brief Check if any message is recieved on the given topic.
   *
   * @return true if any message is recieved.
   * @return false if no message is recieved.
   */
  bool isMessageRecieved() const
  {
    std::scoped_lock<std::recursive_mutex> lock(m_recursive_mutex);
    return m_messageRecieved;
  }

private:
  void callback(const typename T::ConstPtr &msg)
  {
    std::scoped_lock<std::recursive_mutex> lock(m_recursive_mutex);
    m_data = msg;
    m_lastMessgeTime = ros::Time::now().toSec();
    m_messageRecieved = true;
    if (m_postCallbackFunction) { m_postCallbackFunction(m_data); }
  }

  void watchdog_callback(const ros::TimerEvent & /* unused */)
  {
    std::scoped_lock<std::recursive_mutex> lock(m_recursive_mutex);
    double elapsedTime = ros::Time::now().toSec() - m_lastMessgeTime;
    if (elapsedTime > m_topicTimeout) {
      m_isResponsive = false;
      ROS_FATAL_STREAM("Topic: [" << m_topicName << "] unresponsive.");
    } else {
      m_isResponsive = true;
    }
  }

  std::recursive_mutex m_recursive_mutex;
  ros::Timer m_watchdogTimer;
  double m_topicTimeout, m_lastMessgeTime;
  bool m_isResponsive, m_messageRecieved;

  ros::Subscriber m_subT;
  typename T::ConstPtr m_data;
  std::string m_topicName;
  std::function<void(const typename T::ConstPtr &)> m_postCallbackFunction;
};


/**
 * @brief A Subscriber Topic Handler class used to setup the ROS subscriber handle and a
 * timer to check if the topic is active. ROS Timer is only activated if a timeout is
 * specified. Additionally it uses a given function to call during the subscriber
 * callback.
 *
 * @tparam T ROS Message Typename
 */
template<class T> class TopicHandlerWithFunction
{
public:
  /**
   * @brief Construct a new Topic Handler With Function object.
   *
   * @param t_nh A ROS node handle.
   * @param t_topicName A topic name.
   * @param t_postCallbackFunction A Function to be called during subscriber callback.
   * @param t_timeoutDuration ptional desired timeout duration to indicate topic
   * inactivity.
   */
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

  /**
   * @brief Get the latest Data from the topic.
   *
   * @return const T&
   */
  const T &getData() const { return m_data; }

  /**
   * @brief Check if the topic is responsive within the specified timeout.
   *
   * @return true if the topic is responsive.
   * @return false if topic is unresponsive.
   */
  bool isResponsive() const { return m_isResponsive; }

  /**
   * @brief Check if any message is recieved on the given topic.
   *
   * @return true if any message is recieved.
   * @return false if no message is recieved.
   */
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

}// namespace ros_util

#endif /* TOPIC_HANDLER_HPP */