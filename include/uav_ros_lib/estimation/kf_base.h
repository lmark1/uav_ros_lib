#ifndef KF_BASE_H
#define KF_BASE_H

#include <Eigen/Dense>
#include <ros/ros.h>
#include <string>
#include <dynamic_reconfigure/server.h>

/**
 * @brief Base class for a Kalman Filter implemnentation.
 *
 * @tparam kalman_config Kalman Filter configuration type
 * @tparam measurement_count Number of model measurements
 * @tparam state_count Number of model states
 */
template<typename kalman_config, unsigned int measurement_count, unsigned int state_count>
class kf_base
{
public:
  using config_t = kalman_config;
  using meas_t = Eigen::Matrix<float, measurement_count, 1>;
  using state_t = Eigen::Matrix<float, state_count, 1>;
  using cov_t = Eigen::Matrix<float, state_count, state_count>;

  static constexpr auto measure_size = measurement_count;
  static constexpr auto state_size = state_count;
  static constexpr auto max_invalid_time = 2;

  /**
   * @brief Construct a new kf base object.
   *
   * @param name Name of the reconfigure server
   */
  explicit kf_base(std::string name)
    : m_name(name), m_kalman_initialized(false),
      m_time_invalid(0), m_config_server{ m_config_mutex,
        ros::NodeHandle("kalman_ns/" + name + "_config") }
  {}

  /**
   * @brief Construct a new kf base object with a default name.
   *
   */
  kf_base() : kf_base("default") {}

  /**
   * @brief Reconfigure server callback.
   *
   */
  virtual void parametersCallback(config_t &configMsg, unsigned int /* unnused */) = 0;

  /**
   * @brief Initialize parameters in a kalman configuration object.
   *
   * @param nh A ROS node handle
   * @return config_t Kalman configuration object
   */
  virtual config_t initializeParameters(ros::NodeHandle &nh) = 0;

  /**
   * @brief This function does the Kalman model update.
   *
   * @param dt Step size
   */
  virtual void modelUpdate(double dt) = 0;

  /**
   * @brief This function does the Kalman measurement update based on the given
   * measurement vector.
   *
   * @param measurement_vector Measurement vector
   */
  virtual void measurementUpdate(meas_t measurement_vector) = 0;

  /**
   * @brief Initialize Kalman filter with the given state initialization vector
   *
   * @param initialState Initial state vector
   */
  virtual void initializeState(meas_t initialState) = 0;

  /**
   * @brief Get the State object
   *
   * @return const state_t&
   */
  const state_t &getState() const { return x_k_; }

  /**
   * @brief Get the State Covariance object
   *
   * @return const cov_t&
   */
  const cov_t &getStateCovariance() const { return P_k_; }

  /**
   * @brief A Generic kalman filter cycle function. It does the model update every time.
   * If measurements are available does also the measurement update.
   *
   * @param dt Step size
   * @param measurements Measurement array
   * @param newMeasurementFlag True if new measurements are available, false otherwise
   */
  void estimateState(double dt,
    std::array<float, measure_size> measurements,
    bool newMeasurementFlag)
  {
    // Reset filtered distance if filter is not initialized
    if (!m_kalman_initialized) { this->resetState(); }

    // Check if initialization failed
    if (!m_kalman_initialized && !newMeasurementFlag) {
      ROS_WARN_THROTTLE(5.0, "KalmanWrapper - Failed to initialize");
      return;
    }

    // Check if initialization should take place
    if (!m_kalman_initialized && newMeasurementFlag) {
      m_kalman_initialized = true;
      this->initializeState(meas_t(measurements.data()));
      ROS_INFO_STREAM("KalmanWrapper " << m_name << " - Initialized.");
    }

    // Do model update
    this->modelUpdate(dt);

    // Do measure update if everything is valid
    if (newMeasurementFlag) {
      // ROS_INFO("KalmanFilter - New measurement! update called");
      this->measurementUpdate(meas_t(measurements.data()));
      m_time_invalid = 0;
    } else {
      // Increase time invalid
      // ROS_WARN("KalmanFilter - doing only model update");
      m_time_invalid += dt;
    }

    // Check if invalid time reached maximum
    if (m_time_invalid > max_invalid_time) {
      this->resetState();
      ROS_FATAL_STREAM("KalmanWrapper " << m_name << " - Max invalid time reached.");
      return;
    }
  }

  /**
   * @brief Setup a ROS reconfigure server for the Kalman filter parameters.
   *
   * @param nh A ROS node handle
   * @param cfg Initial configuration parameters
   */
  void setupReconfigureServer(ros::NodeHandle &nh, const config_t &cfg)
  {
    m_config_server.updateConfig(cfg);
    m_config_param_cb = boost::bind(&kf_base::parametersCallback, this, _1, _2);
    m_config_server.setCallback(m_config_param_cb);
  }

  /**
   * @brief Reset Kalman filter state
   *
   */
  void resetState()
  {
    m_kalman_initialized = false;
    m_time_invalid = 0;
  }

  /**
   * @brief Get the Name object
   *
   * @return const std::string&
   */
  const std::string &getName() const { return m_name; }

private:
  std::string m_name;

  /** Define Dynamic Reconfigure parameters **/
  boost::recursive_mutex m_config_mutex;
  dynamic_reconfigure::Server<config_t> m_config_server;
  typename dynamic_reconfigure::Server<config_t>::CallbackType m_config_param_cb;

  /** Flag signaling that kalman filter is initialized. */
  bool m_kalman_initialized;

  /** Time passed while measurements are invalid. */
  double m_time_invalid;

protected:
  state_t x_k_;
  cov_t P_k_;
};

#endif// KF_BASE_H