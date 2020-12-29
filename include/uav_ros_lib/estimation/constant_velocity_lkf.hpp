#ifndef CONSTANT_VELOCITY_LKF_HPP
#define CONSTANT_VELOCITY_LKF_HPP

#include <uav_ros_lib/estimation/kf_base.h>
#include <uav_ros_lib/ConstantVelocityLKFParametersConfig.h>
#include <iostream>

class ConstantVelocityLKF
  : public kf_base<uav_ros_lib::ConstantVelocityLKFParametersConfig, 1, 2>
{
public:
  /**
   * @brief Default Constructor. Initializes Kalman filter without the reconfigure server.
   */
  ConstantVelocityLKF();

  /**
   * @brief Construct a new Constant Velocity LKF object with a reconfigure server.
   *
   * @param name Name of the reconfigure server
   * @param nh A ROS node handle
   */
  ConstantVelocityLKF(std::string name, ros::NodeHandle &nh);

  void modelUpdate(double dt) override;
  void measurementUpdate(meas_t measurement_vector) override;
  void parametersCallback(typename kf_base::config_t &configMsg,
    uint32_t /* unnused */) override;
  kf_base::config_t initializeParameters(ros::NodeHandle &nh) override;
  void initializeState(meas_t initialState) override;

  /**
   * @brief Output operator override for printhing class objects
   *
   * @param out
   * @param filt
   * @return std::ostream&
   */
  friend std::ostream &operator<<(std::ostream &out, const ConstantVelocityLKF &filt)
  {
    out << "ConstantVelocityLKF parameters are:"
        << "\nMeasure noise=" << filt.R_
        << "\nPosition noise=" << filt.process_noise_position_stddev_
        << "\nVelocity noise=" << filt.process_noise_velocity_stddev_ << std::endl;
    return out;
  }

private:
  void initializeInternal();

  /**
   * @brief Sets the kalman filter parameter measurement noise
   *
   * @param r Measurement noise
   */
  inline void setMeasureNoise(float r) { R_ = r * r; }

  /**
   * @brief Method sets the kalman filter process noise (position)
   *
   * @param q Process noise (position)
   */
  inline void setPositionNoise(float q) { process_noise_position_stddev_ = q; }

  /**
   * @brief Method sets the kalman filter process noise (velocity)
   *
   * @param q Process noise (velocity)
   */
  inline void setVelocityNoise(float q) { process_noise_velocity_stddev_ = q; }

  // Measurement noise covariance
  float R_;
  float process_noise_position_stddev_;
  float process_noise_velocity_stddev_;

  // Measurement matrix
  Eigen::RowVector2f H_;

  // Dynamic model matrix
  Eigen::Matrix2f A_;

  // Identity matrix
  const Eigen::Matrix2f IDENTITY = Eigen::Matrix2f::Identity(2, 2);
};

#endif// CONSTANT_VELOCITY_LKF_HPP
