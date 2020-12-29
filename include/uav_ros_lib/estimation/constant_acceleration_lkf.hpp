#ifndef CONSTANT_ACCELERATION_LKF_HPP
#define CONSTANT_ACCELERATION_LKF_HPP

#include <uav_ros_lib/estimation/kf_base.h>
#include <uav_ros_lib/ConstantAccelerationLKFParametersConfig.h>

class ConstantAccelerationLKF
  : public kf_base<uav_ros_lib::ConstantAccelerationLKFParametersConfig, 2, 3>
{
public:
  /**
   * @brief Construct a new Constant Acceleration LKF object
   *
   */
  ConstantAccelerationLKF();

  /**
   * @brief Construct a new Constant Acceleration LKF object
   *
   * @param name
   * @param nh
   */
  ConstantAccelerationLKF(std::string name, ros::NodeHandle &nh);

  void modelUpdate(double dt) override;
  void measurementUpdate(meas_t measurement_vector) override;
  void initializeState(meas_t initialState) override;
  void parametersCallback(config_t &configMsg, unsigned int /* unnused */) override;
  config_t initializeParameters(ros::NodeHandle &nh) override;

  /**
   * @brief Output operator override for printhing class objects
   *
   * @param out
   * @param filt
   * @return std::ostream&
   */
  friend std::ostream &operator<<(std::ostream &out, const ConstantAccelerationLKF &filt)
  {
    out << "Kalman Filter parameters are:"
        << "\nMeasure noise=" << filt.R_
        << "\nPosition noise=" << filt.process_noise_position_stddev_
        << "\nVelocity noise=" << filt.process_noise_velocity_stddev_
        << "\nAcceleration noise=" << filt.process_noise_acceleration_stddev_
        << std::endl;
    return out;
  }

private:
  void initializeInternal();

  /**
   * @brief Sets the kalman filter parameter measurement noise
   *
   * @param r Measurement position noise
   */
  inline void setMeasurePositionNoise(float r) { R_(0, 0) = r * r; }

  /**
   * @brief Set the Measure Acceleration Noise object.
   *
   * @param r Measurement acceleration noise
   */
  inline void setMeasureAccelerationNoise(float r) { R_(1, 1) = r * r; }

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

  /**
   * @brief Set the Acceleration Noise object.
   *
   * @param q Process noise (acceleration)
   */
  inline void setAccelerationNoise(float q) { process_noise_acceleration_stddev_ = q; }

  // Measurement noise covariance
  Eigen::Matrix2f R_;

  float process_noise_position_stddev_;
  float process_noise_velocity_stddev_;
  float process_noise_acceleration_stddev_;

  // Measurement matrix
  Eigen::Matrix<float, 2, 3> H_;

  // Dynamic model matrix
  Eigen::Matrix3f A_;

  // Identity matrix
  const Eigen::Matrix3f IDENTITY = Eigen::Matrix3f::Identity(3, 3);
};

#endif// CONSTANT_ACCELERATION_LKF_HPP