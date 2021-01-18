#ifndef CONST_ACC_KF_H
#define CONST_ACC_KF_H

#include <uav_ros_lib/estimation/kf_base.h>
#include <uav_ros_lib/ConstantAccelerationUKFParametersConfig.h>
#include <iostream>
#include <vector>

struct UKFConfig
{
  UKFConfig(int tuneMe, int nAugmented);
  int lambda;
  int scale;
  double W;
  double W0;
  std::vector<double> weights;
};

/**
 * @brief A concrete implementation of an unscented Kalman filter with a constant
 * acceleration model.
 *
 */
class ConstantAccelerationUKF
  : public kf_base<uav_ros_lib::ConstantAccelerationUKFParametersConfig, 2, 3>
{
public:
  static constexpr auto augmented_count = kf_base::state_size + 3;
  static constexpr auto sigma_count = augmented_count * 2 + 1;
  using aug_sig_t = Eigen::Matrix<float, augmented_count, sigma_count>;
  using aug_x_t = Eigen::Matrix<float, augmented_count, 1>;
  using aug_P_t = Eigen::Matrix<float, augmented_count, augmented_count>;

  /**
   * @brief A constructor.
   * Initializes all private variables.
   */
  ConstantAccelerationUKF();

  /**
   * @brief Construct a new Constant Acceleration UKF object with ROS configuration setup.
   *
   * @param name
   * @param nh A ROS Node handle
   */
  ConstantAccelerationUKF(std::string name, ros::NodeHandle &nh);

  void modelUpdate(double dt) override;
  void measurementUpdate(meas_t measurement_vector) override;
  void initializeState(meas_t initialState) override;
  void parametersCallback(config_t &configMsg, unsigned int /* unnused */) override;
  config_t initializeParameters(ros::NodeHandle &nh) override;

  friend std::ostream &operator<<(std::ostream &out, const ConstantAccelerationUKF &filt)
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

  void predict_mean_and_covariance(aug_sig_t &pred_sigma_points);
  aug_sig_t compute_sigma_points();
  aug_sig_t sigma_point_prediction(aug_sig_t &sigma_points, double dt);

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
  void setMeasureAccelerationNoise(float r) { R_(1, 1) = r * r; }

  /**
   * @brief Method sets the kalman filter process noise (position)
   *
   * @param q Process noise (position)
   */
  void setPositionNoise(float q) { process_noise_position_stddev_ = q; }

  /**
   * @brief Method sets the kalman filter process noise (velocity)
   *
   * @param q Process noise (velocity)
   */
  void setVelocityNoise(float q) { process_noise_velocity_stddev_ = q; }

  /**
   * @brief Set the Acceleration Noise object.
   *
   * @param q Process noise (acceleration)
   */
  void setAccelerationNoise(float q) { process_noise_acceleration_stddev_ = q; }

  Eigen::Matrix2f R_;
  float process_noise_position_stddev_;
  float process_noise_velocity_stddev_;
  float process_noise_acceleration_stddev_;

  UKFConfig _ukfConfig;

  // Measurement matrix
  Eigen::Matrix<float, 2, 3> H_;

  // Dynamic model matrix
  Eigen::Matrix3f A_;
  aug_sig_t _pred_sigma_points;

  // Identity matrix
  const Eigen::Matrix3f IDENTITY = Eigen::Matrix3f::Identity(3, 3);

public:
  float _nis;
};

#endif// CONST_ACC_KF_H