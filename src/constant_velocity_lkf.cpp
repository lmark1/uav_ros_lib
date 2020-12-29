#include <uav_ros_lib/estimation/constant_velocity_lkf.hpp>

ConstantVelocityLKF::ConstantVelocityLKF() : kf_base() { initializeInternal(); }

ConstantVelocityLKF::ConstantVelocityLKF(std::string name, ros::NodeHandle &nh)
  : kf_base(name)
{
  initializeInternal();
  auto cfg = initializeParameters(nh);
  setupReconfigureServer(nh, cfg);
}

void ConstantVelocityLKF::modelUpdate(double dt)
{
  Eigen::Matrix2f A;
  A << 1, dt, 0, 1;
  x_k_ = A * x_k_;

  Eigen::Matrix2f Q;
  Q << pow(process_noise_position_stddev_, 2), 0, 0,
    pow(process_noise_velocity_stddev_, 2);
  P_k_ = A * P_k_ * A.transpose() + Q;
}

void ConstantVelocityLKF::measurementUpdate(meas_t measurement_vector)
{
  auto S = H_ * P_k_ * H_.transpose() + R_;
  auto K = P_k_ * H_.transpose() / S;
  auto residual = measurement_vector - H_ * x_k_;
  x_k_ = x_k_ + K * residual;
  P_k_ = (IDENTITY - K * H_) * P_k_;
}

ConstantVelocityLKF::kf_base::config_t ConstantVelocityLKF::initializeParameters(
  ros::NodeHandle &nh)
{
  ROS_WARN("ConstantVelocityLKF::initializeParameters()");
  // Setup dynamic reconfigure server
  float kalmanNoiseMv;
  float kalmanNoisePos;
  float kalmanNoiseVel;
  bool initialized = nh.getParam(getName() + "/kalman/noise_mv", kalmanNoiseMv)
                     && nh.getParam(getName() + "/kalman/noise_pos", kalmanNoisePos)
                     && nh.getParam(getName() + "/kalman/noise_vel", kalmanNoiseVel);

  setMeasureNoise(kalmanNoiseMv);
  setPositionNoise(kalmanNoisePos);
  setVelocityNoise(kalmanNoiseVel);
  ROS_INFO_STREAM(*this);

  if (!initialized) {
    ROS_FATAL(
      "ConstantVelocityLKF::initializeParameters() - parameter initialization failed.");
    throw std::invalid_argument("ConstantVelocityLKF parameters not properly set.");
  }

  kf_base::config_t cfg;
  cfg.noise_mv = kalmanNoiseMv;
  cfg.noise_pos = process_noise_position_stddev_;
  cfg.noise_vel = process_noise_velocity_stddev_;
  return cfg;
}

void ConstantVelocityLKF::initializeState(meas_t initialState)
{
  x_k_[0] = initialState[0];
  x_k_[1] = 0;
}

void ConstantVelocityLKF::initializeInternal()
{
  static constexpr auto initial_pos_stddev = 1;
  static constexpr auto initial_vel_stddev = 10;
  static constexpr auto measurement_position_stddev = 1;
  P_k_ << initial_pos_stddev * initial_pos_stddev, 0, 0,
    initial_vel_stddev * initial_vel_stddev;
  R_ = measurement_position_stddev * measurement_position_stddev;
  x_k_ << 0, 0;
  H_ << 1, 0;
  process_noise_position_stddev_ = initial_pos_stddev;
  process_noise_velocity_stddev_ = initial_vel_stddev;
}

void ConstantVelocityLKF::parametersCallback(typename kf_base::config_t &configMsg,
  uint32_t /* unnused */)
{
  setMeasureNoise(configMsg.noise_mv);
  setPositionNoise(configMsg.noise_pos);
  setVelocityNoise(configMsg.noise_vel);
  ROS_INFO_STREAM(*this);
}
