#include <uav_ros_lib/estimation/constant_acceleration_lkf.hpp>

ConstantAccelerationLKF::ConstantAccelerationLKF() : kf_base() { initializeInternal(); }

ConstantAccelerationLKF::ConstantAccelerationLKF(std::string name, ros::NodeHandle &nh)
  : kf_base(name)
{
  initializeInternal();
  auto cfg = initializeParameters(nh);
  setupReconfigureServer(nh, cfg);
}

void ConstantAccelerationLKF::modelUpdate(double dt)
{
  Eigen::Matrix3f A;
  A << 1, dt, dt * dt / 2., 0, 1., dt, 0, 0, 1.;
  x_k_ = A * x_k_;

  Eigen::Matrix3f Q;
  Q << pow(process_noise_position_stddev_, 2), 0, 0, 0,
    pow(process_noise_velocity_stddev_, 2), 0, 0, 0,
    pow(process_noise_acceleration_stddev_, 2);
  P_k_ = A * P_k_ * A.transpose() + Q;
}

void ConstantAccelerationLKF::measurementUpdate(meas_t measurement_vector)
{
  auto S = H_ * P_k_ * H_.transpose() + R_;// 2x2
  auto K = (P_k_ * H_.transpose()) * S.inverse();// 3x2
  auto residual = measurement_vector - H_ * x_k_;// 2x1
  x_k_ = x_k_ + K * residual;// 3x1
  P_k_ = (IDENTITY - K * H_) * P_k_;// 3x3
}

void ConstantAccelerationLKF::initializeState(meas_t initialState)
{
  x_k_[0] = initialState[0];
  x_k_[1] = 0;
  x_k_[2] = initialState[1];
}

void ConstantAccelerationLKF::parametersCallback(config_t &configMsg,
  unsigned int /* unnused */)
{
  ROS_INFO("ConstantAccelerationLKF::parametersCallback");
  ROS_INFO_STREAM(*this);
  setMeasurePositionNoise(configMsg.noise_pos_mv);
  setMeasureAccelerationNoise(configMsg.noise_acc_mv);
  setPositionNoise(configMsg.noise_pos_proc);
  setVelocityNoise(configMsg.noise_vel_proc);
  setAccelerationNoise(configMsg.noise_acc_proc);
  ROS_INFO_STREAM(*this);
}

config_t ConstantAccelerationLKF::initializeParameters(ros::NodeHandle &nh)
{
  ROS_WARN("ConstantAccelerationLKF::initializeParameters()");
  // Setup dynamic reconfigure server

  float kalmanNoisePosMv;
  float kalmanNoiseAccMv;
  float kalmanNoisePosProc;
  float kalmanNoiseVelProc;
  float kalmanNoiseAccProc;

  bool initialized =
    nh.getParam(getName() + "/kalman/noise_pos_mv", kalmanNoisePosMv)
    && nh.getParam(getName() + "/kalman/noise_acc_mv", kalmanNoiseAccMv)
    && nh.getParam(getName() + "/kalman/noise_pos_proc", kalmanNoisePosProc)
    && nh.getParam(getName() + "/kalman/noise_vel_proc", kalmanNoiseVelProc)
    && nh.getParam(getName() + "/kalman/noise_acc_proc", kalmanNoiseAccProc);

  setMeasurePositionNoise(kalmanNoisePosMv);
  setMeasureAccelerationNoise(kalmanNoiseAccProc);
  setPositionNoise(kalmanNoisePosProc);
  setVelocityNoise(kalmanNoiseVelProc);
  setAccelerationNoise(kalmanNoiseAccProc);
  ROS_INFO_STREAM(*this);

  if (!initialized) {
    ROS_FATAL("KalmanWrapper::initializeParameters() - parameter initialization failed.");
    throw std::invalid_argument("KalmanWrapper parameters not properly set.");
  }

  kf_base::config_t cfg;
  cfg.noise_pos_mv = kalmanNoisePosMv;
  cfg.noise_acc_mv = kalmanNoiseAccMv;
  cfg.noise_pos_proc = kalmanNoisePosProc;
  cfg.noise_vel_proc = kalmanNoiseVelProc;
  cfg.noise_acc_proc = kalmanNoiseAccProc;
  return cfg;
}

void ConstantAccelerationLKF::initializeInternal()
{
  static constexpr auto initial_pos_stddev = 1;
  static constexpr auto initial_vel_stddev = 10;
  static constexpr auto initial_acc_stddev = 10;
  static constexpr auto measurement_position_stddev = 1;
  static constexpr auto measurement_acc_stddev = 10;

  P_k_ << initial_pos_stddev * initial_pos_stddev, 0, 0, 0,
    initial_vel_stddev * initial_vel_stddev, 0, 0, 0,
    initial_acc_stddev * initial_acc_stddev;
  R_ << measurement_position_stddev * measurement_position_stddev, 0, 0,
    measurement_acc_stddev * measurement_acc_stddev;
  x_k_ << 0, 0;
  H_ << 1, 0, 0, 0, 0, 1;

  process_noise_position_stddev_ = initial_pos_stddev;
  process_noise_velocity_stddev_ = initial_vel_stddev;
  process_noise_acceleration_stddev_ = initial_acc_stddev;
}