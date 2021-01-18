#include <uav_ros_lib/estimation/constant_acceleration_ukf.hpp>

UKFConfig::UKFConfig(int tuneMe, int nAugmented)
  : lambda(tuneMe - nAugmented), scale(sqrt(lambda + nAugmented)),
    W(0.5 / (lambda + nAugmented)), W0(lambda / double(lambda + nAugmented))
{
  int nSigma = nAugmented * 2 + 1;
  weights = std::vector<double>(nSigma, W);
  weights[0] = W0;
}

ConstantAccelerationUKF::ConstantAccelerationUKF()
  : _ukfConfig(3, augmented_count), kf_base()
{
  initializeInternal();
}

ConstantAccelerationUKF::ConstantAccelerationUKF(std::string name, ros::NodeHandle &nh)
  : _ukfConfig(5, augmented_count), kf_base(name)
{
  initializeInternal();
  auto cfg = initializeParameters(nh);
  setupReconfigureServer(nh, cfg);
}

void ConstantAccelerationUKF::modelUpdate(double dt)
{
  Eigen::Matrix3f A;
  auto sigma_points = compute_sigma_points();
  _pred_sigma_points = sigma_point_prediction(sigma_points, dt);
  predict_mean_and_covariance(_pred_sigma_points);
}

void ConstantAccelerationUKF::predict_mean_and_covariance(aug_sig_t &pred_sigma_points)
{
  x_k_.fill(0.0);
  for (int i = 0; i < pred_sigma_points.cols(); i++) {
    x_k_ += _ukfConfig.weights[i] * pred_sigma_points.col(i).head(kf_base::state_size);
  }

  P_k_.fill(0.0);
  for (int i = 0; i < pred_sigma_points.cols(); i++) {
    auto x_diff = pred_sigma_points.col(i).head(kf_base::state_size) - x_k_;
    P_k_ += _ukfConfig.weights[i] * x_diff * x_diff.transpose();
  }
}

ConstantAccelerationUKF::aug_sig_t ConstantAccelerationUKF::compute_sigma_points()
{

  aug_sig_t augmented_sigma = aug_sig_t::Zero();
  aug_x_t augmented_x = aug_x_t::Zero();
  aug_P_t augmented_P = aug_P_t::Zero();

  // Initizlize entries of augmented x
  augmented_x.head(kf_base::state_size) = x_k_;

  // Initialize entries of augmented P
  augmented_P.topLeftCorner(kf_base::state_size, kf_base::state_size) = P_k_;
  augmented_P(kf_base::state_size, kf_base::state_size) =
    process_noise_position_stddev_ * process_noise_position_stddev_;
  augmented_P(kf_base::state_size + 1, kf_base::state_size + 1) =
    process_noise_velocity_stddev_ * process_noise_velocity_stddev_;
  augmented_P(kf_base::state_size + 2, kf_base::state_size + 2) =
    process_noise_acceleration_stddev_ * process_noise_acceleration_stddev_;

  aug_P_t L = augmented_P.llt().matrixL();
  augmented_sigma.col(0) = augmented_x;

  for (int col_idx = 0; col_idx < augmented_count; col_idx++) {
    augmented_sigma.col(col_idx + 1) = augmented_x + _ukfConfig.scale * L.col(col_idx);
    augmented_sigma.col(col_idx + 1 + augmented_count) =
      augmented_x - _ukfConfig.scale * L.col(col_idx);
  }

  // ROS_INFO_STREAM("Scale: " << _ukfConfig.scale << getName() << " Sigma points: \n"
  // << augmented_sigma << "\n");
  return augmented_sigma;
}

ConstantAccelerationUKF::aug_sig_t
  ConstantAccelerationUKF::sigma_point_prediction(aug_sig_t &sigma_points, double dt)
{
  aug_sig_t pred_sigma_point = aug_sig_t::Zero();
  Eigen::Matrix3f A;
  A << 1, dt, dt * dt / 2., 0, 1., dt, 0, 0, 1.;

  for (int i = 0; i < sigma_points.cols(); i++) {
    pred_sigma_point.col(i).head(kf_base::state_size) =
      A * sigma_points.col(i).head(kf_base::state_size) + sigma_points.col(i).tail(3);
  }
  // ROS_INFO_STREAM("Scale: " << _ukfConfig.scale << getName() << " Predicted Sigma
  // points: \n"  << pred_sigma_point << "\n");
  return pred_sigma_point;
}

void ConstantAccelerationUKF::measurementUpdate(meas_t measurement_vector)
{

  using z_sig_t = Eigen::Matrix<float, kf_base::measure_size, sigma_count>;
  using s_sig_t = Eigen::Matrix<float, kf_base::measure_size, kf_base::measure_size>;
  using tc_t = Eigen::Matrix<float, kf_base::state_size, kf_base::measure_size>;

  z_sig_t z_sigma_points = z_sig_t::Zero();
  for (int i = 0; i < sigma_count; i++) {
    z_sigma_points(0, i) = _pred_sigma_points(0, i);
    z_sigma_points(1, i) = _pred_sigma_points(2, i);
  }
  // ROS_INFO_STREAM("Scale: " << _ukfConfig.scale << getName() << " Z Sigma points: \n"
  // << z_sigma_points << "\n");

  kf_base::meas_t z_pred;
  z_pred.fill(0.0);
  for (int i = 0; i < z_sigma_points.cols(); i++) {
    z_pred += _ukfConfig.weights[i] * z_sigma_points.col(i);
  }

  s_sig_t S = s_sig_t::Zero();
  S.fill(0.0);
  for (int i = 0; i < sigma_count; i++) {
    auto z_diff = z_sigma_points.col(i) - z_pred;
    S += _ukfConfig.weights[i] * z_diff * z_diff.transpose();
  }
  S = S + R_;

  tc_t Tc = tc_t::Zero();
  Tc.fill(0.0);
  for (int i = 0; i < sigma_count; i++) {
    auto z_diff = z_sigma_points.col(i) - z_pred;
    auto x_diff = _pred_sigma_points.col(i).head(kf_base::state_size) - x_k_;
    Tc += _ukfConfig.weights[i] * x_diff * z_diff.transpose();
  }

  auto Sinv = S.inverse();
  auto K = Tc * S.inverse();// 3x2
  auto residual = measurement_vector - z_pred;// 2x1
  x_k_ = x_k_ + K * residual;// 3x1
  P_k_ = P_k_ - K * S * K.transpose();// 3x3
  _nis = residual.transpose() * Sinv * residual;
}

void ConstantAccelerationUKF::initializeState(meas_t initialState)
{
  x_k_[0] = initialState[0];
  x_k_[1] = 0;
  x_k_[2] = initialState[1];
}

void ConstantAccelerationUKF::parametersCallback(config_t &configMsg,
  unsigned int /* unnused */)
{
  ROS_INFO("ConstantAccelerationUKF::parametersCallback");
  setMeasurePositionNoise(configMsg.noise_pos_mv);
  setMeasureAccelerationNoise(configMsg.noise_acc_mv);
  setPositionNoise(configMsg.noise_pos_proc);
  setVelocityNoise(configMsg.noise_vel_proc);
  setAccelerationNoise(configMsg.noise_acc_proc);
  ROS_INFO_STREAM(*this);
  _ukfConfig = UKFConfig(configMsg.tune_me, augmented_count);
}

ConstantAccelerationUKF::config_t ConstantAccelerationUKF::initializeParameters(
  ros::NodeHandle &nh)
{
  ROS_WARN("ConstantAccelerationUKF::initializeParameters()");
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
    ROS_FATAL(
      "KalmanWrapper::initializeParameters() - parameter "
      "initialization failed.");
    throw std::invalid_argument("KalmanWrapper parameters not properly set.");
  }

  kf_base::config_t cfg;
  cfg.noise_pos_mv = kalmanNoisePosMv;
  cfg.noise_acc_mv = kalmanNoiseAccMv;
  cfg.noise_pos_proc = kalmanNoisePosProc;
  cfg.noise_vel_proc = kalmanNoiseVelProc;
  cfg.noise_acc_proc = kalmanNoiseAccProc;
  cfg.tune_me = 3;
  return cfg;
}

void ConstantAccelerationUKF::initializeInternal()
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