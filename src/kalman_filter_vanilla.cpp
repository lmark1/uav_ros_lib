#include <uav_ros_lib/kalman_filter_vanilla.hpp>

KalmanFilterVanilla::KalmanFilterVanilla()
{
  // Initialization of the covariances and
  // measure and model noises
  x_[0] = 0;
  x_[1] = 0;
  x_cov_[0][0] = 1;
  x_cov_[0][1] = 0;
  x_cov_[1][0] = 0;
  x_cov_[1][1] = 1;

  q_[0] = 1;
  q_[1] = 10;
  r_ = 10;
}

void KalmanFilterVanilla::initializePosition(double pos) { x_[0] = pos; }

void KalmanFilterVanilla::setPositionNoise(double q) { q_[0] = q; }

void KalmanFilterVanilla::setVelocityNoise(double q) { q_[1] = q; }

void KalmanFilterVanilla::setMeasureNoise(double r) { r_ = r; }

void KalmanFilterVanilla::modelUpdate(double dt)
{
  x_[0] = x_[0] + dt * x_[1];
  x_cov_[0][0] =
    x_cov_[0][0] + dt * (x_cov_[1][0] + x_cov_[0][1]) + dt * dt * x_cov_[1][1] + q_[0];
  x_cov_[0][1] = x_cov_[0][1] + dt * x_cov_[1][1];
  x_cov_[1][0] = x_cov_[1][0] + dt * x_cov_[1][1];
  x_cov_[1][1] = x_cov_[1][1] + q_[1];
}

void KalmanFilterVanilla::measureUpdate(double pos_m)
{
  double sk, k1, k2, dk;

  sk = x_cov_[0][0] + r_;
  k1 = x_cov_[0][0] / sk;
  k2 = x_cov_[1][0] / sk;
  dk = pos_m - x_[0];
  x_[0] = x_[0] + k1 * dk;
  x_[1] = x_[1] + k2 * dk;
  x_cov_[0][0] = (1 - k1) * x_cov_[0][0];
  x_cov_[0][1] = (1 - k1) * x_cov_[0][1];
  x_cov_[1][0] = -k2 * x_cov_[0][0] + x_cov_[1][0];
  x_cov_[1][1] = -k2 * x_cov_[0][1] + x_cov_[1][1];
}

double KalmanFilterVanilla::getPosition() { return x_[0]; }

double KalmanFilterVanilla::getVelocity() { return x_[1]; }

double KalmanFilterVanilla::getMesaureNoise() { return r_; }

double KalmanFilterVanilla::getPositionNoise() { return q_[0]; }

double KalmanFilterVanilla::getVelocityNoise() { return q_[1]; }

std::ostream &operator<<(std::ostream &out, const KalmanFilterVanilla &filt)
{
  out << "Kalman Filter parameters are:"
      << "\nMeasure noise=" << filt.r_ << "\nPosition noise=" << filt.q_[0]
      << "\nVelocity noise=" << filt.q_[1] << std::endl;
  return out;
}
