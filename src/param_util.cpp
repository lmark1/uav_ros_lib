#include <uav_ros_lib/param_util.hpp>

Eigen::MatrixXd param_util::loadMatrixOrThrow(ros::NodeHandle &t_nh,
  const std::string &matrix_name,
  int row_count,
  int col_count)
{

  std::vector<double> tmp_vec;
  bool success = t_nh.getParam(matrix_name, tmp_vec);
  if (!success) {
    ROS_FATAL_STREAM("Unable to get param: [" << matrix_name << "]. Throwing...");
    throw std::runtime_error("Parameter initialization failed.");
  }

  if (static_cast<int>(tmp_vec.size()) != row_count * col_count) {
    ROS_FATAL_STREAM("Param: [" << matrix_name << "] of size " << tmp_vec.size()
                                << ". Expected " << row_count << "x" << col_count << "!");
    throw std::runtime_error("Parameter initialization failed.");
  }

  Eigen::MatrixXd matrix(row_count, col_count);
  // Matrix is the correct size, assign it
  try {
    for (int i = 0; i < row_count; i++) {
      for (int j = 0; j < col_count; j++) {
        matrix(i, j) = tmp_vec[i * col_count + j];
      }
    }
  } catch (std::runtime_error &e) {
    ROS_FATAL_STREAM("Loading param: [" << matrix_name << "] failed.");
    throw e;
  }

  // Assume everything is correct from here
  ROS_INFO_STREAM("Got param [" << matrix_name << "]\n" << matrix);
  return matrix;
}