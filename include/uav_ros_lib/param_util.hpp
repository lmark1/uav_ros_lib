#ifndef PARAM_UTIL_HPP
#define PARAM_UTIL_HPP

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <iostream>
#include <vector>

namespace param_util {

/**
 * @brief Attmeps to load a matrix from the ROS parameter server or throws a
 * runtime_error.
 *
 * @tparam nRow Number of rows
 * @tparam nCol Number of collumns
 * @param t_nh A ROS Node Handle
 * @param matrix_name Name of the matrix parameter
 * @return Eigen::Matrix<double, nRow, nCol>
 */
template<int nRow, int nCol>
Eigen::Matrix<double, nRow, nCol> loadMatrixOrThrow(ros::NodeHandle &t_nh,
  const std::string &matrix_name)
{
  Eigen::Matrix<double, nRow, nCol> matrix;

  std::vector<double> tmp_vec;
  bool success = t_nh.getParam(matrix_name, tmp_vec);
  if (!success) {
    ROS_FATAL_STREAM("Unable to get param: [" << matrix_name << "]. Throwing...");
    throw std::runtime_error("Parameter initialization failed.");
  }

  if (static_cast<int>(tmp_vec.size()) != nRow * nCol) {
    ROS_FATAL_STREAM("Param: [" << matrix_name << "] of size " << tmp_vec.size()
                                << ". Expected " << nRow << "x" << nCol << "!");
    throw std::runtime_error("Parameter initialization failed.");
  }

  // Matrix is the correct size, assign it
  try {
    for (int i = 0; i < nRow; i++) {
      for (int j = 0; j < nCol; j++) { matrix(i, j) = tmp_vec[i * nCol + j]; }
    }
  } catch (std::runtime_error &e) {
    ROS_FATAL_STREAM("Loading param: [" << matrix_name << "] failed.");
    throw e;
  }

  // Assume everything is correct from here
  ROS_INFO_STREAM("Got param [" << matrix_name << "]\n" << matrix);
  return matrix;
}

/**
 * @brief Attempt to load a dynamic matrix from the ROS parameter server or throw a
 * runtimer error.
 *
 * @param t_nh A ROS node handle.
 * @param matrix_name Name of the matrix parameter.
 * @param row_count Number of matrix rows.
 * @param col_count Number of matrix columns.
 * @return Eigen::MatrixXd
 */
Eigen::MatrixXd loadMatrixOrThrow(ros::NodeHandle &t_nh,
  const std::string &matrix_name,
  int row_count,
  int col_count);

/**
 * @brief This output operator overload is needed to corrcly print out std::vector<T>
 * parameter types in the getParamOrThrow(...) function.
 * 
 * @tparam T 
 * @param o 
 * @param vec 
 * @return std::ostream& 
 */
template<class T>
std::ostream& operator<<(std::ostream& o, const std::vector<T>& vec)
{
  o << "[";
  for (const auto& el : vec) {
    o << el << ", ";
  }
  o << "]";
  return o;
}

/**
 * @brief Attempt to get a value from the ROS parameter server or throws a runtime_error.
 *
 * @tparam T Value Typename
 * @param t_nh A ROS node handle
 * @param t_paramName ROS Parameter name.
 * @param t_paramContainer Variable where the parameter value will be saved.
 */
template<class T>
void getParamOrThrow(ros::NodeHandle &t_nh,
  const std::string &t_paramName,
  T &t_paramContainer)
{
  bool gotParam = t_nh.getParam(t_paramName, t_paramContainer);
  ROS_INFO_STREAM("Got param [" << t_paramName << "] = " << t_paramContainer);
  if (!gotParam) {
    ROS_FATAL_STREAM("Unable to get param: [" << t_paramName << "]. Throwing...");
    throw std::runtime_error("Parameter initialization failed.");
  }
}

/**
 * @brief Attemps to return a value from the ROS parameter server or throws a
 * runtime_error.
 *
 * @tparam T Value Typename
 * @param nh A ROS node handle
 * @param param_name ROS parameter name
 * @return T
 */
template<class T> T getParamOrThrow(ros::NodeHandle &nh, std::string param_name)
{
  T param;
  getParamOrThrow(nh, param_name, param);
  return param;
}

}// namespace param_util
#endif /* PARAM_UTIL_HPP */