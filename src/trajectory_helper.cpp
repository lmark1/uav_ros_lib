#include <fstream>

#include <uav_ros_lib/trajectory/trajectory_helper.hpp>
#include <uav_ros_lib/ros_convert.hpp>

bool trajectory_helper::is_close_to_reference(
  const trajectory_msgs::MultiDOFJointTrajectoryPoint &ref,
  const nav_msgs::Odometry &odom,
  const double tol)
{
  return sqrt(pow(ref.transforms[0].translation.x - odom.pose.pose.position.x, 2)
              + pow(ref.transforms[0].translation.y - odom.pose.pose.position.y, 2)
              + pow(ref.transforms[0].translation.z - odom.pose.pose.position.z, 2))
         < tol;
}

trajectory_msgs::MultiDOFJointTrajectory trajectory_helper::generate_circle_trajectory(
  const double t_x,
  const double t_y,
  const double t_z,
  const trajectory_msgs::MultiDOFJointTrajectoryPoint &currentPoint,
  const int t_numberOfPoints,
  const double t_circleRadius)
{
  trajectory_msgs::MultiDOFJointTrajectory trajectory;
  trajectory.header.stamp = ros::Time::now();
  trajectory.points.push_back(currentPoint);

  const double DEG_TO_RAD = M_PI / 180.0;
  double angleInc = 360.0 / t_numberOfPoints * DEG_TO_RAD;
  for (int i = 0; i <= t_numberOfPoints; i++) {
    const double newX = t_x + t_circleRadius * cos(i * angleInc);
    const double newY = t_y + t_circleRadius * sin(i * angleInc);

    trajectory.points.push_back(ros_convert::to_trajectory_point(newX,
      newY,
      t_z,
      currentPoint.transforms[0].rotation.x,
      currentPoint.transforms[0].rotation.y,
      currentPoint.transforms[0].rotation.z,
      currentPoint.transforms[0].rotation.w));
  }
  return trajectory;
}

std::vector<geometry_msgs::PoseStamped> trajectory_helper::interpolate_points(
  const std::vector<geometry_msgs::PoseStamped> &input,
  int interpolate_count)
{
  std::vector<geometry_msgs::PoseStamped> result;
  const auto icount_double = static_cast<double>(interpolate_count);
  for (std::size_t i = 0; i < input.size() - 1; i++) {
    const auto current_pose = input.at(i);
    const auto next_pose = input.at(i + 1);

    for (int j = 0; j < interpolate_count; j++) {

      double ratio = j / icount_double;
      tf2::Vector3 v;
      v.setInterpolate3(ros_convert::vector_from_pose(current_pose.pose),
        ros_convert::vector_from_pose(next_pose.pose),
        ratio);
      auto q = tf2::slerp(ros_convert::quaternion_from_pose(current_pose.pose),
        ros_convert::quaternion_from_pose(next_pose.pose),
        ratio);
      result.emplace_back(ros_convert::get_pose_stamped(v, q));
    }
  }
  return result;
}

std::vector<std::string> split(const std::string &in, const char delim)
{
  std::vector<std::string> results;
  std::string working;

  for (const char c : in) {
    if (c == delim || c == '\r' || c == '\n') {
      results.push_back(working);
      working.clear();
    } else {
      working.push_back(c);
    }
  }
  results.push_back(working);
  return results;
}

trajectory_msgs::MultiDOFJointTrajectory trajectory_helper::trajectory_from_csv(
  const std::string &infile,
  const std::string &frame_id = "mavros/world",
  bool verbose = false)
{
  std::map<std::string, int> header_map;
  std::map<int, std::vector<double>> result_map;

  std::ifstream myFile(infile);
  if (!myFile.is_open()) { throw std::runtime_error("Could not open file"); }

  std::string line, colname;
  if (myFile.good()) {
    std::getline(myFile, line);
    int index = 0;

    std::vector<std::string> split_result = split(line, ',');
    for (const auto &result : split_result) {
      if (result.empty()) { continue; }

      header_map.emplace(result, index);
      result_map.emplace(index, std::vector<double>{});
      if (verbose) { std::cout << "Colname: " << result << " Index: " << index << "\n"; }
      index++;
    }
  }

  const int t_x_index = header_map.at("t_x");
  const int t_y_index = header_map.at("t_y");
  const int t_z_index = header_map.at("t_z");
  const int q_x_index = header_map.at("q_x");
  const int q_y_index = header_map.at("q_y");
  const int q_z_index = header_map.at("q_z");
  const int q_w_index = header_map.at("q_w");

  double val;
  trajectory_msgs::MultiDOFJointTrajectory trajectory;
  while (std::getline(myFile, line)) {
    std::stringstream ss(line);

    int colIdx = 0;
    while (ss >> val) {
      if (verbose) { std::cout << val << " "; }
      result_map[colIdx].push_back(val);
      if (ss.peek() == ',') { ss.ignore(); }
      colIdx++;
    }
    trajectory.points.emplace_back(
      ros_convert::to_trajectory_point(result_map.at(t_x_index).back(),
        result_map.at(t_y_index).back(),
        result_map.at(t_z_index).back(),
        result_map.at(q_x_index).back(),
        result_map.at(q_y_index).back(),
        result_map.at(q_z_index).back(),
        result_map.at(q_w_index).back()));
    if (verbose) { std::cout << "\n"; }
  }

  // Close file
  myFile.close();

  trajectory.header.stamp = ros::Time::now();
  trajectory.header.frame_id = frame_id;
  return trajectory;
}
