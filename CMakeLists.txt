cmake_minimum_required(VERSION 3.10.0)
project(uav_ros_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
find_package(catkin REQUIRED COMPONENTS roscpp dynamic_reconfigure mavros)

# Find Eigen
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
endif()

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  nfilt
  dkf_vanilla
  global_to_local
  ros_convert
  trajectory_helper
  DEPENDS
  EIGEN3)

# Nonlinear filters library
add_library(nfilt src/nonlinear_filters.cpp)
target_link_libraries(nfilt ${catkin_LIBRARIES})

# Convertsion util library
add_library(ros_convert src/ros_convert.cpp)
target_link_libraries(ros_convert ${catkin_LIBRARIES})

# Trajectory helper library
add_library(trajectory_helper src/ros_convert.cpp)
target_link_libraries(trajectory_helper ${catkin_LIBRARIES} ros_convert)

# Global to local transformation library
add_library(global_to_local src/global_to_local.cpp)
target_link_libraries(global_to_local ${catkin_LIBRARIES})

# Kalman filter library
add_library(dkf_vanilla src/kalman_filter_vanilla.cpp)
target_link_libraries(dkf_vanilla ${catkin_LIBRARIES})

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
