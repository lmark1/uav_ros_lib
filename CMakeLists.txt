cmake_minimum_required(VERSION 3.10.0)
project(uav_ros_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
find_package(catkin REQUIRED COMPONENTS roscpp dynamic_reconfigure mavros
                                        dynamic_reconfigure)

# Find Eigen
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
endif()

generate_dynamic_reconfigure_options(cfg/ConstantVelocityLKFParameters.cfg
                                     cfg/ConstantAccelerationLKFParameters.cfg
                                     cfg/ConstantAccelerationUKFParameters.cfg)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  nfilt
  dkf_vanilla
  global_to_local
  ros_convert
  trajectory_helper
  constant_velocity_lkf
  constant_acceleration_lkf
  constant_acceleration_ukf
  DEPENDS
  EIGEN3)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS}
                    ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)

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

# Constant velocity LKF library
add_library(constant_velocity_lkf src/constant_velocity_lkf.cpp)
target_link_libraries(constant_velocity_lkf ${catkin_LIBRARIES})
add_dependencies(constant_velocity_lkf ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

# Constant acceleration LKF library
add_library(constant_acceleration_lkf src/constant_acceleration_lkf.cpp)
target_link_libraries(constant_acceleration_lkf ${catkin_LIBRARIES})
add_dependencies(constant_acceleration_lkf ${catkin_EXPORTED_TARGETS}
                 ${PROJECT_NAME}_gencfg)

# Constant acceleration UKF library
add_library(constant_acceleration_ukf src/constant_acceleration_ukf.cpp)
target_link_libraries(constant_acceleration_ukf ${catkin_LIBRARIES})
add_dependencies(constant_acceleration_ukf ${catkin_EXPORTED_TARGETS}
                 ${PROJECT_NAME}_gencfg)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
