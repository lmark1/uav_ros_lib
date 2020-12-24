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

catkin_package(INCLUDE_DIRS include LIBRARIES nfilt DEPENDS EIGEN3)

# Nonlinear filters library
add_library(nfilt src/nonlinear_filters.cpp)
target_link_libraries(nfilt ${catkin_LIBRARIES})

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
