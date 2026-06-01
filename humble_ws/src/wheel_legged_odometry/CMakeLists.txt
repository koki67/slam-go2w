cmake_minimum_required(VERSION 3.14)
project(wheel_legged_odometry)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(unitree_go REQUIRED)
find_package(Eigen3 REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  unitree_go
  Eigen3
)

add_library(${PROJECT_NAME}_core
  src/go2w_kinematics.cpp
  src/odometry_estimator.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME}_core ${dependencies})

add_executable(wheel_legged_odometry_node src/wheel_legged_odometry_node.cpp)
target_link_libraries(wheel_legged_odometry_node ${PROJECT_NAME}_core)
ament_target_dependencies(wheel_legged_odometry_node ${dependencies})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_go2w_kinematics test/test_go2w_kinematics.cpp)
  target_link_libraries(test_go2w_kinematics ${PROJECT_NAME}_core)

  ament_add_gtest(test_odometry_estimator test/test_odometry_estimator.cpp)
  target_link_libraries(test_odometry_estimator ${PROJECT_NAME}_core)
endif()

install(TARGETS ${PROJECT_NAME}_core wheel_legged_odometry_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)

ament_package()
