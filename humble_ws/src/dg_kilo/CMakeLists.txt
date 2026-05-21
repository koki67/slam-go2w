cmake_minimum_required(VERSION 3.14)
project(dg_kilo)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ---- Dependencies ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(unitree_go REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io kdtree registration)
find_package(GTSAM REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  unitree_go
)

# ---- Custom messages ----
set(msg_files
  "msg/LegOdometry.msg"
  "msg/DegradationStatus.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES geometry_msgs std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# ---- ikd-Tree static library ----
add_library(dg_kilo_ikd STATIC
  include/ikd-Tree/ikd_Tree.cpp
)
target_include_directories(dg_kilo_ikd PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
target_compile_options(dg_kilo_ikd PRIVATE -O3)
target_link_libraries(dg_kilo_ikd PUBLIC
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

# ---- Main executable sources (added incrementally per commit) ----
set(DG_KILO_SRCS
  src/dg_kilo_node.cpp
  src/params.cpp
  src/lowstate_buffer.cpp
  src/go2w_kinematics.cpp
  src/imu_processor.cpp
  src/ikd_tree_wrapper.cpp
  src/feature_extractor.cpp
  src/degradation_detector.cpp
  src/scan_slicer.cpp
  src/leg_eskf.cpp
  src/ground_plane_estimator.cpp
  src/ground_constraint_factor.cpp
  src/loop_closure.cpp
  src/factor_graph_backend.cpp
)

add_executable(dg_kilo_node ${DG_KILO_SRCS})
target_include_directories(dg_kilo_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)
target_link_libraries(dg_kilo_node
  dg_kilo_ikd
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  gtsam
)
ament_target_dependencies(dg_kilo_node ${dependencies})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(dg_kilo_node ${cpp_typesupport_target})

# ---- Tests ----
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_kinematics test/test_kinematics.cpp src/go2w_kinematics.cpp)
  target_include_directories(test_kinematics PRIVATE include ${EIGEN3_INCLUDE_DIR})
  target_link_libraries(test_kinematics Eigen3::Eigen)

  ament_add_gtest(test_degradation
    test/test_degradation.cpp
    src/degradation_detector.cpp
    src/ground_constraint_factor.cpp
  )
  target_include_directories(test_degradation PRIVATE include ${EIGEN3_INCLUDE_DIR} ${GTSAM_INCLUDE_DIR})
  target_link_libraries(test_degradation Eigen3::Eigen gtsam)

  ament_add_gtest(test_leg_eskf test/test_leg_eskf.cpp src/leg_eskf.cpp src/go2w_kinematics.cpp)
  target_include_directories(test_leg_eskf PRIVATE include ${EIGEN3_INCLUDE_DIR})
  target_link_libraries(test_leg_eskf Eigen3::Eigen)
endif()

# ---- Install ----
install(TARGETS dg_kilo_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS dg_kilo_ikd
  EXPORT dg_kilo_ikdTargets
  ARCHIVE DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)

ament_package()
