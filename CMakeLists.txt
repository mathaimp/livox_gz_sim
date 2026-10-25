cmake_minimum_required(VERSION 3.10)
project(livox_gz_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# the Livox lidar and IMU Gazebo systems; found by Gazebo through GZ_SIM_SYSTEM_PLUGIN_PATH (hooks/)
add_library(${PROJECT_NAME} SHARED src/livox_lidar.cpp src/livox_imu.cpp)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
target_link_libraries(${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${livox_ros_driver2_TARGETS}
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10)
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

install(DIRECTORY launch meshes rviz scan_patterns urdf worlds DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # files carry an SPDX one-liner, not the full license text ament_copyright wants
  # (cpplint still checks for the copyright line)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  # no GPU needed; the Gazebo plugins themselves need one, so they are not tested here
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_xacro test/test_xacro.py)
  ament_add_pytest_test(test_scan_pattern test/test_scan_pattern.py)
endif()

ament_package()
