cmake_minimum_required(VERSION 3.8)
project(mars_rover_tasks)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_srvs REQUIRED)

# Add the executable
add_executable(text_recog_executable src/text_recog_node.cpp)
add_executable(mars_rover_s_server_executable src/mars_rover_status_s_server.cpp)
add_executable(text_recog_s_server_executable src/text_recog_s_server.cpp)
add_executable(mars_rover_status_s_client_executable src/mars_rover_status_s_client.cpp)
add_executable(text_recog_s_client_executable src/text_recog_s_client.cpp)

ament_target_dependencies(text_recog_executable
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

ament_target_dependencies(mars_rover_s_server_executable
  rclcpp
  std_srvs
)

ament_target_dependencies(text_recog_s_server_executable
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  std_srvs
)

ament_target_dependencies(mars_rover_status_s_client_executable
  rclcpp
  std_srvs
)

ament_target_dependencies(text_recog_s_client_executable
  rclcpp
  std_srvs
)


# Install the executable
install(TARGETS
  text_recog_executable
  mars_rover_s_server_executable
  text_recog_s_server_executable
  mars_rover_status_s_client_executable
  text_recog_s_client_executable
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
