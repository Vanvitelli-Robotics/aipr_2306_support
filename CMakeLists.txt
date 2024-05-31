cmake_minimum_required(VERSION 3.8)
project(aipr_2306_support)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)


add_executable(robot_joint_vel_control src/robot_joint_vel_control_node.cpp)
ament_target_dependencies(robot_joint_vel_control rclcpp std_srvs sensor_msgs)

add_executable(target_frame_generator src/target_frame_generator_node.cpp)
ament_target_dependencies(target_frame_generator rclcpp tf2_ros)

install(TARGETS
  robot_joint_vel_control target_frame_generator
  DESTINATION lib/${PROJECT_NAME})

########## INSTALL the launch files ############
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
################################################

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
