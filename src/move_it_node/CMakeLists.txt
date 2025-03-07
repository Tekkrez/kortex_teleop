cmake_minimum_required(VERSION 3.8)
project(move_it_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(teleop_interfaces REQUIRED)


add_executable(move_it_node src/move_it_node.cpp src/robot_util.cpp)
add_executable(move_it_vel_control_node src/move_it_node_vel_control.cpp src/robot_util.cpp)

ament_target_dependencies(move_it_vel_control_node rclcpp moveit_ros_planning sensor_msgs geometry_msgs visualization_msgs trajectory_msgs tf2_ros std_srvs teleop_interfaces)
ament_target_dependencies(move_it_node rclcpp moveit_ros_planning sensor_msgs geometry_msgs visualization_msgs trajectory_msgs tf2_ros)

target_include_directories(move_it_node
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include/move_it_node
  )
target_include_directories(move_it_vel_control_node
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include/move_it_node
  )

install(
  TARGETS
  move_it_vel_control_node
  move_it_node
  DESTINATION
    lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY include
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
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




