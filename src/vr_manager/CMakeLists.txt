cmake_minimum_required(VERSION 3.8)
project(vr_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(ros2_aruco_interfaces REQUIRED)
find_package(teleop_interfaces REQUIRED)


add_executable(vr_manager_node src/vr_manager_node.cpp src/vr_manager_utils.cpp)
add_executable(camera_alignment_node src/camera_alignment.cpp src/vr_manager_utils.cpp)

ament_target_dependencies(vr_manager_node rclcpp geometry_msgs tf2_eigen ros2_aruco_interfaces)
ament_target_dependencies(camera_alignment_node rclcpp geometry_msgs tf2_eigen ros2_aruco_interfaces teleop_interfaces)

target_include_directories(vr_manager_node
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
)
target_include_directories(camera_alignment_node
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
)

install(
  TARGETS
  vr_manager_node
  camera_alignment_node
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
