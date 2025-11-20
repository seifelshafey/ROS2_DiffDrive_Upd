cmake_minimum_required(VERSION 3.8)
project(teleop_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)

add_executable(teleop src/teleop_node.cpp)
add_executable(spawn src/spawn_robot.cpp)
add_executable(state src/state_estimation_node.cpp)
ament_target_dependencies(teleop rclcpp geometry_msgs)
ament_target_dependencies(spawn rclcpp geometry_msgs std_srvs tf2 tf2_ros)
ament_target_dependencies(state rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS teleop spawn state DESTINATION lib/${PROJECT_NAME})

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

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
