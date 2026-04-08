cmake_minimum_required(VERSION 3.8)
project(slam_cpp)

# C++17 required for constexpr LUT, structured bindings, std::optional
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP)

# --- LIBRARY TARGET: slam_core ---
# Pure algorithm code. No ROS2 node logic. Tests link against this.
# This separation lets us test math without spinning up a ROS2 runtime.
add_library(slam_core SHARED
  src/occupancy_grid.cpp
  src/scan_matching.cpp
  src/particle_filter.cpp
)

target_include_directories(slam_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(slam_core PUBLIC Eigen3::Eigen)

# Optimization flags for Release builds.
# -O3: aggressive optimization including loop vectorization and inlining
# -ffast-math: allows FP reordering for better vectorization (acceptable for
#   SLAM where strict IEEE compliance is unnecessary — the particle filter
#   is inherently stochastic, so minor FP ordering differences don't matter)
# Applied only in Release/RelWithDebInfo to keep Debug builds debuggable.
target_compile_options(slam_core PRIVATE
  $<$<CONFIG:Release>:-O3 -ffast-math>
  $<$<CONFIG:RelWithDebInfo>:-O3 -ffast-math>
)

# OpenMP for parallel scan matching in particle_filter.cpp
if(OpenMP_CXX_FOUND)
  target_link_libraries(slam_core PUBLIC OpenMP::OpenMP_CXX)
  target_compile_definitions(slam_core PUBLIC SLAM_HAS_OPENMP)
endif()

# --- EXECUTABLE TARGET: slam_node ---
# ROS2 wiring: subscriptions, publishers, TF, parameter loading.
# Links against slam_core for the actual algorithms.
#
# NOTE: ament_target_dependencies uses "plain" target_link_libraries internally,
# so we must also use plain mode for slam_core (no PRIVATE/PUBLIC keyword).
# Mixing keyword and plain modes on the same target is a CMake error.
add_executable(slam_node src/slam_node.cpp)
target_link_libraries(slam_node slam_core)
ament_target_dependencies(slam_node
  rclcpp nav_msgs sensor_msgs geometry_msgs tf2_ros tf2_geometry_msgs
)

# --- Install ---
install(TARGETS slam_node DESTINATION lib/${PROJECT_NAME})

install(TARGETS slam_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# --- Testing ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # Skip style linters during initial development — will enforce before merge
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

  # Unit tests link against slam_core, NOT slam_node
  ament_add_gtest(test_occupancy_grid test/test_occupancy_grid.cpp)
  target_link_libraries(test_occupancy_grid slam_core)

  ament_add_gtest(test_scan_matching test/test_scan_matching.cpp)
  target_link_libraries(test_scan_matching slam_core)

  ament_add_gtest(test_particle_filter test/test_particle_filter.cpp)
  target_link_libraries(test_particle_filter slam_core)
endif()

ament_package()
