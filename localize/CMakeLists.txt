cmake_minimum_required(VERSION 3.8)
project(localize)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
# find_package(ament_lint_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
# tf2
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

ament_auto_add_executable(localize_node src/localize_node.cpp)
target_include_directories(localize_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(localize_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  localize_node
  "rclcpp"
  "geometry_msgs"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_eigen"
)

ament_auto_add_executable(tempo_localize_node src/emcl2_posestamped.cpp)
target_include_directories(tempo_localize_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tempo_localize_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  tempo_localize_node
  "rclcpp"
  "geometry_msgs"
)

ament_auto_add_executable(timestamp_publisher_node src/mcl_pose_test.cpp)
target_include_directories(timestamp_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(timestamp_publisher_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  timestamp_publisher_node
  "rclcpp"
  "geometry_msgs"
)

install(TARGETS localize_node
DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  timestamp_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_add_executable(tf_to_posestamped src/tf_to_posestamped.cpp)
target_include_directories(tf_to_posestamped PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tf_to_posestamped PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  tf_to_posestamped
  "rclcpp"
  "geometry_msgs"
  "tf2_ros"
)

install(TARGETS
  tf_to_posestamped
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

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  yaml
)

# ament_package()
