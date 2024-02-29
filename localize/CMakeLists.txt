cmake_minimum_required(VERSION 3.8)
project(localize)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(localize_node src/localize_node.cpp)

ament_auto_add_executable(tempo_localize_node src/emcl2_posestamped.cpp)

ament_auto_add_executable(timestamp_publisher_node src/mcl_pose_test.cpp)

ament_auto_add_executable(tf_to_posestamped src/tf_to_posestamped.cpp)

# ament_auto_add_executable(lidar_with_trans src/lidar_with_trans.cpp)

ament_auto_add_executable(pose_rate_change src/pose_rate_change.cpp)

install(TARGETS localize_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS timestamp_publisher_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS tf_to_posestamped
  DESTINATION lib/${PROJECT_NAME})

# install(TARGETS lidar_with_trans
#   DESTINATION lib/${PROJECT_NAME})

install(TARGETS pose_rate_change
  DESTINATION lib/${PROJECT_NAME})

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