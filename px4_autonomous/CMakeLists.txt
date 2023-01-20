cmake_minimum_required(VERSION 3.8)
project(px4_autonomous)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(px4_autonomous_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(offboard src/offboard.cpp)
target_include_directories(offboard PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(offboard PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(offboard rclcpp px4_msgs px4_autonomous_interfaces rclcpp_action tf2_geometry_msgs tf2)

add_executable(transformation src/transformation.cpp)
target_include_directories(transformation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(transformation PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(transformation rclcpp px4_msgs geometry_msgs tf2_ros)

add_executable(visualization src/visualization.cpp)
target_include_directories(visualization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(visualization PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(visualization rclcpp px4_msgs geometry_msgs nav_msgs tf2_ros)

install(TARGETS offboard transformation visualization
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config worlds models
  DESTINATION share/${PROJECT_NAME}/
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
