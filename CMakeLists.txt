cmake_minimum_required(VERSION 3.8)
project(tim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(tim REQUIRED)

# generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Action.msg"
  "msg/Edge.msg"
  "msg/Footprint.msg"
  "msg/GeographicalPath.msg"
  "msg/Node.msg"
  "msg/Object.msg"
  "msg/Pose.msg"
  "msg/Prediction.msg"
  "msg/RegionalExtension.msg"
  "msg/TrajectoryForecasting.msg"
  "msg/TravelerDataFrame.msg"
  "msg/TravelerInformationMessage.msg"
  "msg/Velocity.msg"
  DEPENDENCIES std_msgs
)

# add nodes
## infra
add_executable(infra src/infra.cpp)
target_include_directories(infra PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(infra PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  infra
  "rclcpp"
  "tim"
  "visualization_msgs"
)
install(TARGETS infra
  DESTINATION lib/${PROJECT_NAME})

## vehicle
add_executable(vehicle src/vehicle.cpp)
target_include_directories(vehicle PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(vehicle PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  vehicle
  "rclcpp"
  "tim"
  "visualization_msgs"
)
install(TARGETS vehicle
  DESTINATION lib/${PROJECT_NAME})

# test
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
