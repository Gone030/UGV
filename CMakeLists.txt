cmake_minimum_required(VERSION 3.8)
project(ugv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(ugv_control_node
  src/control_node.cpp
  src/track_kinematics.cpp
  src/serial_bridge.cpp
)
target_compile_features(ugv_control_node PRIVATE cxx_std_17)
target_include_directories(ugv_control_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ugv_control_node rclcpp std_msgs geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(serial_protocol_test tests/serial_protocol_test.cpp src/serial_bridge.cpp)
  if(TARGET serial_protocol_test)
    target_compile_features(serial_protocol_test PRIVATE cxx_std_17)
    target_include_directories(serial_protocol_test PRIVATE include)
  endif()
endif()

install(TARGETS ugv_control_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
