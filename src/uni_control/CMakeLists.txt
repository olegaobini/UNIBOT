cmake_minimum_required(VERSION 3.8)
project(uni_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# External Library (WiringPi)
find_library(WIRINGPI_LIB wiringPi REQUIRED)

# Executable
add_executable(pwm_node control/pwm.cpp)

# ROS2 dependencies linking
ament_target_dependencies(pwm_node rclcpp)

# WiringPi linking
target_link_libraries(pwm_node ${WIRINGPI_LIB})

# Install executable
install(TARGETS pwm_node
  DESTINATION lib/${PROJECT_NAME})

# Testing setup (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
