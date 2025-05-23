cmake_minimum_required(VERSION 3.8)
project(rad_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(trajectory_msgs REQUIRED)

set(Targets
    rad_drive_controller
    rad_status
    rad_tool
    rad_calibration_init
    rover_steer_pos
    rover_arm_steer_pos
  )

foreach(target ${Targets})
  add_executable(${target} src/${target}.cpp src/rad.cpp)
  ament_target_dependencies(${target} rclcpp custom_interfaces std_msgs trajectory_msgs)
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(${target} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

endforeach()

install(TARGETS ${Targets}
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
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
