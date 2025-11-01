cmake_minimum_required(VERSION 3.8)
project(arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)


add_executable(keyboard_controller src/keyboard_controller.cpp)
ament_target_dependencies(keyboard_controller rclcpp trajectory_msgs sensor_msgs)

add_executable(joy_joint_trajectory_controller src/joy_joint_trajectory.cpp)
ament_target_dependencies(joy_joint_trajectory_controller rclcpp trajectory_msgs sensor_msgs)
target_compile_features(joy_joint_trajectory_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(joy_controller src/joy_controller.cpp)
ament_target_dependencies(joy_controller rclcpp std_msgs sensor_msgs control_msgs std_srvs moveit_msgs rclcpp_components)
target_include_directories(keyboard_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(keyboard_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


add_executable(rad_converter src/rad_converter.cpp)
ament_target_dependencies(rad_converter rclcpp sensor_msgs custom_interfaces trajectory_msgs)
target_include_directories(rad_converter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(rad_converter PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(arm_rad_dummy src/arm_rad_dummy.cpp)
ament_target_dependencies(arm_rad_dummy rclcpp sensor_msgs)
target_include_directories(arm_rad_dummy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(arm_rad_dummy PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(servo_keyboard_input src/servo_keyboard_input.cpp)
target_include_directories(servo_keyboard_input PUBLIC include)
ament_target_dependencies(servo_keyboard_input rclcpp sensor_msgs control_msgs)


install(TARGETS keyboard_controller
  rad_converter
  arm_rad_dummy
  servo_keyboard_input
  joy_controller
  joy_joint_trajectory_controller
  DESTINATION lib/${PROJECT_NAME})


# Install launch files.
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
