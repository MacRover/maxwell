cmake_minimum_required(VERSION 3.8)
project(arm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
)
set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  controller_interface
  realtime_tools
  trajectory_msgs
)

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

foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  arm_controller
  SHARED
  hardware/arm_hardware.cpp
  controller/arm_controller.cpp
)

target_compile_features(arm_controller PUBLIC cxx_std_17)
target_include_directories(arm_controller PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
$<INSTALL_INTERFACE:include/arm_controller>
)

ament_target_dependencies(
  arm_controller PUBLIC
  ${HW_IF_INCLUDE_DEPENDS}
  ${CONTROLLER_INCLUDE_DEPENDS}
)

add_executable(keyboard_controller src/keyboard_controller.cpp)
ament_target_dependencies(keyboard_controller rclcpp trajectory_msgs sensor_msgs)

add_executable(joy_controller src/joy_controller.cpp)
ament_target_dependencies(keyboard_controller rclcpp trajectory_msgs)
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


pluginlib_export_plugin_description_file(hardware_interface arm_controller.xml)
pluginlib_export_plugin_description_file(controller_interface arm_controller.xml)

install(
  DIRECTORY hardware/include/
  DESTINATION include/arm_controller
)

install(TARGETS keyboard_controller
  rad_converter
  arm_rad_dummy
  servo_keyboard_input
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS joy_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS arm_controller
  EXPORT arm_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
