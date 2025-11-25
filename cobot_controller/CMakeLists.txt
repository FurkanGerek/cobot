cmake_minimum_required(VERSION 3.8)
project(cobot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(cobot_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

generate_parameter_library(cobot_controller_parameters
  src/cobot_controller_parameter.yaml
)

pluginlib_export_plugin_description_file(controller_interface cobot_control_plugin.xml)

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


add_library(
  ${PROJECT_NAME} SHARED
  src/cobot_controller.cpp
)

target_link_libraries(${PROJECT_NAME}
  cobot_controller_parameters
  rclcpp_action::rclcpp_action
  ${cobot_interfaces_LIBRARIES}
)

target_include_directories(cobot_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/cobot_controller>
  ${cobot_interfaces_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  controller_interface
  geometry_msgs
  rclcpp_action
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS cobot_controller cobot_controller_parameters
  EXPORT export_cobot_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_cobot_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  controller_interface
  geometry_msgs
)

ament_package()
