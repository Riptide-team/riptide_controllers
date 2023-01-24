cmake_minimum_required(VERSION 3.8)
project(riptide_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
    controller_interface
    generate_parameter_library
    pluginlib
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    tf2
    tf2_ros
    std_srvs
    sensor_msgs
    visualization_msgs
)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(state_estimator_parameters src/state_estimator.yaml)
generate_parameter_library(riptide_controller_parameters src/riptide_controller.yaml)
generate_parameter_library(riptide_echosounder_parameters src/riptide_echosounder.yaml)
generate_parameter_library(riptide_actuators_tester_parameters src/riptide_actuators_tester.yaml)
generate_parameter_library(riptide_battery_card_tester_parameters src/riptide_battery_card_tester.yaml)

add_library(${PROJECT_NAME} SHARED src/state_estimator.cpp src/riptide_controller.cpp src/riptide_echosounder.cpp src/riptide_actuators_tester.cpp src/riptide_battery_card_tester.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include)
target_link_libraries(${PROJECT_NAME} state_estimator_parameters riptide_controller_parameters riptide_echosounder_parameters riptide_actuators_tester_parameters riptide_battery_card_tester_parameters Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(controller_interface riptide_controllers.xml)

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS
  ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
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

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
