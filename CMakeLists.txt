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
    rclcpp_action
    realtime_tools
    tf2
    tf2_ros
    std_srvs
    sensor_msgs
    visualization_msgs
    riptide_msgs
)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## Broadcaster library
# broadcaster source files
list(APPEND broadcaster_controllers
    src/broadcasters/riptide_pressure_broadcaster.cpp
    src/broadcasters/riptide_imu_broadcaster.cpp
    src/broadcasters/riptide_tail_broadcaster.cpp
)

# Compile broadcaster library
add_library(riptide_broadcasters SHARED ${broadcaster_controllers})
target_include_directories(riptide_broadcasters PRIVATE include)
foreach(filename ${broadcaster_controllers})
  get_filename_component(target_name ${filename} NAME_WE)
  generate_parameter_library(${target_name}_parameters src/broadcasters/${target_name}.yaml)
  target_link_libraries(riptide_broadcasters ${target_name}_parameters)
endforeach()
ament_target_dependencies(riptide_broadcasters ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install boradcaster library
pluginlib_export_plugin_description_file(controller_interface riptide_broadcasters.xml)
install(
  TARGETS
  riptide_broadcasters
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

## Riptide controller library
generate_parameter_library(riptide_controller_parameters src/riptide_controller.yaml)
generate_parameter_library(depth_controller_parameters src/depth_controller.yaml)
generate_parameter_library(full_riptide_controller_parameters src/full_riptide_controller.yaml)

add_library(${PROJECT_NAME} SHARED src/riptide_controller.cpp src/full_riptide_controller.cpp src/depth_controller.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include)
target_link_libraries(${PROJECT_NAME} riptide_controller_parameters depth_controller_parameters full_riptide_controller_parameters Eigen3::Eigen)
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
