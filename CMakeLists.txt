cmake_minimum_required(VERSION 3.8)
project(jetracer_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)


# NODE
add_executable(jet_racer_node 
                src/jet_racer_node.cpp
                src/jet_racer_api.cpp
                src/configuration.cpp)
target_include_directories(jet_racer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(jet_racer_node PUBLIC c_std_99 cxx_std_17) 
ament_target_dependencies(jet_racer_node
  rclcpp
  serial
)

add_executable(test_serial 
                src/test_serial.cpp
                src/jet_racer_api.cpp
                src/configuration.cpp)
target_include_directories(test_serial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(test_serial PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(test_serial
  rclcpp
  serial
)


install(TARGETS jet_racer_node test_serial
  DESTINATION lib/${PROJECT_NAME})




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

#ament_export_targets(
#  export_${PROJECT_NAME}
#)

ament_package()
