cmake_minimum_required(VERSION 3.8)
project(mercury_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclpy REQUIRED)
find_package(riptide_msgs2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

#thruster test script
add_executable(thruster_test_canbus models/complete_system/send_thruster_cmd_canbus.c)
target_compile_definitions(thruster_test_canbus PUBLIC ENABLE_THRUSTER_TEST) #enable the thruster test code in send_thruster_cmd_canbus.c

add_executable(controller_overseer src/controller_overseer.cpp src/simulink_model.cpp)


target_include_directories(controller_overseer PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/mercury_controller>
    $<INSTALL_INTERFACE:include>
    ${YAML_CPP_INCLUDEDIR})

ament_target_dependencies(controller_overseer
  rclcpp
  rcl_interfaces
  std_srvs
)


install(TARGETS thruster_test_canbus
  DESTINATION lib/${PROJECT_NAME})

# yoink python programs from python source
file(GLOB src_py_files RELATIVE ${PROJECT_SOURCE_DIR} mercury_controller/*.py)
install(PROGRAMS ${src_py_files}
  DESTINATION lib/${PROJECT_NAME})

# yoink python programs from models directory
file(GLOB model_py_files RELATIVE ${PROJECT_SOURCE_DIR} models/*.py)
install(PROGRAMS ${model_py_files}
  DESTINATION lib/${PROJECT_NAME})

#Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

#install 
install(DIRECTORY config 
  DESTINATION share/${PROJECT_NAME}
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
