cmake_minimum_required(VERSION 3.5)
project(skyways)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

ament_export_dependencies(rosidl_default_runtime)

add_executable(drone_server src/drone_server.cpp)
ament_target_dependencies(drone_server
  rclcpp geometry_msgs std_msgs rosidl_default_runtime
)

add_executable(drone_client src/drone_client.cpp)
ament_target_dependencies(drone_client
  rclcpp geometry_msgs std_msgs rosidl_default_runtime
)

add_executable(waypoint_logger src/waypoint_logger.cpp)
ament_target_dependencies(waypoint_logger
  rclcpp sensor_msgs rosidl_default_runtime
)

add_executable(switch src/switch.cpp)
ament_target_dependencies(switch
  rclcpp sensor_msgs rosidl_default_runtime
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DataPacket.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_target_interfaces(drone_server  
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(drone_client  
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS 
    drone_server 
    drone_client 
    waypoint_logger
    switch
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
