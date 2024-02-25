cmake_minimum_required(VERSION 3.5)
project(my_robot_bringup)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cartesian_controller_base REQUIRED)
find_package(cartesian_compliance_controller REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(controller_manager REQUIRED)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

#pluginlib_export_plugin_description_file(hardware_interface cartesian_controller_simulation.xml)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

# target_link_libraries(${ament_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_package()
