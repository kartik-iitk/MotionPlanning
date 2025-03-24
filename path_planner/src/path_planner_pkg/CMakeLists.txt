cmake_minimum_required(VERSION 3.5)
project(path_planner_pkg)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)

# Additional dependencies
find_package(ompl REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(ACADO REQUIRED)

# Add the executable
add_executable(main src/main.cpp src/Visualize.cpp)

# Include directories for additional dependencies
target_include_directories(main PUBLIC
  include
  ${OMPL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${ACADO_INCLUDE_DIRS}
)

# Link against additional libraries
target_link_libraries(main
  ${OMPL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  Boost::program_options
  ${ACADO_SHARED_LIBRARIES}
)

# Link against ROS2 dependencies
ament_target_dependencies(main rclcpp geometry_msgs std_msgs pluginlib)

# Install the executable
install(TARGETS main
  DESTINATION lib/${PROJECT_NAME}
)

# Ament package declaration
ament_package()
