cmake_minimum_required(VERSION 3.5)
project(publisher_package)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add the publisher executable
add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher rclcpp geometry_msgs)

# Add the subscriber executable
add_executable(subscriber src/subscriber.cpp)
ament_target_dependencies(subscriber rclcpp geometry_msgs)

# Install the executables
install(TARGETS
  publisher
  subscriber
  DESTINATION lib/${PROJECT_NAME})

# ament package
ament_package()
