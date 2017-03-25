cmake_minimum_required(VERSION 2.8.3)
project(mobility)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  random_numbers
<<<<<<< HEAD
  tf
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs random_numbers tf
=======
  message_generation
  shared_messages
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs random_numbers message_runtime shared_messages
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
<<<<<<< HEAD
  mobility 
  src/PickUpController.cpp
  src/DropOffController.cpp
  src/SearchController.cpp
  src/mobility.cpp
=======
  mobility src/mobility.cpp
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
)

add_dependencies(mobility ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  mobility
  ${catkin_LIBRARIES}
)

