cmake_minimum_required(VERSION 2.8.3)
project(gazebo_plugins)

<<<<<<< HEAD
set(CMAKE_CXX_FLAGS "-std=c++0x")

=======
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros 
)

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
<<<<<<< HEAD
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} src src/GripperPlugin ${CMAKE_CURRENT_BINARY_DIR})

add_library(${PROJECT_NAME} src/SetupWorld.cpp)

add_library(${PROJECT_NAME}_gripper 
  src/GripperPlugin/GripperPlugin.cpp
  src/GripperPlugin/PIDController.cpp 
  src/GripperPlugin/GripperManager.cpp)

add_library(${PROJECT_NAME}_score
  src/ScorePlugin/ScorePlugin.cpp)

=======
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/SetupWorld.cpp)
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

