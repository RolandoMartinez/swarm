cmake_minimum_required(VERSION 2.8.3)
project(rqt_rover_gui)

set(CMAKE_BUILD_TYPE Debug)

<<<<<<< HEAD
#Get version info from git and pass it into compiled code
execute_process(COMMAND git rev-parse --abbrev-ref HEAD
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_BRANCH)
string(STRIP "${GIT_BRANCH}" GIT_BRANCH)

execute_process(COMMAND git describe --tags --abbrev=0
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_VERSION)
string(STRIP "${GIT_VERSION}" GIT_VERSION)
=======
execute_process(COMMAND git describe --tags --abbrev=0
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_VERSION)
string(STRIP ${GIT_VERSION} GIT_VERSION)
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

execute_process(COMMAND date "+%Y-%m-%d-%H:%M"
    OUTPUT_VARIABLE BUILD_TIME)
string(STRIP ${BUILD_TIME} BUILD_TIME)

<<<<<<< HEAD
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/src/Version.cpp.in ${CMAKE_CURRENT_BINARY_DIR}/Version.cpp)
set(version_file "${CMAKE_CURRENT_BINARY_DIR}/Version.cpp")
=======
set_source_files_properties(src/rover_gui_plugin.cpp PROPERTIES COMPILE_DEFINITIONS "GIT_VERSION=\"${GIT_VERSION}\";BUILD_TIME=\"${BUILD_TIME}\"")
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS 
  rqt_gui
  rqt_gui_cpp
  cv_bridge
  image_transport
<<<<<<< HEAD
  geometry_msgs
  ublox_msgs
  ublox_serialization
=======
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
)

find_package(Qt4 REQUIRED COMPONENTS
  QtCore
  QtGui
  QtXml
)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

catkin_package(
<<<<<<< HEAD
  CATKIN_DEPENDS rqt_gui rqt_gui_cpp cv_bridge image_transport geometry_msgs
=======
  CATKIN_DEPENDS rqt_gui rqt_gui_cpp cv_bridge image_transport
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
)

SET(rover_gui_plugin_RESOURCES resources/resources.qrc)

qt4_add_resources(rqt_rover_gui_resources_rcc ${rover_gui_plugin_RESOURCES})

qt4_wrap_cpp(
  rover_gui_plugin_MOCS
  src/rover_gui_plugin.h
  src/CameraFrame.h
  src/MapFrame.h
  src/USFrame.h
  src/GPSFrame.h
  src/IMUFrame.h
<<<<<<< HEAD
  src/JoystickGripperInterface.h
=======
  #src/IMUWidget.h
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
)

qt4_wrap_ui(
  rover_gui_plugin_UIS_H
  src/rover_gui_plugin.ui
)

include(
  ${QT_USE_FILE}
)

include_directories(
  include
  src
  ${CMAKE_CURRENT_BINARY_DIR}
)

link_directories(
  lib
)

add_library(
  rqt_rover_gui
<<<<<<< HEAD
  ${version_file}
  src/GazeboSimManager.cpp
  src/JoystickGripperInterface.cpp
=======
  src/GazeboSimManager.cpp
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
  src/rover_gui_plugin.cpp
  src/CameraFrame.cpp
  src/MapFrame.cpp
  src/USFrame.cpp
  src/GPSFrame.cpp
<<<<<<< HEAD
  src/MapData.cpp
=======
  #src/IMUWidget.cpp
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
  src/IMUFrame.cpp
  src/BWTabWidget.cpp
  ${rover_gui_plugin_RESOURCES}
  ${rover_gui_plugin_MOCS}
  ${rover_gui_plugin_UIS_H}
  ${OpenCV_LIBS}
)

<<<<<<< HEAD
add_dependencies(
  rqt_rover_gui ${catkin_EXPORTED_TARGETS}
  ${ublox_msgs_EXPORTED_TARGETS}
)

target_link_libraries(
  rqt_rover_gui
=======
target_link_libraries(
  rqt_rover_gui
  libapriltag.a	
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
  ${GAZEBO_libraries}
  ${catkin_LIBRARIES}
)

catkin_python_setup()

set(CMAKE_BUILD_TYPE Debug)
