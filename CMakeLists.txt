cmake_minimum_required(VERSION 2.8.3)
project(rviz_camera_stream)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  rviz
  sensor_msgs
)

## Compiler flags
# if(CMAKE_COMPILER_IS_GNUCXX)
#  set(CMAKE_CXX_FLAGS "-std=c++0x")  ## Enable C++11
# endif()

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

roslint_cpp()

## Avoid defining "emit", "slots",
## etc for Qt signals because they can conflict with boost signals
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

qt4_wrap_cpp(MOC_FILES
  src/camera_display.h
  )

add_library(rviz_camera_stream
  src/camera_display.cpp
  ${MOC_FILES}
)

target_link_libraries(rviz_camera_stream
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)
