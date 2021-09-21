cmake_minimum_required(VERSION 2.8.3)
project(rviz_camera_stream)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  roslint
  rviz
  sensor_msgs
)

roslint_cpp()
## Compiler flags
# if(CMAKE_COMPILER_IS_GNUCXX)
#  set(CMAKE_CXX_FLAGS "-std=c++0x")  ## Enable C++11
# endif()

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

## Avoid defining "emit", "slots",
## etc for Qt signals because they can conflict with boost signals
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rviz
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

if(rviz_QT_VERSION VERSION_LESS "5")
  qt4_wrap_cpp(MOC_FILES
    include/${PROJECT_NAME}/camera_display.h
  )
else()
  qt5_wrap_cpp(MOC_FILES
    include/${PROJECT_NAME}/camera_display.h
  )
endif()

add_library(${PROJECT_NAME}
  src/camera_display.cpp
  ${MOC_FILES}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)

# install
install (TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
