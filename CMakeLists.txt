cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init()
rosbuild_add_boost_directories()

## Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "-std=c++0x")  ## Enable C++11
  #set(CMAKE_CXX_FLAGS "-O2")        ## Optimize
  #set(CMAKE_EXE_LINKER_FLAGS "-s")  ## Strip binary
endif()

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

pkg_check_modules(OGRE OGRE) 
include_directories( ${OGRE_INCLUDE_DIRS} ) 
link_directories( ${OGRE_LIBRARY_DIRS} ) 

## Avoid defining "emit", "slots",
## etc for Qt signals because they can conflict with boost signals
add_definitions(-DQT_NO_KEYWORDS)

qt4_wrap_cpp(MOC_FILES
  src/camera_display.h
  )

set(SOURCE_FILES
  src/camera_display.cpp
  ${MOC_FILES}
  )

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(${PROJECT_NAME} ${SOURCE_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES
  DEFINE_SYMBOL THIS_PACKAGE_NAME="${PROJECT_NAME}")

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${OGRE_LIBRARIES})
