cmake_minimum_required(VERSION 2.8.3)
project(devices)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  log
  aisoundlib
  atris_msgs
  libmediaplayer
  tts_strings
  sound
  utils
  database
  imemory
  config
  md5 
  rtc 
  jsoncpp
  eeprom
  can
  transferfile
  task_manager
  platform
  tinyros
  libxml
  cpp-base64
  ffmpeg
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  #DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

find_package(PkgConfig REQUIRED)
if (GLIB_PKG_FOUND)
    message(Found glib-2.0)
    include_directories(${GLIB_PKG_INCLUDE_DIRS})
endif()

pkg_check_modules(GST_PKG gstreamer-1.0)

if (GST_PKG_FOUND)
    message(Found gstreamer-1.0)
    include_directories(${GST_PKG_INCLUDE_DIRS})
endif()

include_directories(
  ./
  robot
  abi
  ppplay
  disperse
  chassis
  dev_manager
  flash_lamp
  peripheral_service
  dbcom
  power_manager
  router
  ptz
  mobile_detector
  telecom
  voip
  can_service
  ultrasound
  negotiates
  imu
  gps
  ${catkin_INCLUDE_DIRS}
  ${CATKIN_DEVEL_PREFIX}/include
  ${CATKIN_DEVEL_PREFIX}/include/tinyros_client_library/gcc/include
)

aux_source_directory(./ sources)
aux_source_directory(robot sources)
aux_source_directory(abi sources)
aux_source_directory(disperse sources)
aux_source_directory(ppplay sources)
aux_source_directory(chassis sources)
aux_source_directory(flash_lamp sources)
aux_source_directory(power_manager sources)
aux_source_directory(router sources)
aux_source_directory(ptz sources)
aux_source_directory(mobile_detector sources)
aux_source_directory(telecom sources)
aux_source_directory(voip sources)
aux_source_directory(can_service sources)
aux_source_directory(ultrasound sources)
aux_source_directory(negotiates sources)
aux_source_directory(imu sources)
aux_source_directory(gps sources)
aux_source_directory(peripheral_service sources)
aux_source_directory(dbcom sources)

add_executable(devices ${sources}
  ${CATKIN_DEVEL_PREFIX}/include/tinyros_client_library/gcc/include/tiny_ros/time.cpp
  ${CATKIN_DEVEL_PREFIX}/include/tinyros_client_library/gcc/include/tiny_ros/duration.cpp
  ${CATKIN_DEVEL_PREFIX}/include/tinyros_client_library/gcc/include/tiny_ros/ros.cpp
  ${CATKIN_DEVEL_PREFIX}/include/tinyros_client_library/gcc/include/tiny_ros/log.cpp
)

add_dependencies(devices ${catkin_EXPORTED_TARGETS})

target_link_libraries(devices
  ${catkin_LIBRARIES}
  ${CATKIN_DEVEL_PREFIX}/lib/libdhnetsdk.so
  ${CATKIN_DEVEL_PREFIX}/lib/libhcnetsdk.so
  ${CATKIN_DEVEL_PREFIX}/lib/libAudioRender.so
  ${CATKIN_DEVEL_PREFIX}/lib/libHCCore.so
  ${CATKIN_DEVEL_PREFIX}/lib/libhpr.so
  ${CATKIN_DEVEL_PREFIX}/lib/libNPQos.so
  ${CATKIN_DEVEL_PREFIX}/lib/libSuperRender.so
  ${CATKIN_DEVEL_PREFIX}/lib/libz.so
  ${CATKIN_DEVEL_PREFIX}/lib/libPlayCtrl.so
)

install(TARGETS devices
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

