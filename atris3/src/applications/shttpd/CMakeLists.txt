cmake_minimum_required(VERSION 2.8.3)
project(shttpd)

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
  websocket
  tinyros
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
include_directories(
  ./
  shttpd-1.42/src
  ${catkin_INCLUDE_DIRS}
  ${CATKIN_DEVEL_PREFIX}/include
  ${CATKIN_DEVEL_PREFIX}/include/tinyros_client_library/gcc/include
)

add_executable(shttpd 
  ${CATKIN_DEVEL_PREFIX}/include/tinyros_client_library/gcc/include/tiny_ros/time.cpp
  ${CATKIN_DEVEL_PREFIX}/include/tinyros_client_library/gcc/include/tiny_ros/duration.cpp
  ${CATKIN_DEVEL_PREFIX}/include/tinyros_client_library/gcc/include/tiny_ros/ros.cpp
  ${CATKIN_DEVEL_PREFIX}/include/tinyros_client_library/gcc/include/tiny_ros/log.cpp
  shttpd-1.42/src/string.c
  shttpd-1.42/src/shttpd.c
  shttpd-1.42/src/log.c
  shttpd-1.42/src/auth.c
  shttpd-1.42/src/md5.c
  shttpd-1.42/src/cgi.c
  shttpd-1.42/src/io_ssi.c
  shttpd-1.42/src/io_file.c
  shttpd-1.42/src/io_socket.c
  shttpd-1.42/src/io_ssl.c
  shttpd-1.42/src/io_emb.c
  shttpd-1.42/src/io_dir.c
  shttpd-1.42/src/io_cgi.c
  shttpd-1.42/src/io_cgi.c
  shttpd-1.42/src/compat_unix.c
  main.cpp
)

add_dependencies(shttpd ${catkin_EXPORTED_TARGETS})

target_link_libraries(shttpd
  libdl.so
  ${catkin_LIBRARIES}
  ${CATKIN_DEVEL_PREFIX}/lib/libatris_websockets.so
)

execute_process(
  COMMAND mkdir -p ${CATKIN_DEVEL_PREFIX}/bin
  COMMAND cp -a ${PROJECT_SOURCE_DIR}/dist ${CATKIN_DEVEL_PREFIX}/bin
)

install(TARGETS shttpd
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/dist
   DESTINATION bin
)

