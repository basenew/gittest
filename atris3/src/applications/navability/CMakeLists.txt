cmake_minimum_required(VERSION 2.8.3)
project(navability)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  log
  atris_msgs
  jsoncpp
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

find_package(PkgConfig REQUIRED)
pkg_check_modules(GLIB_PKG glib-2.0)

IF (GLIB_PKG_FOUND)
    message(Found glib-2.0)
    include_directories(${GLIB_PKG_INCLUDE_DIRS})
ENDIF()

pkg_check_modules(GST_PKG gstreamer-1.0)

IF (GST_PKG_FOUND)
    message(Found gstreamer-1.0)
    include_directories(${GST_PKG_INCLUDE_DIRS})
ENDIF()

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  src
  src/porting
  src/porting/xs
  src/porting/gs
  src/porting/ubt
  src/3rd/shttpd-1.42
  src/3rd/uWebSockets-0.6.0
  src/3rd/tinyros_client_library
  ${catkin_INCLUDE_DIRS}
  ${CATKIN_DEVEL_PREFIX}/include
  ${CATKIN_DEVEL_PREFIX}/include/tinyros_client_library/gcc/include
)

aux_source_directory(src sources)
aux_source_directory(src/porting/xs sources)
aux_source_directory(src/porting/gs sources)
aux_source_directory(src/porting/ubt sources)
aux_source_directory(src/3rd/uWebSockets-0.6.0 sources)

add_executable(navability ${sources}
    src/3rd/shttpd-1.42/string.c
    src/3rd/shttpd-1.42/shttpd.c
    src/3rd/shttpd-1.42/log.c
    src/3rd/shttpd-1.42/auth.c
    src/3rd/shttpd-1.42/md5.c
    src/3rd/shttpd-1.42/cgi.c
    src/3rd/shttpd-1.42/io_ssi.c
    src/3rd/shttpd-1.42/io_file.c
    src/3rd/shttpd-1.42/io_socket.c
    src/3rd/shttpd-1.42/io_ssl.c
    src/3rd/shttpd-1.42/io_emb.c
    src/3rd/shttpd-1.42/io_dir.c
    src/3rd/shttpd-1.42/io_cgi.c
    src/3rd/shttpd-1.42/io_cgi.c
    src/3rd/shttpd-1.42/compat_unix.c
    ${CATKIN_DEVEL_PREFIX}/include/tinyros_client_library/gcc/include/tiny_ros/time.cpp
    ${CATKIN_DEVEL_PREFIX}/include/tinyros_client_library/gcc/include/tiny_ros/duration.cpp
    ${CATKIN_DEVEL_PREFIX}/include/tinyros_client_library/gcc/include/tiny_ros/ros.cpp
    ${CATKIN_DEVEL_PREFIX}/include/tinyros_client_library/gcc/include/tiny_ros/log.cpp
)

add_dependencies(navability ${catkin_EXPORTED_TARGETS})

target_link_libraries(navability
  ${catkin_LIBRARIES}
  libpthread.so
  libdl.so
  libz.so
  libuv.so
  libssl.so
  libcrypto.so
  ${CATKIN_DEVEL_PREFIX}/lib/libprotobuf.so
  ${CATKIN_DEVEL_PREFIX}/lib/libxstsdk.so
)

execute_process(
  COMMAND mkdir -p ${CATKIN_DEVEL_PREFIX}/lib
  COMMAND cp -a ${PROJECT_SOURCE_DIR}/src/porting/xs/lib/. ${CATKIN_DEVEL_PREFIX}/lib
)

install(TARGETS navability
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/src/porting/xs/lib/.
   DESTINATION lib
)

