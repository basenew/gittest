cmake_minimum_required(VERSION 2.8.3)
project(tinyros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
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
  #LIBRARIES atris_qiniu
  CATKIN_DEPENDS roscpp
  #DEPENDS system_lib
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -O3")

set(ROSLIB_PATHS ${CATKIN_DEVEL_PREFIX}/include/tinyros_client_library/gcc/include/tiny_ros)

add_definitions(-DSPDLOG_COMPILED_LIB)

aux_source_directory(${PROJECT_SOURCE_DIR}/spdlog/src SPDLOG_SRC_LIST)

aux_source_directory(${PROJECT_SOURCE_DIR}/uWebSockets UWS_SRC_LIST)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CATKIN_DEVEL_PREFIX}/include/tinyros_client_library/gcc/include
  ${PROJECT_SOURCE_DIR}/spdlog/include
  ${PROJECT_SOURCE_DIR}/uWebSockets
  ${PROJECT_SOURCE_DIR}/core/include
)

## tinyrosdds
aux_source_directory(${PROJECT_SOURCE_DIR}/core/src tinyrosdds_SRC_LIST)
add_executable(tinyrosdds 
  ${UWS_SRC_LIST} 
  ${SPDLOG_SRC_LIST}
  ${ROSLIB_PATHS}/time.cpp 
  ${tinyrosdds_SRC_LIST})

target_link_libraries(tinyrosdds 
  ${catkin_LIBRARIES} 
  libpthread.so 
  libz.so 
  libuv.so 
  libssl.so 
  libcrypto.so
)

## tinyrosconsole
add_executable(tinyrosconsole
  ${PROJECT_SOURCE_DIR}/tools/rosconsole/rosconsole.cpp
  ${ROSLIB_PATHS}/ros.cpp
  ${ROSLIB_PATHS}/time.cpp
  ${ROSLIB_PATHS}/duration.cpp
  ${ROSLIB_PATHS}/log.cpp
  ${SPDLOG_SRC_LIST}
)
target_link_libraries(tinyrosconsole ${catkin_LIBRARIES} libpthread.so)

## tinyrosservice
add_executable(tinyrosservice
  ${PROJECT_SOURCE_DIR}/tools/rosservice/rosservice.cpp
  ${ROSLIB_PATHS}/ros.cpp
  ${ROSLIB_PATHS}/time.cpp
  ${ROSLIB_PATHS}/duration.cpp
  ${ROSLIB_PATHS}/log.cpp
)
target_link_libraries(tinyrosservice ${catkin_LIBRARIES} libpthread.so)

## tinyrostopic
add_executable(tinyrostopic
  ${PROJECT_SOURCE_DIR}/tools/rostopic/rostopic.cpp
  ${ROSLIB_PATHS}/ros.cpp
  ${ROSLIB_PATHS}/time.cpp
  ${ROSLIB_PATHS}/duration.cpp
  ${ROSLIB_PATHS}/log.cpp
  ${SPDLOG_SRC_LIST}
)
target_link_libraries(tinyrostopic ${catkin_LIBRARIES} libpthread.so)

## ExamplePublisher
add_executable(ExamplePublisher
  ${PROJECT_SOURCE_DIR}/examples/publisher/ExamplePublisher.cpp
  ${ROSLIB_PATHS}/ros.cpp
  ${ROSLIB_PATHS}/time.cpp
  ${ROSLIB_PATHS}/duration.cpp
  ${ROSLIB_PATHS}/log.cpp
)
target_link_libraries(ExamplePublisher ${catkin_LIBRARIES} libpthread.so)

## ExampleService
add_executable(ExampleService
  ${PROJECT_SOURCE_DIR}/examples/service/ExampleService.cpp
  ${ROSLIB_PATHS}/ros.cpp
  ${ROSLIB_PATHS}/time.cpp
  ${ROSLIB_PATHS}/duration.cpp
  ${ROSLIB_PATHS}/log.cpp
)
target_link_libraries(ExampleService ${catkin_LIBRARIES} libpthread.so)

## ExampleServiceClient
add_executable(ExampleServiceClient
  ${PROJECT_SOURCE_DIR}/examples/service_client/ExampleServiceClient.cpp
  ${ROSLIB_PATHS}/ros.cpp
  ${ROSLIB_PATHS}/time.cpp
  ${ROSLIB_PATHS}/duration.cpp
  ${ROSLIB_PATHS}/log.cpp
)
target_link_libraries(ExampleServiceClient ${catkin_LIBRARIES} libpthread.so)

## ExampleSubscriber
add_executable(ExampleSubscriber
  ${PROJECT_SOURCE_DIR}/examples/subscriber/ExampleSubscriber.cpp
  ${ROSLIB_PATHS}/ros.cpp
  ${ROSLIB_PATHS}/time.cpp
  ${ROSLIB_PATHS}/duration.cpp
  ${ROSLIB_PATHS}/log.cpp
)
target_link_libraries(ExampleSubscriber ${catkin_LIBRARIES} libpthread.so)

###########
## install ##
###########
install(TARGETS tinyrosdds tinyrosconsole tinyrosservice tinyrostopic ExamplePublisher ExampleService ExampleServiceClient ExampleSubscriber
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/tinyros_client_library DESTINATION include)

