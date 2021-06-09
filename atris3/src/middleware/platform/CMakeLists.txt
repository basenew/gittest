cmake_minimum_required(VERSION 2.8.3)
project(platform)

execute_process(
  COMMAND mkdir -p ${CATKIN_DEVEL_PREFIX}/include/platform
  COMMAND cp -a ${PROJECT_SOURCE_DIR}/include/. ${CATKIN_DEVEL_PREFIX}/include/platform
)
        
catkin_package(
  INCLUDE_DIRS include
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/.
   DESTINATION include/platform
)

