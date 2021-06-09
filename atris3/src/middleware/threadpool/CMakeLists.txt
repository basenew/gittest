cmake_minimum_required(VERSION 2.8.3)
project(threadpool)

execute_process(
  COMMAND mkdir -p ${CATKIN_DEVEL_PREFIX}/include
  COMMAND cp -a ${PROJECT_SOURCE_DIR}/include/. ${CATKIN_DEVEL_PREFIX}/include
)
        
catkin_package(
  INCLUDE_DIRS include
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/.
   DESTINATION include
)
