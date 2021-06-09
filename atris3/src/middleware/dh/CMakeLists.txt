cmake_minimum_required(VERSION 2.8.3)
project(dh)

execute_process(
  COMMAND mkdir -p ${CATKIN_DEVEL_PREFIX}/include
  COMMAND mkdir -p ${CATKIN_DEVEL_PREFIX}/lib
  COMMAND cp -a ${PROJECT_SOURCE_DIR}/include/. ${CATKIN_DEVEL_PREFIX}/include
  COMMAND cp -a ${PROJECT_SOURCE_DIR}/lib/. ${CATKIN_DEVEL_PREFIX}/lib
)
        
catkin_package(
  INCLUDE_DIRS include
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/.
   DESTINATION include
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/lib/.
   DESTINATION lib
)
