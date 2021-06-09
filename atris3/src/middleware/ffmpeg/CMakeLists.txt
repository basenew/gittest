cmake_minimum_required(VERSION 2.8.3)
project(ffmpeg)

execute_process(
  COMMAND mkdir -p ${CATKIN_DEVEL_PREFIX}/include
  COMMAND mkdir -p ${CATKIN_DEVEL_PREFIX}/lib/ffmpeg
  COMMAND mkdir -p ${CATKIN_DEVEL_PREFIX}/bin
  #COMMAND cp -a ${PROJECT_SOURCE_DIR}/include/. ${CATKIN_DEVEL_PREFIX}/include
  COMMAND cp -a ${PROJECT_SOURCE_DIR}/lib/. ${CATKIN_DEVEL_PREFIX}/lib/ffmpeg
  COMMAND cp -a ${PROJECT_SOURCE_DIR}/bin/. ${CATKIN_DEVEL_PREFIX}/bin/
)
        
catkin_package(
  INCLUDE_DIRS include
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/.
   DESTINATION include/ffmpeg
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/lib/.
   DESTINATION lib/ffmpeg
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/bin/.
   DESTINATION lib/ffmpeg
)
