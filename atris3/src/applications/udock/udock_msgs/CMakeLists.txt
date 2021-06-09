cmake_minimum_required(VERSION 2.8.3)
project(udock_msgs)

find_package(catkin REQUIRED roscpp sensor_msgs std_msgs nav_msgs message_generation)

#  add_message_files(
#    FILES
# )



# add_service_files(
#   FILES
# )

add_action_files(
  FILES
  MoveTo.action
)

 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   sensor_msgs
   nav_msgs
   geometry_msgs
 )

# FILE(GLOB_RECURSE MsgFiles "msg/*.msg")
# add_custom_target(udock_msgs SOURCES ${MsgFiles})
# FILE(GLOB_RECURSE SrvFiles "srv/*.srv")
# add_custom_target(udock_srvs SOURCES ${SrvFiles})

#FILE(GLOB_RECURSE CfgFiles "cfg/*.cfg")
#add_custom_target(cruiserCfgFiles SOURCES ${CfgFiles})

catkin_package(
# INCLUDE_DIRS include
#  LIBRARIES u_msgs
  CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)


## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES   ${LaunchFiles}
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )
    
