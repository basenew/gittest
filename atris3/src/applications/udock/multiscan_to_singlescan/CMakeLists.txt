cmake_minimum_required(VERSION 2.8.3)
project(scan_trans)
# add_definitions("-Wall -g")

find_package(catkin REQUIRED COMPONENTS 
		roscpp
		std_srvs
		rospy
		pcl_ros
 		pcl_conversions
		sensor_msgs
	)

catkin_package(	

)

include_directories(
	${catkin_INCLUDE_DIRS}
	# ${EIGEN3_INCLUDE_DIRS}
	# ${PCL_INCLUDE_DIRS}
)

#
add_executable(${PROJECT_NAME}_node src/scan_trans.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} )

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
