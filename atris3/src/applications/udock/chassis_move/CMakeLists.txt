cmake_minimum_required(VERSION 2.8.3)
project(chassis_move)
add_definitions("-Wall -g")

find_package(catkin REQUIRED COMPONENTS 
		roscpp
		std_srvs
		udock_msgs
		rospy
		actionlib_msgs
		actionlib
		grid_map_core
		grid_map_ros
		grid_map_cv
		grid_map_filters
		grid_map_loader
		grid_map_msgs
		grid_map_octomap
		grid_map_rviz_plugin
		grid_map_visualization
		pcl_ros
		jsoncpp
	)

find_package(Eigen3 REQUIRED)

set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl")
find_package(PCL 1.8 REQUIRED)

catkin_package(	

)

include_directories(
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})


#move
# add_executable(${PROJECT_NAME}_node src/chassis_move.cpp src/collision_detector.cpp)
# add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS} u_msgs_gencpp)
# target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} )

# add_executable(${PROJECT_NAME}_interface src/chassis_move_interface.cpp)
# add_dependencies(${PROJECT_NAME}_interface ${catkin_EXPORTED_TARGETS} u_msgs_gencpp)
# target_link_libraries(${PROJECT_NAME}_interface ${catkin_LIBRARIES} ) 

# add_executable(${PROJECT_NAME}_test src/chassis_move_test.cpp)
# add_dependencies(${PROJECT_NAME}_test ${catkin_EXPORTED_TARGETS} u_msgs_gencpp)
# target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES})

add_executable(local_map_builder_node src/local_map_builder.cpp src/local_map_builder.cpp src/local_map_builder_node.cpp)
add_dependencies(local_map_builder_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(local_map_builder_node ${catkin_LIBRARIES}  ${PCL_LIBRARIES})

add_executable(collision_detector_test_node src/collision_detector_test_node.cpp src/collision_detector.cpp )
add_dependencies(collision_detector_test_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(collision_detector_test_node ${catkin_LIBRARIES})



#install ${PROJECT_NAME}_test
# install(TARGETS ${PROJECT_NAME}_node local_map_builder_node collision_detector_test_node ${PROJECT_NAME}_interface
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
# install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})