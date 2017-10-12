cmake_minimum_required(VERSION 2.6 FATAL_ERROR)


project(genie)

#additional cmake definitions
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

#build the ROS interface if using catkin
if(CATKIN_ENV)
	
	find_package(catkin REQUIRED COMPONENTS
		roscpp
		std_msgs
		sensor_msgs
		cv_bridge
		image_transport
		)

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
	#  INCLUDE_DIRS include
	#  LIBRARIES genie_ros
		CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge image_transport
	#  DEPENDS system_lib
	)
	
	add_subdirectory(genie_ros)

endif(CATKIN_ENV)

add_subdirectory(GenieLib)
add_subdirectory(Apps)

