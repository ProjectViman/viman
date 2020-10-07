cmake_minimum_required(VERSION 2.8.3)
project(viman_utility)

find_package(catkin REQUIRED COMPONENTS 
			 roscpp
			 rospy
			 std_msgs
			 message_generation)

include_directories(${catkin_INCLUDE_DIRS})

add_message_files(
	FILES
	CamZ.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

install(DIRECTORY msg DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
