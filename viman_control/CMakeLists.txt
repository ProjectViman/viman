cmake_minimum_required(VERSION 2.8.3)
project(viman_control)

find_package(catkin REQUIRED COMPONENTS 
			 roscpp
			 rospy
			 std_msgs
			 image_transport
			 message_generation)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
endif()


find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Protobuf REQUIRED)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
    pkg_check_modules(OGRE OGRE)
    pkg_check_modules(OGRE-Terrain OGRE-Terrain)
endif()

link_directories(${GAZEBO_LIBRARY_DIRS} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${OGRE_LIBRARY_DIRS})
include_directories(${OpenCV_INCLUDE_DIR} 
                    ${Boost_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${GAZEBO_INCLUDE_DIRS}
                    ${OGRE_INCLUDE_DIRS}
                    ${OGRE-Terrain_INCLUDE_DIRS}
                    include)


set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

###################################
catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp rospy image_transport std_msgs sensor_msgs geometry_msgs message_runtime gazebo_ros
)

include_directories(${catkin_INCLUDE_DIRS})

################## A simple model controller #############
add_library(plugin_viman_ctrl SHARED
    src/plugin_viman_ctrl.cpp
    src/pid_controller.cpp
    include/viman_ctrl.h
    include/pid_controller.h
)

target_link_libraries(plugin_viman_ctrl
   ${catkin_LIBRARIES}
   ${GAZEBO_LIBRARIES}
)

set_target_properties(plugin_viman_ctrl
    PROPERTIES 
    LIBRARY_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/plugins
)


install(DIRECTORY include launch plugins src scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
