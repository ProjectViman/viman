cmake_minimum_required(VERSION 2.8.3)
project(viman_control)

find_package(catkin REQUIRED COMPONENTS 
			 roscpp
			 rospy
			 std_msgs
			 sensor_msgs
			 geometry_msgs
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
                    include
                    include/plugin
                    include/utility
                    )


set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

###################################
catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp rospy image_transport std_msgs sensor_msgs geometry_msgs message_runtime gazebo_ros
)

include_directories(${catkin_INCLUDE_DIRS})

############# Plugin for simulating flight #############
add_library(plugin_viman_flight SHARED
    src/plugin/plugin_viman_flight.cpp
    src/plugin/pid_controller.cpp
)

target_link_libraries(plugin_viman_flight
   ${catkin_LIBRARIES}
   ${GAZEBO_LIBRARIES}
)

set_target_properties(plugin_viman_flight
    PROPERTIES 
    LIBRARY_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/plugins
)

############# Plugin for simulating altimeter #############
add_library(plugin_altimeter SHARED
    src/plugin/plugin_altimeter.cpp
)

target_link_libraries(plugin_altimeter
   ${catkin_LIBRARIES}
   ${GAZEBO_LIBRARIES}
)

set_target_properties(plugin_altimeter
    PROPERTIES 
    LIBRARY_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/plugins
)

#### ROS node for manual operation ####
add_executable(viman_key_ctrl src/viman_key_ctrl.cpp src/viman.cpp)
target_link_libraries(viman_key_ctrl ${catkin_LIBRARIES})

#### ROS node for semi-autonomous operation ####
add_executable(viman_sa src/viman_sa.cpp src/vm_pid_linear.cpp src/vm_pid_rotate.cpp src/viman.cpp)
target_link_libraries(viman_sa ${catkin_LIBRARIES})

#### Utility ROS node to observe sensor data (except camera) ####
add_executable(vm_sensor_data src/utility/vm_sensor_data.cpp)
target_link_libraries(vm_sensor_data ${catkin_LIBRARIES})

install(DIRECTORY include include/plugin include/utility plugins src src/plugin src/utility
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
