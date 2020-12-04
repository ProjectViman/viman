cmake_minimum_required(VERSION 2.8.3)
project(viman_control)

find_package(catkin REQUIRED COMPONENTS 
			 roscpp
			 rospy
			 std_msgs
			 sensor_msgs
			 geometry_msgs
       viman_utility
			 image_transport)

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
include_directories( 
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
   CATKIN_DEPENDS roscpp rospy image_transport std_msgs sensor_msgs geometry_msgs message_runtime gazebo_ros viman_utility
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

############# Libraries for nodes #############

set(NODE_LIBS ${PROJECT_SOURCE_DIR}/lib)

# Library to control viman's instance
add_library(viman STATIC src/viman.cpp)
target_link_libraries(viman ${catkin_LIBRARIES})
set_target_properties(viman 
                      PROPERTIES
                      ARCHIVE_OUTPUT_DIRECTORY
                      ${NODE_LIBS}
)

# Library for PID
add_library(vm_pid STATIC src/vm_pid_linear.cpp src/vm_pid_rotate.cpp)
set_target_properties(vm_pid 
                      PROPERTIES
                      ARCHIVE_OUTPUT_DIRECTORY
                      ${NODE_LIBS}
)

# Library for map storage
add_library(map STATIC src/utility/Map.cpp)
set_target_properties(map
                      PROPERTIES
                      ARCHIVE_OUTPUT_DIRECTORY
                      ${NODE_LIBS}
)

############# ROS Nodes #############

#### ROS node for manual operation ####
add_executable(viman_key_ctrl src/viman_key_ctrl.cpp)
target_link_libraries(viman_key_ctrl ${catkin_LIBRARIES} viman)

#### ROS node for semi-autonomous operation ####
add_executable(viman_sa src/viman_sa.cpp)
target_link_libraries(viman_sa ${catkin_LIBRARIES} vm_pid viman)

add_subdirectory(src/slam_Z)
add_subdirectory(scripts/slam_Z)

############# Utility Nodes #############

#### Utility ROS node to observe sensor data (except camera) ####
add_executable(vm_sensor_data src/utility/vm_sensor_data.cpp)
target_link_libraries(vm_sensor_data ${catkin_LIBRARIES})

add_subdirectory(src/utility)

############# Testing Nodes #############

install(DIRECTORY include include/plugin include/utility plugins src src/plugin src/utility lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS 
	scripts/frnt_vision.py scripts/vision_process.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
