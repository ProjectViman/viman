#### ROS node for semi-autonomous operation in linear Z direction ####
add_executable(vm_z_linear_nav vm_z_linear_nav.cpp)
target_link_libraries(vm_z_linear_nav ${catkin_LIBRARIES} vm_pid viman map)

#### ROS node for semi-autonomous operation in Z direction ####
add_executable(vm_z vm_z.cpp)
target_link_libraries(vm_z ${catkin_LIBRARIES} vm_pid viman map)