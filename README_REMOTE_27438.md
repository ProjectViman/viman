
## VIMAN - VIrtual Manual and Autonomous Navigation

This repository contains two ROS packages:
 1. [viman_control][1]: Contains plugins to simulate the quadcopter, sensors, and implementation of control algorithms.
 2. [viman_visualize][2]: Contains 3D models that build a Gazebo environment and the model of the quadcopter itself.

![UAV - VIMAN](https://github.com/AuntyVIEW/viman/blob/master/viman_visualize/multimedia/open_sky_1.jpg)

### For manual operation
Step 1: Execute the following command in terminal to launch the Gazebo world with *viman*.
```
roslaunch viman_visualize display-gazebo.launch on_rviz:=true
```
Step 2: Open another terminal and execute the following command to use keyboard keys to control the quadcopter.
```
rosrun viman_control viman_key_ctrl
```
Step 3: Play the simulation in Gazebo, place the focus of the terminal opened in step 2 and read the instructions provided by the ROS node.

### For semi-autonomous operation
Step 1: Execute the following command in terminal to launch the Gazebo world with *viman*.
```
roslaunch viman_visualize display-gazebo.launch on_rviz:=true
```
Step 2: Open another terminal and execute the following command to use keyboard keys to control the quadcopter.
```
rosrun viman_control viman_sa
```
Step 3: Play the simulation in Gazebo, place the focus of the terminal opened in step 2 and read the instructions provided by the ROS node.

### To see sensor data (IMU and altimeter)
```
rosrun viman_control vm_sensor_data
```

### roslaunch parameters
Note: Parameter values in **bold** correspond to default value.

1) `display-gazebo.launch`:
- `on_rviz`: **false**/true | To show or not to show RViz displaying the front camera output.
- `nav`: **0**/1 | Choose type of navigation:
  - 0 - Empty world
  - 1 - Additional stacked cylinders for only z linear navigation


[1]:https://github.com/AuntyVIEW/viman/tree/master/viman_control
[2]:https://github.com/AuntyVIEW/viman/tree/master/viman_visualize
