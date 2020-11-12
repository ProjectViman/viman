# VIMAN - VIrtual Manual and Autonomous Navigation

With Project VIMAN, we aim to build a quadcopter capable of maneuvering in autonomous mode with the ability to detect people having high body temperature in a crowded area. This capability could be used to churn out potential CoVID-19 infected people from a crowd.

In order to incorporate autonomy into our UAV, the following milestones are to be achieved:
- - [x] Control via keyboard (Stage 0)
- - [ ] SLAM Z (Stage 1)
  - - [x] Hover at a set height (Stage 1.1)
  - - [x] Hover with a set heading (Stage 1.2)
  - - [x] Linear-Z mapping (Stage 1.3)
    - - [x] Camera calibration (1.3.0)
    - - [x] Color thresholding (1.3.1)
    - - [x] Color identification (1.3.2)
  - - [ ] Rotation-Z mapping (Stage 1.4)
  - - [ ] Z mapping (Stage 1.5)
- - [ ] SLAM X (Stage 2)
- - [ ] SLAM Y (Stage 3)
- - [ ] Complete SLAM (Stage 4)

> Note: Addition/deletion of sub-stages is dependent on the complexity of implementation of a stage.

### List of sensors used
1. Altimeter
2. IMU
3. Camera

## About the repository
This repository contains two ROS packages:
 1. [viman_control][1]: Contains plugins to simulate the quadcopter, sensors, and ROS nodes to test and process vision and implement control algorithms.
 2. [viman_visualize][2]: Contains 3D models that build a Gazebo environment and the model of the quadcopter itself.
 3. [viman_utility][3]: Contains utility nodes and custom messages to ease the development and control of VIMAN.

> Note: Make sure you have ROS' `cv_bridge` because OpenCV 3.2 with Python2.7 has been used for vision processing.

![UAV - VIMAN](https://github.com/AuntyVIEW/viman/blob/master/viman_visualize/multimedia/open_sky_1.jpg)

## Implementation

### 0 | Control via keyboard
Step 1: Execute the following command in terminal to launch the Gazebo world with *viman*.
```
roslaunch viman_visualize gazebo-disp.launch on_rviz:=true
```
Step 2: Open another terminal and execute the following command to use keyboard keys to control the quadcopter.
```
rosrun viman_control viman_key_ctrl
```
Step 3: Play the simulation in Gazebo, place the focus of the terminal opened in step 2 and read the instructions provided by the ROS node.

---
### 1 | SLAM Z
#### 1.1 & 1.2 | Hover at a height with a set heading 
Step 1: Execute the following command in terminal to launch the Gazebo world with *viman*.
```
roslaunch viman_visualize gazebo-disp.launch on_rviz:=true
```
Step 2: Open another terminal and execute the following command to use keyboard keys to control the quadcopter.
```
rosrun viman_control viman_sa
```
Step 3: Play the simulation in Gazebo, place the focus of the terminal opened in step 2 and read the instructions provided by the ROS node.

#### 1.3 | Map the features observed by VIMAN and store it into a file
Perform steps 1, and 2 from the above and then execute the following command in another terminal.
```
rosrun viman_control vm_z_linear_nav
```
Follow the instructions preseted by the ROS node.

#### Gazebo World
The following is an image of the stack of cylinders that the UAV must map. The shown stack is used to complete stage 1.3.
![Linear-Z-World](https://github.com/AuntyVIEW/viman/blob/master/viman_visualize/multimedia/linear_z_world.png)

## Utility
### To see sensor data (IMU and altimeter)
```
rosrun viman_control vm_sensor_data
```
### To view camera output (SLAM Z)
```
rosrun viman_control frnt_vision.py
```
### To view the color identified by the camera (SLAM Z)
```
rostopic echo /viman/color_id
```

### To find HSV color ranges for color thresholding
Make use the following changes in `frnt_vision.py`
```
...
from vision_process import DisplayImg, Output # For HSV thresholding
...
...
...
th_processing = DisplayImg() # For HSV thresholding
...
...
```
![Color-Thresholding](https://github.com/AuntyVIEW/viman/blob/master/viman_visualize/multimedia/thresholding.png)

### roslaunch parameters
Note: Parameter values in **bold** correspond to default value.

1) `display-gazebo.launch`:
- `on_rviz`: **false**/true | To show or not to show RViz displaying the front camera output.
- `nav`: **0**/1 | Choose type of navigation:
  - 0 - Empty world
  - 1 - Additional stacked cylinders for only z linear navigation


[1]:https://github.com/AuntyVIEW/viman/tree/master/viman_control
[2]:https://github.com/AuntyVIEW/viman/tree/master/viman_visualize
[3]:https://github.com/AuntyVIEW/viman/tree/master/viman_utility
