# VIMAN - VIrtual Manual and Autonomous Navigation

With Project VIMAN, we aim to build a quadcopter capable of maneuvering in autonomous mode with the ability to detect people having high body temperature in a crowded area. This capability could be used to churn out potential CoVID-19 infected people from a crowd.

In order to incorporate autonomy into our UAV, the following milestones are to be achieved:
- - [x] Control via keyboard (Stage 0)
- - [x] SLAM Z (Stage 1)
  - - [x] Hover at a set height (Stage 1.1)
  - - [x] Hover with a set heading (Stage 1.2)
  - - [x] Linear-Z mapping (Stage 1.3)
    - - [x] Camera calibration (1.3.0)
    - - [x] Color thresholding (1.3.1)
    - - [x] Color identification (1.3.2)
  - - [x] Rotation-Z mapping (Stage 1.4)
  - - [x] Z mapping (Stage 1.5)
- - [ ] SLAM X (Stage 2)
  - - [ ] Detect motion in X direction using camera (Stage 2.1)
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

![UAV - VIMAN](https://github.com/AuntyVIEW/viman/blob/master/viman_utility/multimedia/open_sky_1.jpg)

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
Step 1: Execute the following command in terminal to launch the Gazebo world with *viman*.
```
roslaunch viman_visualize gazebo-disp.launch nav:=1
```
Step 2: Open another terminal and then execute the following command
```
rosrun viman_control vm_z_linear_nav
```
Follow the instructions preseted by the ROS node.

#### 1 | Map the features observed by VIMAN and optimize the graph
Perform step 1 from 1.3 and then execute the following command in another terminal.
```
rosrun viman_control vm_z
```
Follow the instructions preseted by the ROS node.

#### Gazebo World
The following is an image of the stack of cylinders that the UAV must map. The shown stack is used to complete stage 1.3.<br>
![Linear-Z-World](https://github.com/AuntyVIEW/viman/blob/master/viman_utility/multimedia/linear_z_world.png)
<br>
The following GIF shows the partial cylinder stack (spawned via `nav:=2`. The environment shown is used for SLAM Z)<br>
![motion-z-key](https://github.com/AuntyVIEW/viman/blob/master/viman_utility/multimedia/motion_z_keyboard.gif)

## Utility
### To see sensor data (IMU and altimeter)
```
rosrun viman_control vm_sensor_data
```
### To view camera output (SLAM Z)
```
rosrun viman_control z_vision.py
```
### To view the color identified by the camera (SLAM Z)
```
rostopic echo /viman/color_id
```
### To visualize optimized and unoptimized maps (works only with `vm_z`)
```
rosrun viman_control visualize_map
```
Once the ROS node is run, it shows two plots:
1. Unoptimized map
2. Optimized map <br>
![unoptimized](https://github.com/AuntyVIEW/viman/blob/master/viman_utility/multimedia/unoptimized.png)
![optimized](https://github.com/AuntyVIEW/viman/blob/master/viman_utility/multimedia/optimized.png)


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
![Color-Thresholding](https://github.com/AuntyVIEW/viman/blob/master/viman_utility/multimedia/thresholding.png)

### roslaunch parameters
Note: Parameter values in **bold** correspond to default value.

1) `gazebo-disp.launch`:
- `on_rviz`: **false**/true | To show or not to show RViz displaying the front camera output.
- `nav`: **0**/1/2 | Choose type of navigation:
  - 0 - Empty world
  - 1 - Additional stacked cylinders for only z linear navigation
  - 2 - Parital cylinders stacked for SLAM Z navigation

## Other VIMAN repositories
1. [viman_vo][8] : Repo containing python code related to visual odometry.
2. [Viman-Dead_Reckoning][9] : Repo containing cpp code to obtain odometry via IMU.

## References
- [sjtu-drone][4] repository for simulation base.
- [This][5] answer from stack-overflow for `getch` equivalent in Ubuntu.
- [dbscan-cpp][6] repository for implementing DBSCAN algorithm to optimize map.
- [matplotlib-cpp][7] repository for `visualize_map` ROS node.
- Google search

[1]:https://github.com/AuntyVIEW/viman/tree/master/viman_control
[2]:https://github.com/AuntyVIEW/viman/tree/master/viman_visualize
[3]:https://github.com/AuntyVIEW/viman/tree/master/viman_utility
[4]:https://github.com/tahsinkose/sjtu-drone
[5]:https://stackoverflow.com/questions/7469139/what-is-the-equivalent-to-getch-getche-in-linux
[6]:https://github.com/foreseaz/dbscan-cpp
[7]:https://github.com/lava/matplotlib-cpp
[8]:https://github.com/AuntyVIEW/viman_vo
[9]:https://github.com/AuntyVIEW/Viman-Dead_Reckoning
