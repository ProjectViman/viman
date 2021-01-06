# VIMAN - VIrtual Manual and Autonomous Navigation

## TODO {deadline: Dec 20, 2020} - (Completed)
- [x] Resolve the minor bugs in Gazebo simulation occurring due to introduction of helipad.
- [x] Add a downward looking camera to VIMAN.<br> **Reason**: SVO works only with downward looking cameras.
- [x] Clean the _ver2_ branch of _viman_ after making the aforementioned changes.
- [x] Add proper instructions to README file. 
> All the instructions would be later shifted to WIki. Since we are in the development phase, let's mention everything in _.md_ files only.
> To move VIMAN in _ver2_, please use only `viman_key_ctrl` node until TODO task 4 is finished.

---

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
- - [ ] Understanding and using an existing SLAM algorithm (Stage 2)
  - - [x] Make a new environment (Stage 2.1)
  - - [ ] Explore various existing algorithms (Stage 2.2)
  - - [ ] Build/find an Android app to view Android camera output on ROS (Stage 2.3)


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

> **IMPORTANT NOTE**: The `README.md` file of `master` branch gives an indepth explanation about the SLAM Z ROS nodes. Please refer to it in case of any descripancies until the Wiki page for the repository is ready.

## Implementation

### 0 | Control via keyboard
Step 1: Execute the following command in terminal to launch the Gazebo world with *viman*.
```
roslaunch viman_visualize gazebo-disp.launch
```
Step 2: Open another terminal and execute the following command to use keyboard keys to control the quadcopter.
```
rosrun viman_control viman_key_ctrl
```
Step 3: Play the simulation in Gazebo, place the focus of the terminal opened in step 2 and read the instructions provided by the ROS node.

---
### 1 | SLAM Z
Execute the following command in terminal to launch the Gazebo world with *viman*.
```
roslaunch viman_visualize gazebo-disp.launch
```
To use keyboard keys to control the quadcopter (surge, sway, heave, yaw)
> **NOTE**: Viman can be now controlled semi-autonomuosly. It could reach a certain user defined heigth irrespective of takeoff platforms.
```
rosrun viman_control viman_sa
```
To map the features observed by VIMAN and store it into a file (use `nav:=1` with `gazebo-disp.launch`)
```
rosrun viman_control vm_z_linear_nav
```
To map the features observed by VIMAN and optimize the graph (use `nav:=2` with `gazebo-disp.launch`)
```
rosrun viman_control vm_z
```
Follow the instructions preseted by the ROS node.

---
### 2 | Exploring existing SLAM algorithms
### The new evironment:
![env2_1](https://github.com/AuntyVIEW/viman/blob/ver2/viman_utility/multimedia/env2_1.jpg)
![env2_2](https://github.com/AuntyVIEW/viman/blob/ver2/viman_utility/multimedia/env2_2.jpg)
![env2_3](https://github.com/AuntyVIEW/viman/blob/ver2/viman_utility/multimedia/env2_3.jpg)

## Utility
### To see sensor data (IMU and altimeter)
```
rosrun viman_control vm_sensor_data
```
### To view camera output (SLAM Z)
There are two cams in viman: Front cam and Down Cam.
```
rosrun viman_control z_vision.py
```
- For Front Cam | Enter 1 to ur choice
- For Down Cam | Enter 0 to ur choice
### To view the color identified by the camera (SLAM Z)
```
rostopic echo /viman/color_id
```
### To visualize optimized and unoptimized maps (works only with `vm_z`)
```
rosrun viman_control visualize_map
```
### roslaunch parameters
Note: Parameter values in **bold** correspond to default value.

1) `gazebo-disp.launch`:
- `on_rviz`: **false**/true | To show or not to show RViz displaying the front camera output.
- `nav`: **0**/1/2 | Choose type of navigation:
  - 0 - Empty world
  - 1 - Additional stacked cylinders for only z linear navigation
  - 2 - Parital cylinders stacked for SLAM Z navigation
  - 3 - Environment 2

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
