# VIMAN - VIrtual Manual and Autonomous Navigation

With Project VIMAN, we aim to build a quadcopter capable of maneuvering in autonomous mode with the ability to detect people having high body temperature in a crowded area. This capability could be used to churn out potential CoVID-19 infected people from a crowd.

In order to incorporate autonomy into our UAV, the following milestones are to be achieved:
- - [x] Control via keyboard (Stage 0)
- - [ ] SLAM Z (Stage 1)
  - - [x] Hover at a set height (Stage 1.1)
  - - [x] Hover with a set heading (Stage 1.2)
  - - [ ] Linear-Z mapping (Stage 1.3)
    - - [x] Camera calibration (1.3.0)
    - - [x] Color thresholding (1.3.1)
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

> Note: Make sure you have ROS' `cv_bridge` because OpenCV with Python3 has been used for vision processing.

![UAV - VIMAN](https://github.com/AuntyVIEW/viman/blob/master/viman_visualize/multimedia/open_sky_1.jpg)

## Implementation

### 0 | Control via keyboard
Step 1: Execute the following command in terminal to launch the Gazebo world with *viman*.
```
roslaunch viman_visualize display-gazebo.launch on_rviz:=true
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
roslaunch viman_visualize display-gazebo.launch on_rviz:=true
```
Step 2: Open another terminal and execute the following command to use keyboard keys to control the quadcopter.
```
rosrun viman_control viman_sa
```
Step 3: Play the simulation in Gazebo, place the focus of the terminal opened in step 2 and read the instructions provided by the ROS node.

## Utility
### To see sensor data (IMU and altimeter)
```
rosrun viman_control vm_sensor_data
```

### To find HSV range for color thresholding
Make use the following `DisplayImg` class in `vision_process.py`
```
class DisplayImg(threading.Thread):
	
	def __init__(self):
		threading.Thread.__init__(self)
		self.th_findThresh = FindThresholds()
		self.stop_process = False
	
	def run(self):
		self.th_findThresh.start()
		while not self.stop_process:
			Output.lock.acquire()
			try:
				Output.lock.wait(0.1)
				self.th_findThresh.img = Output.img
				cv2.imshow("Front camera vision", Output.img)
				cv2.waitKey(1)
			except:
				print('some issue')
			finally:
				Output.lock.release()
		self.th_findThresh.stop_process = True
		self.th_findThresh.join()
		cv2.destroyAllWindows()
		print('Processing ended..')
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
