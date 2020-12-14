# On SLAM algorithm repos
The aim of this file is to list down various SLAM algorithms that are available on GitHub under open source license. By listing, the intention is to select the algorithm that satisfies the following requirements:
1. Algo must use the following sensors:
	- monocular camera
	- IMU {acclerometer, gyroscope, magnetometer} *(not high end)*
	- altitude/depth sensor
2. Algo must run properly on system equivalent to Jetson Nano.
3. A provision must be present/created to visualize the map.
4. A provision must be present/created to use the map for path planning.
5. Enough resources about the algorithm must be present to understand the implementation completely. *A niche algo would be considered iff it's implementation is completely understood and it is tested on Nano (exceptional case).*
6. Higher preference to the algorithm which is completely implemented in C++ over the one implemented in Python. *Note: This does not mean we do not look out for algos implemented in Python.*

## Algo 1 | [SVO][1]
SVO stands for Semi-direct Visual Odometry. The repository is maintained by Robotics & Perception group of University of Zurich. The developers have actually implemented this on their runs and it works great! Check this [video][4] and going through the description is a must. The best thing about this package is that RPG of UZH have run the on on a [Ordroid-U3][5] processor. If it can run on that obsolete processor, then I am sure it will run on Jetson Nano smoothly and concurrently with the control subsystem. 

This algo satisfies requirements 1-2,5,6. For requirements 3-4, some additional work must be done.

**Status**: Test implementation

## References
1. [awesome-visual-slam][2]

[1]:https://github.com/uzh-rpg/rpg_svo 
[2]:https://github.com/tzutalin/awesome-visual-slam#projects
[3]:https://www.hardkernel.com/shop/odroid-u3/
[4]:https://www.youtube.com/watch?v=2YnIMfw6bJY&feature=youtu.be
[5]:https://www.hardkernel.com/shop/odroid-u3/