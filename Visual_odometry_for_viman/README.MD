# Visual-Odometry-for-Viman
This is the repository containing the Computer vision algorithm using OpenCV in python for Viman application.

1. What the repo does?

*Finally this project will give the relative orientation of the two images. In this project, we are using single monocular camera
which provides the two images at two different instants of time. This two images, we are using to detect the relative orientation of the
camera. Along that there is a python code for optical flow on video fed. In this project we are testing with limited number of frames, but 
we can extent to the infinite loop video capturing also.

2. Why this repo is useful?

*Normally, stereo camera is very useful for computer vision applications. But if we reduce the distance between the two camera of stereo
arrangement, we would not get proper result through stereo. Alternative of that arrangement is monocular camera. This code is very usefull
for mano camera operation as well as stereo (obviously for good distance between two cameras). In our project, we are using this for estimating 
the position in x and y directions such that drone can move x and y direction also.

four codes
*opticalflow_test_1.py --> optical flow algo on video fed.

*orbdet_test_1.py      --> ORB implementation of keypoints and descriptors.

*position_test_2.py    --> Provide the relative orientations (translation) for each frame.

*position_test_3.py    --> Provide the final relative Homogeneous transformation Matrix for each frame.


