#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from process_vision import IdColor, Output

"""
Coverts ROS msg into OpenCV image matrix, calibrates the image, and then
stores the output into a class Output from where processing can access
the image matrix.
"""
class img_calibrator:
  
    # Define Camera Matrix
	mtx =  np.array([[421.375597, 0, 319.170565],
					 [0, 421.314788, 239.644714],
					 [0, 0, 1]])

	# Define distortion coefficients
	dist = np.array([-0.000233, -0.000328, 0.000116, -0.000326, 0.000000])
  
	def __init__(self, img_w, img_h):
		self.bridge = CvBridge()
		
		# subscribing to ROS topic
		self.image_sub = rospy.Subscriber("/viman/vm_frnt_cam/image",Image,self.callback)
		self.img_w = img_w
		self.img_h = img_h
		
		# obtaining the camera matrix
		self.newCamMtx, _ = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist,
									(self.img_w,self.img_h), 1,
									 (self.img_w,self.img_h))
	
	def callback(self,data):
		# converting ROS msg to CV img
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
			
		Output.lock.acquire()
		# calibrating the image
		Output.img = cv2.undistort(cv_image, self.mtx, self.dist,
									None, self.newCamMtx)
		try:
			Output.lock.notify()
		finally:
			Output.lock.release()

if __name__ == '__main__':
	ic = img_calibrator(640,480)
	th_processing = IdColor()
	rospy.init_node('frnt_vision')
	
	# Start vision processing in the background
	th_processing.daemon = True
	th_processing.start()
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	finally:
		th_processing.stop_process = True;
		th_processing.join()
		print("Shut down.")
