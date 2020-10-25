#!/usr/bin/env python

import threading

import numpy as np
import cv2

import rospy
from viman_utility.msg import CamZ

"""
Common class where calibrated CV image matrix is stored and accessed
"""
class Output:
	# dummy image
	img = np.zeros((640,480,3), np.uint8)
	
	lock = threading.Condition()
	
	def __init__(self):
		pass

class DisplayImg(threading.Thread):
	
	def __init__(self):
		threading.Thread.__init__(self)
		self.th_processing = FindThresholds()
		self.stop_process = False
	
	def run(self):
		self.th_processing.start()
		while not self.stop_process:
			Output.lock.acquire()
			try:
				Output.lock.wait(0.1)
				self.th_processing.img = Output.img
				cv2.imshow("Front camera vision", Output.img)
				cv2.waitKey(2)
			except:
				print('some issue')
			finally:
				Output.lock.release()
		self.th_processing.stop_process = True
		self.th_processing.join()
		cv2.destroyAllWindows()
		print('Processing ended..')

class FindThresholds(threading.Thread):
	
	def __init__(self, w=640, h=480, ch=3):
		threading.Thread.__init__(self)
		self.low_H = 0
		self.low_S = 0
		self.low_V = 0
		self.high_H = 180
		self.high_S = 255
		self.high_V = 255
		self.thresh_window_name = 'Thresholding Window'
		self.img = np.zeros((h,w,ch), np.uint8)
		self.stop_process = False
		
	def on_low_H_thresh_trackbar(self, val):
		self.low_H = val
		self.low_H = min(self.high_H-1, self.low_H)
		cv2.setTrackbarPos('Low H', self.thresh_window_name, self.low_H)

	def on_high_H_thresh_trackbar(self, val):
		self.high_H = val
		self.high_H = max(self.high_H, self.low_H+1)
		cv2.setTrackbarPos('High H', self.thresh_window_name, self.high_H)

	def on_low_S_thresh_trackbar(self, val):
		self.low_S = val
		self.low_S = min(self.high_S-1, self.low_S)
		cv2.setTrackbarPos('Low S', self.thresh_window_name, self.low_S)

	def on_high_S_thresh_trackbar(self, val):
		self.high_S = val
		self.high_S = max(self.high_S, self.low_S+1)
		cv2.setTrackbarPos('High S', self.thresh_window_name, self.high_S)

	def on_low_V_thresh_trackbar(self, val):
		self.low_V = val
		self.low_V = min(self.high_V-1, self.low_V)
		cv2.setTrackbarPos('Low V', self.thresh_window_name, self.low_V)

	def on_high_V_thresh_trackbar(self, val):
		self.high_V = val
		self.high_V = max(self.high_V, self.low_V+1)
		cv2.setTrackbarPos('High V', self.thresh_window_name, self.high_V)
	
	def create_window(self):
		cv2.namedWindow(self.thresh_window_name)
		
		cv2.createTrackbar('Low H', self.thresh_window_name, self.low_H, 180, self.on_low_H_thresh_trackbar)
		cv2.createTrackbar('Low S', self.thresh_window_name, self.low_S, 255, self.on_low_S_thresh_trackbar)
		cv2.createTrackbar('Low V', self.thresh_window_name, self.low_V, 255, self.on_low_V_thresh_trackbar)
		
		cv2.createTrackbar('High H', self.thresh_window_name, self.high_H, 180, self.on_high_H_thresh_trackbar)
		cv2.createTrackbar('High S', self.thresh_window_name, self.high_S, 255, self.on_high_S_thresh_trackbar)
		cv2.createTrackbar('High V', self.thresh_window_name, self.high_V, 255, self.on_high_V_thresh_trackbar)
	
	def run(self):
		self.create_window()
		while not self.stop_process:
			frame_hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
			frame_thresh = cv2.inRange(frame_hsv, (self.low_H, self.low_S, self.low_V), (self.high_H, self.high_S, self.high_V))
			
			cv2.imshow(self.thresh_window_name, frame_thresh)
		cv2.destroyAllWindows()
		print('Stopped finding thresholds...')

class IdColor(threading.Thread):
	"""
	Define color ranges for each color. Color sequence:
	green-red-purple-blue-yellow
	"""
	colors = ['green','red','purple','blue','yellow']
	lower_limits = [(41,45,0), (0,0,190), (144,0,0), (94,127,0), (26,0,86)]
	upper_limits = [(64,255,255), (7,255,255), (180,255,255), (124,255,255), (44,255,255)]
	
	def __init__(self, w=640, h=480, ch=3):
		threading.Thread.__init__(self)
		
		# create a publisher to send ID-ed color
		self.color_pub = rospy.Publisher('/viman/color_id', CamZ, queue_size=100)
		self.colorid = CamZ()
		
		# default values
		self.window_name = 'Thresholded Window'
		self.thresh_area = 140000.0
		self.img = np.zeros((h, w, ch), np.uint8)
		self.masks = np.zeros((h, w, len(self.colors)), np.uint8)
		self.stop_process = False
	
	def run(self):
		kernal = np.ones((5,5), "uint8")
		cv2.namedWindow(self.window_name)
		while not self.stop_process:
			Output.lock.acquire()
			max_area = 0
			color_idx = -1;
			self.colorid.name = ''
			self.colorid.area = 0
			try:
				Output.lock.wait(0.1)
				self.img = Output.img
			except:
				print('some issue')
			finally:
				Output.lock.release()
				
			# create masks for each color, dilating to eliminate noise
			frame_hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
			for count, _ in enumerate(self.colors):
				self.masks[:,:, count] = cv2.inRange(frame_hsv,
												self.lower_limits[count],
												self.upper_limits[count])
				self.masks[:,:, count] = cv2.dilate(self.masks[:,:,count],
													kernal)
			
			# find contours of each color
			for count, col_name in enumerate(self.colors):
				_, contours, _ = cv2.findContours(self.masks[:,:,count].copy(),
														  cv2.RETR_TREE,
														  cv2.CHAIN_APPROX_SIMPLE)
														  
				# get contour of maximum area of a color
				color_max = 0
				area = 0
				c = None;
				for _, contour in enumerate(contours):
					area = cv2.contourArea(contour)
					if (area > color_max):
						color_max = area
						c = contour
						
				# take the max area contour amongst the colors
				if(color_max > self.thresh_area and color_max > max_area):
					max_area = color_max
					color_idx = count
					
					# draw the bounding rectangle
					x, y, w, h = cv2.boundingRect(c)
					self.img = cv2.rectangle(self.img, (x,y), (x+w, y+h),
											 (0,0,0), 2)
					self.colorid.name = col_name
					self.colorid.area = area
				elif(color_idx == -1 and count == len(self.colors)-1):
					color_idx = -1
			self.color_pub.publish(self.colorid)
			cv2.imshow(self.window_name, self.img)
			cv2.waitKey(1)
		cv2.destroyAllWindows()
		print('Stopped IDing colors...')

if __name__ == '__main__':
	print('Please run the node frnt_vision.py')

