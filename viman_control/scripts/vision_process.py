#!/usr/bin/env python

import threading

import cv2
import numpy as np

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
		self.img = np.zeros((w,h,ch), np.uint8)
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
	
if __name__ == '__main__':
	print('Please run the node frnt_vision.py')

