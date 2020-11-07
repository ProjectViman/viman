

"""
Vishnu... thank you for Electronics.

Project name : VIMAN
@author: Manas kumar mishra
TaskNumber-D(2)  D--> Decimal number system.
"""

"""
Task :Apply the ORB algorithm with (FAST) to define the descriptor.
"""
"""
Theory:
    ORB:
	Oriented fast rotated BRIEF.
    What is a descriptor?
    It is a unique signature of the points in a image.
    
since video is the set of frame coming in a sec which implies we can use
this for video also. Let's see the algorithm.

Note:- here I am applying the blurring operation on the image.
"""


import numpy as np
import cv2
import time

cap = cv2.VideoCapture(0)

i=0
noofframe =600
start  = time.time()
orbdet = cv2.ORB_create()

while(i<noofframe):
	ret, frame = cap.read()
	
	frame = cv2.medianBlur(frame, 5)
	kp = orbdet.detect(frame, None)
	
	kp, des = orbdet.compute(frame, kp)
	newframe = cv2.drawKeypoints(frame, kp, None, color=(0, 255, 0))
	cv2.imwrite('orbframe'+str(i)+'.png', newframe)
	cv2.imshow('newframe',newframe)
	i =i+1
	
end = time.time()

sec = end-start
print('Time taken = ', sec)

cap.release()
