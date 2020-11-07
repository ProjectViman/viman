"""
Vishnu... Thank you for electronics.

Author :- Manas Kumar Mishra.
Task :- D(9) D--> Decimal number system.
"""
"""
Task :- find the relative Homogenous tranformation matrix position through the images 
		using ORB+ essential matrics .
"""
"""
Theory:
	In this task I am following following steps:
	1. Take a inital frame, change into gray, apply blurring operation, Find keypoints
	and descriptors.
	2. While loop for a set of frames (Note:- we can use infinite loop with breaking condition).
	3. Same step as first but for next/new frame.
	4. Use feature matcher for matching the two frame value. In short, find correspoinds between 
	the images.
	5. Store correspoinding points for further operations.
	6. Apply Fundamental matrix operation.
	7. Use camera matrix to convert the fundamental matrix to essential matrix.
	8. Calculate the relative pose  (Rotation and translation).
	9. find the relative homogeneous transform matrix.
"""


import numpy as np
import cv2
import time


def HtransformationMat(R,t):
	c = np.c_[R,t]
	
	# prespective vector and scaling value.
	scale =1
	eta = np.array([0,0,0,scale])
	
	htrans = np.vstack([c,eta])
	
	return htrans
	
# camera parameters (camera matrics)
# Change this matrix according to your camera.
K = np.array([1189.46, 0.0, 805.49, 0.0, 1191.78, 597.44, 0.0, 0.0, 1.0]).reshape(3,3)

# Capturing the video through the webcam
cap = cv2.VideoCapture(0)

# ORB OBJECT
orb = cv2.ORB_create()

#Capture inital frame.
ret0, oldframe = cap.read()
oldframe = cv2.medianBlur(oldframe, 5)

oldgray = cv2.cvtColor(oldframe, cv2.COLOR_BGR2GRAY)

# keypoints and descriptors
kp0, des0 = orb.detectAndCompute(oldgray, None)

# no of frame set for a video
noofframe=10

# IdentityMat = np.array([[1,0,0,0],
					 # [0,1,0,0],
					 # [0,0,1,0],
					 # [0,0,0,1]])



# inital time set
start = time.time()
i_no=0
while(i_no<noofframe):
	# read the video frame by frame
	ret, newframe = cap.read()
	newframe = cv2.medianBlur(newframe, 5)
	
	newgray = cv2.cvtColor(newframe, cv2.COLOR_BGR2GRAY)
	
	# keypoints and descriptors
	kp1,des1 = orb.detectAndCompute(newgray, None)
	
	showframe = cv2.drawKeypoints(newgray, kp1, None, color=(0, 255, 0))
	cv2.imwrite('orbframeforfundamental'+str(i_no)+'.png', showframe)
	
	# Features
	# Matcher
	FLANN_INDEX_LSH = 6
	index_params= dict(algorithm = FLANN_INDEX_LSH,
						table_number = 12, # 6
						key_size = 20,     # 10
						multi_probe_level = 2) #1 also possible

	search_params = dict(checks=50)   # or pass empty dictionary
	flann = cv2.FlannBasedMatcher(index_params,search_params)
	matches = flann.knnMatch(des0,des1,k=2)
	
	
	good = []
	pt0 = []
	pt1 = []
	
	for i,pair in enumerate(matches):
		try:
			m,n = pair
			if m.distance < 0.7*n.distance:
				good.append(m)
				pt1.append(kp1[m.trainIdx].pt)
				pt0.append(kp0[m.queryIdx].pt)
		except ValueError:
			pass
	
	pt0 = np.int32(pt0)
	pt1 = np.int32(pt1)
	
	F, mask1 = cv2.findFundamentalMat(pt0, 
									pt1, 
									method=cv2.FM_RANSAC ) #method= cv2.FM_LWEDS also a possiblity.
	
	E = K.T.dot(F).dot(K)
	p, R, t, mask = cv2.recoverPose(E, pt0, pt1, focal=1, pp=(0., 0.))
	tt = HtransformationMat(R,t)
	
	print(tt)
	print('\n')
	i_no = i_no+1

end = time.time()

sec = end-start
print(sec)
cap.release()

# Thank you.
