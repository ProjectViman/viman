"""
Vishnu... Thank you for electronics.

Author :- Manas Kumar Mishra.
Task :- D(8) D--> Decimal number system.
"""
"""
Task :- find the relative  position vector through the images 
		using ORB+ essential matrics .
"""


import numpy as np
import cv2
import time

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

IdentityMat = np.array([[1,0,0,0],
					 [0,1,0,0],
					 [0,0,1,0],
					 [0,0,0,1]])



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
						table_number = 12, # 12
						key_size = 20,     # 20
						multi_probe_level = 2) #2

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
	# Mat = np.concatenate((R,t.T),axis=1)
	# nope = np.array([[0,0,0,1]])
	
	# instantMat = np.concatenate((Mat,nope), axis=0)
	
	# print(instantMat)
	
	print(t)
	i_no = i_no+1

end = time.time()

sec = end-start
print(sec)
cap.release()
