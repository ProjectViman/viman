"""
 Vishnu... Thank you for electronics.
 
 Author: Manas Kumar Mishra
 Task:- D(3) D--> Decimal number system.
"""

"""
 Task :Apply the optical flow algorithm with shi-tumasi to define the motion path.
"""

"""

Theory:- 
	Optical flow technique is for the motion tracking through the pixcel level analysis.
	Basically it will generate the pattern of apparent motion of the objects in the image 
	by analysing the two consecutive frames. 
	There are few assumption  which I am making:
	1. There is  not lighting  intensity change.
	2. There is no shadow of the object otherwise it will consider that shadow as an
		another object.
	3. No Rotional motion on the object otherwise it can't detect that motion. Like a sphere revolving around
		it own axis.
	
	In cv2 we have well define function/method for optical flow or KL tracking. Where it is using the
	shi-tomasi corner detection and pyramid techniques for tracing the motion.
"""
import numpy as np
import cv2
 
cap = cv2.VideoCapture(0)
 
# For shi-tumasi corner detection
feature_param = dict(maxCorners =200,
					 qualityLevel=0.3,
					 minDistance = 7,
					 blockSize =7)

# KL parameters definations
lk_param = dict(winSize=(15,15),
				maxLevel = 2,
				criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
				

# color = np.random.randint(0, 255, (100,3))

# Take first image as initial frame

ret, oldframe = cap.read()
oldgray = cv2.cvtColor(oldframe,cv2.COLOR_BGR2GRAY)
p0 = cv2.goodFeaturesToTrack(oldgray, mask = None, **feature_param)

mask =np.zeros_like(oldframe)

while(1):
	ret, newframe = cap.read()
	framegray = cv2.cvtColor(newframe, cv2.COLOR_BGR2GRAY)
	
	# Now apply the optical flow inbuilt function of the opencv
	p1, st, err = cv2.calcOpticalFlowPyrLK(oldgray, framegray, p0, None, **lk_param)
	
	goodnew = p1[st==1]
	goodold = p0[st==1]
	
	for i,(new, old) in enumerate(zip(goodnew, goodold)):
		a,b = new.ravel()
		c,d = new.ravel()
		mask = cv2.line(mask, (a,b), (c,d),(0,255,0), 2)
		# newframe = cv2.circle(newframe, (a,b), 5, (0,255,0), -1)
		newframe = cv2.arrowedLine(newframe, (a,b), (c,d), (255,255,0), 10, cv2.LINE_AA, 0, 2)
	
	img = cv2.add(newframe, mask)
	
	cv2.imshow('newframe', img)
	oldgray = framegray.copy()
	p0 = goodnew.reshape(-1,1,2)
	
	if cv2.waitKey(1)&0xff == ord('q'):
		break
	


cap.release()

# Thank you
