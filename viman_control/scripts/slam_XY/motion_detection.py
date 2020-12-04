"""
 Vishnu... Thank you for electronics.
 
 Author: Manas Kumar Mishra
 Task:- D(3) D--> Decimal number system.
"""

"""
 Task :Apply the optical flow algorithm with shi-tumasi to define the motion path and
		print that is there any change in the person or image is is motion or not.
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
feature_param = dict(maxCorners =500,
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


while True:
	ret, newframe = cap.read()
	framegray = cv2.cvtColor(newframe, cv2.COLOR_BGR2GRAY)
	
	if p0 is None:
		p0 = cv2.goodFeaturesToTrack(framegray, mask =None, **feature_param)
	# Now apply the optical flow inbuilt function of the opencv
	p1, st, err = cv2.calcOpticalFlowPyrLK(oldgray, framegray, p0, None, **lk_param)
	
	if p1 is not None and p0 is not None:
		try:
				goodnew = p1[st==1]
				goodold = p0[st==1]
				
				# Now consider the difference of the tracking point positions such that we can define the motion.
				diffpos = goodnew - goodold
				
				
				# Now compare the elements of the array to thnumber
				# 2.2 is a threshold value after that I will declear the motion
				comp1 = (diffpos < -1.2).astype(int)
				comp2 = (diffpos > 1.2 ).astype(int)
				comp = np.add(comp1,comp2)
				
				# compare all elements of the array to null array.
				is_all_zero = np.all((comp == 0))
				
				
				if is_all_zero:
					# print("No motion")
					cv2.putText(newframe, 'No motion',(50, 50), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 255), 2, cv2.LINE_AA)
					# print(is_all_zero)
				else:
					# print("Motion")
					cv2.putText(newframe, 'Motion',(50,50), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 0), 2, cv2.LINE_AA)
		except ValueError:
			pass
	
	
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
	# ino = ino+1
	
	if cv2.waitKey(1)&0xff == ord('q'):
		break

cap.release()

# Thank you
