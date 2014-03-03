##Background eliminator
import cv2
import numpy as np
from matplotlib import pyplot as plt
def nothing(x):
	pass
def unit_vector(vector):
	return np.array(vector / np.linalg.norm(vector),np.float32)

def angle_between(v1, v2 ,origin):
	
	v1_u = unit_vector(v1-origin)
	v2_u = unit_vector(v2-origin)
	
	dot = np.dot(v1_u, v2_u)
	angle = None
	if(dot >=0 and dot <= 1):
		angle = np.arccos(dot)

	if angle == None:
		if (v1_u == v2_u).all():
			return 0.0
		else:
			return np.pi
	return angle
	
	
def discoverHorizontalLines(bkelim):
	#Generate the Gabor, isolate _horizontal_ white lines
	#This function expects the result of the background eliminator function
	gaborKernel = cv2.getGaborKernel((101,101),1,np.pi*93/180.0,13,15)
	horizontalLines = cv2.filter2D(bkelim[:,:,2], cv2.CV_32F, gaborKernel)
	return horizontalLines
	
	
def discoverSquares(image,canny1=64,canny2=31, discrim1 = 2.0, dicrim2 =3.0):
	#Finding objects based on their shape and not being stupid convex blobs

	#in this color plane:
	thresh_level = 2
	finalContours = []
	dilateKernel = np.ones((5,5),np.uint8)
	edges = None
	contours = None
	#Eventually: Scan through possibilities and append acceptable results
	for l in range(thresh_level):
		if l == 0:
			#edges = cv2.Canny(image, canny1, canny2, apertureSize = 3)
			#dilated = cv2.dilate(edges, dilateKernel, iterations = 1,anchor = (-1,-1))
			pass
		else:
			#edges = image >= ( (l+1)*255 )/thresh_level
			edges = image
		
	contours,hierarchy = cv2.findContours(np.array(edges,np.uint8), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
	
	#dispimg = np.array(np.zeros((image.shape[0],image.shape[1],3)),np.uint8)
	for ctr in contours:
		arcLen = cv2.arcLength(ctr,True)
		if arcLen > 0.1:
			approx = cv2.approxPolyDP(ctr,arcLen*(th4/500.0), True)	
	
			if len(approx)>=4:# and cv2.isContourConvex(approx): #&&np.fabs(cv2.contourArea(approx)) > 1000
				sumAngle = 0
				#cv2.drawContours(dispimg,approx,-1, (50,80,200))
				for j in range(2,len(approx)):
					angle = angle_between(approx[j%len(approx)][0], approx[j-2][0], approx[j-1][0])
					sumAngle += angle
				if sumAngle > 3 or sumAngle < 4:
					#cv2.drawContours(dispimg,[approx],-1,(255,0,100))
					finalContours.append(approx)
					
	return dispimg, finalContours
def eliminateBackground(image):
	##TODO: 
	# - Add a discriminator
	# - Improve detection under weird lighting conditions
	# - Improve reliability of isolation in all areas
	
	##Channels:
	#0: Blue Blocks
	#1: White Lines
	#2: Black Background (Decent - Don't rely on it)
	assert image != None, "No image passed"
	assert image.shape[2] == 3, "Need a 3-Channel RGB"
	
	HSVimage = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

	
	blue_lower=np.array([100,66,109],np.uint8)
	blue_upper=np.array([140,255,255],np.uint8)
	blue=cv2.inRange(HSVimage,blue_lower,blue_upper)
	#Isolation of low-high blue
	#Target element: Firing Blocks
	
	#This is the merge of the filtered blue and  g r layers to make a display image
	whitemax = np.array([180, 29, 255], np.uint8)
	white_lower = np.array([0, 0, 145], np.uint8)
	wht = cv2.inRange(HSVimage, white_lower, whitemax) - blue
	#This is the isolation of the white or very white objects.
	#Target element: Lines
	
	blackmax = np.array([180, 185,  185], np.uint8)
	black_lower = np.array([0, 0, 0], np.uint8)
	blk = cv2.inRange(HSVimage, black_lower, blackmax) - blue
	#Isolation of black
	#Target element: Background
	
	return cv2.merge((blue,wht,blk))
	
def makeOdd(x):
	if(x%2):
		return(x)
	else:
		return(x+1)
		
		
		
if __name__=='__main__':
	#image = cv2.imread('../Implementation/TwoBlueBlocks.png')
	#input = cv2.imread('../Implementation/myalgo2101974.jpg')
	#input = cv2.imread('../Implementation/myalgo2099447.jpg')
	input = cv2.imread('../Implementation/CourseMarch2-3.png')
	image = cv2.GaussianBlur(input, (7, 7), sigmaX = 1, sigmaY = 1)

	cv2.namedWindow('Frame')
	
	
	cv2.createTrackbar('Th1','Frame',200,255,nothing)
	cv2.createTrackbar('Th2','Frame',1,255,nothing)
	cv2.createTrackbar('Th3','Frame',200,255,nothing)
	cv2.createTrackbar('Th4', 'Frame', 12, 255, nothing)
	
	HSVimage = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)



	while(True):
		th1 = cv2.getTrackbarPos('Th1','Frame')
		th2 = cv2.getTrackbarPos('Th2','Frame')
		th3 = cv2.getTrackbarPos('Th3','Frame')
		th4 = cv2.getTrackbarPos('Th4','Frame')

		bkelim = eliminateBackground(image)
		blue = bkelim[:,:,0]
		wht = bkelim[:,:,1]
		black = bkelim[:,:,2]
		openKernel = np.ones((10,10),np.uint8)
		bfixed = cv2.morphologyEx(blue,cv2.MORPH_CLOSE, openKernel)
		
		
		#Shouldn't be trying to find blue squares - just squares
		
		squares = discoverSquares(blue, th1, th2)
		#cny = cv2.Canny(image, th3, th4, apertureSize=3)
		#squares = discoverSquares(cny)
		
		
		#cv2.imshow('original',image)
		cv2.imshow('Fixie', squares)
		#cv2.imshow('Canny', cny)
		cv2.imshow('Display', bfixed)
		cv2.imshow('Color',  cv2.merge((blue,wht,black)))
		keyPress = cv2.waitKey(1) & 0xFF
		if (keyPress):
			if(keyPress == ord('q')):

				break
	cv2.destroyAllWindows()
