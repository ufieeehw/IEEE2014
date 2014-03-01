##Background eliminator
import cv2
import numpy as np
from matplotlib import pyplot as plt
def nothing(x):
	pass
def eliminateBackground(image):
	##TODO: Add a discriminator
	#0: Blue Blocks
	#1: White Lines
	#2: Black Background (Decent - Don't rely on it)
	
	HSVimage = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	blue_lower=np.array([100,66,147],np.uint8)
	blue_upper=np.array([140,255,255],np.uint8)
	blue=cv2.inRange(HSVimage,blue_lower,blue_upper)
	#Isolation of low-high blue
	#Target element: Firing Blocks
	
	#This is the merge of the filtered blue and  g r layers to make a display image
	whitemax = np.array([180, 29, 255], np.uint8)
	white_lower = np.array([0, 0, 145], np.uint8)
	wht = cv2.inRange(HSVimage, white_lower, whitemax)
	#This is the isolation of the white or very white objects.
	#Target element: Lines
	
	blackmax = np.array([180, 185,  185], np.uint8)
	black_lower = np.array([0, 0, 0], np.uint8)
	blk = cv2.inRange(HSVimage, black_lower, blackmax)
	#Isolation of black
	#Target element: Background
	
	return cv2.merge((blue,wht,blk))
	
def makeOdd(x):
	if(x%2):
		return(x)
	else:
		return(x+1)
if __name__=='__main__':
	#image = cv2.imread('./Implementation/TwoBlueBlocks.png')
	#input = cv2.imread('./Implementation/myalgo2101974.jpg')
	input = cv2.imread('../Implementation/myalgo2099447.jpg')
	image = cv2.GaussianBlur(input, (7, 7), sigmaX = 1, sigmaY = 1)

	cv2.namedWindow('Frame')
	
	
	cv2.createTrackbar('Th1','Frame',200,255,nothing)
	cv2.createTrackbar('Th2','Frame',1,255,nothing)
	cv2.createTrackbar('Th3','Frame',1,255,nothing)
	cv2.createTrackbar('Size', 'Frame', 15, 100, nothing)
	#gray = cv2.cvtColor(image,  cv2.COLOR_BGR2GRAY)
	
	HSVimage = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


	
	#cv2.startWindowThread()
	while(True):
		th1 = cv2.getTrackbarPos('Th1','Frame')
		th2 = cv2.getTrackbarPos('Th2','Frame')
		th3 = cv2.getTrackbarPos('Th3','Frame')
		th4 = cv2.getTrackbarPos('Size','Frame')

		
		blue_lower=np.array([100,66,147],np.uint8)
		blue_upper=np.array([140,255,255],np.uint8)
		blue=cv2.inRange(HSVimage,blue_lower,blue_upper)
		#Isolation of low-high blue
		#Target element: Firing Blocks
		
		#This is the merge of the filtered blue and  g r layers to make a display image
		whitemax = np.array([180, 29, 255], np.uint8)
		white_lower = np.array([0, 0, 145], np.uint8)
		wht = cv2.inRange(HSVimage, white_lower, whitemax)
		#This is the isolation of the white or very white objects.
		#Target element: Lines
		
		blackmax = np.array([180, 185,  185], np.uint8)
		black_lower = np.array([0, 0, 0], np.uint8)
		blk = cv2.inRange(HSVimage, black_lower, blackmax)
		#Isolation of black
		#Target element: Background
		

	
		
		cny = cv2.Canny(blue, th2, th3, apertureSize=3)
		cv2.imshow('Canny', cny)
		cv2.imshow('Display', blue)
		#cv2.imshow('Color',  cv2.merge((blue,  wht,  blk)))
		keyPress = cv2.waitKey(1) & 0xFF
		if (keyPress):
			if(keyPress == ord('q')):

				break
	cv2.destroyAllWindows()
