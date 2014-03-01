import cv2
import numpy as np
import time

def nothing(x):
	pass
def makeOdd(x):
	return(x + (x%2))

		
	
cv2.namedWindow('Frame',  cv2.WINDOW_NORMAL)
img = cv2.imread('./Implementation/BlueBlockOne.jpg')
cv2.createTrackbar('Th1','Frame',185,255,nothing)

cleanedImage = np.uint8( np.zeros(img.shape) )
cleanedImage = cv2.GaussianBlur(img, (21,21), sigmaX = 1, sigmaY = 1)

while(True):
	th1 = cv2.getTrackbarPos('Th1','Frame')
	finalImage = np.copy(img)
	
	flag,thresh = cv2.threshold(cleanedImage,th1,255,cv2.THRESH_BINARY)
	#I know that this is ridiculous.
	gray = cv2.cvtColor(thresh,cv2.COLOR_BGR2GRAY)
	cny = cv2.Canny(gray,600,200,apertureSize=5)
	cv2.imshow('cny',cny)
	contourArray, hierarchy = cv2.findContours(cny, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	clist = []	
	cv2.drawContours(finalImage,contourArray,-1,(0, 255, 0),thickness=-1)
	#cv2.imshow('Gray', gray)
	#cv2.imshow('fin',finalImage)
	dispimg = cv2.pyrUp(cv2.pyrUp(finalImage))
	maskSum = np.zeros(gray.shape, np.uint8)
	for ctr in contourArray:
	
		mask = np.zeros(gray.shape,np.uint8)
		np.add(maskSum,  mask,  maskSum)
		cv2.drawContours(mask,ctr,0,255,-1)
		mean = cv2.mean(img,mask=mask)
		M = cv2.moments(ctr)
		cx = 0
		cy = 0
		try:
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
		except:
			pass
		#print (cx, cy)
		
		
		cv2.putText(dispimg, str(mean),(cx*4, cy*4)	, cv2.FONT_HERSHEY_PLAIN, 0.8, (0,0,255), thickness=1)
		
		
#		if(cv2.contourArea(ctr) > 100):
#			mask = np.zeros(gray.shape,np.uint8)
#			cv2.drawContours(mask,ctr,0,255,-1)
#			mean = cv2.mean(img,mask=mask)
#			
#			RGBThreshUpper = (255,255,230)
#			RGBThreshLower = (0,0,180)
#			
#			meanTrue1 = map(lambda A,B: A > B, RGBThreshUpper, mean)
#			meanTrue2 = map(lambda A,B: A < B, RGBThreshLower, mean)
#			
#			
#			if(sum(meanTrue1) + sum(meanTrue2) == 6):
#				clist.append(ctr)
#			

	#mskd = np.zeros(finalImage.shape)
	#cv2.drawContours(mskd,clist,-1,(255),-1)
	
	dispimg
	#cv2.imshow('And', maskSum)
	cv2.imshow('Frame',dispimg)
	
	keyPress = cv2.waitKey(1) & 0xFF
	if (keyPress):
		if(keyPress == ord('q')):
			exit()
		elif keyPress == ord('v'):
			print(contourArray)
	time.sleep(0.01)
