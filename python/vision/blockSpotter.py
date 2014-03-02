import cv2
import numpy as np

try:
	from IEEE2014Functions import backgroundEliminator as be
except:
	print "Background Elimination Import Failed"
def nothing(x):
	pass


#Notable parameters
cameraHeight = 10.0 #inches
cameraAzimuth = 0.0 #Rads Relative to robot
cameraAltitude = 0.0 #Rads relative to robot
focal = 2.54 #in/in
##The math was done at 640x480

#Discrimination factor - minimum acceptable area for the block contour
minimumArea = 200


robotPosition = (0.0,0.0) #inches

cv2.namedWindow('Frame')
cv2.namedWindow('Display', cv2.WINDOW_NORMAL)
cv2.createTrackbar('Th1','Frame',25,50,nothing)
cv2.createTrackbar('Th2','Frame',500,3000,nothing)
cv2.createTrackbar('Th3','Frame',200,3000,nothing)

#cap = cv2.VideoCapture(0)
#ret, frame = cap.read()
#cropped = frame[frame.shape[0]*0.45:frame.shape[0],:]
#cleanedImage = cv2.GaussianBlur(cropped, (3,3), sigmaX = 1, sigmaY = 1)
##For video

	
img = cv2.imread('./Implementation/myalgo2099447.jpg')
cropped = img[img.shape[0]*0.45:img.shape[0],:]
cleanedImage = cv2.GaussianBlur(cropped, (3,3), sigmaX = 1, sigmaY = 1)





while(True):
	th1 = cv2.getTrackbarPos('Th1','Frame')
	th2 = cv2.getTrackbarPos('Th2','Frame')
	th3 = cv2.getTrackbarPos('Th3','Frame')


	bkelim = be.eliminateBackground(cleanedImage)

	# Generate the Gabor, isolate horizontal white lines
	#gaborKernel = cv2.getGaborKernel((101,101),1,np.pi*93/180.0,13,15)
	#horizontalLines = cv2.filter2D(bkelim[:,:,2], cv2.CV_32F, gaborKernel)
	#cv2.imshow('Horizontal',horizontalLines)
	
	blocks = np.array(bkelim[:,:,0],np.uint8)
	contours, hierarchy = cv2.findContours(blocks,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	F = np.zeros_like(cleanedImage)
	centerofMass = [];
	for ctr in contours:
		M = cv2.moments(ctr)
		cx = 0
		cy = 0
		#This is the poor man's way of saying "If m00!=0:"
		
		#print area
		if M['m00']!= 0:
			
			area = cv2.contourArea(ctr)
			#Minimum Area!
			if(area > minimumArea):
			
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])
				centerofMass.append((cx,cy))
				cv2.drawContours(F,[ctr], -1,  (255,255,0))
				cv2.circle(F,(cx,cy), int(area/100), (0,0,255), thickness=-1)
				
				#Need to control for 'rectangleness' - OR: remove non-course background

#Coordinate Axis ASCII art
#< is the camera
#    Y (Height)
# X- |
#  \ |
#<__\|__________Z+(Dist)
#    \
#    |\
#    | \


#(0,0)
#__________________(img.shape[1], 0)
#|
#|
#|
#|
#|                
#|
#|
#|						.<- (img.shape[1],img.shape[0])
#(0,img.shape[0])

	dispimg = cv2.pyrUp(cv2.pyrUp(cleanedImage))
	cv2.circle(cleanedImage, (cleanedImage.shape[1]*2,cleanedImage.shape[0]*2), 10, (255,0,255),thickness = -1)
	for com in centerofMass:
		cx = com[0]
		cy = com[1]
		
		#Center of image is the origin
		xim = ( cx - (cleanedImage.shape[1]/2.0) )/(cleanedImage.shape[1]/2.0)
		yim = ( (cleanedImage.shape[0] - cy) - (cleanedImage.shape[0]/2.0) )/(cleanedImage.shape[0]/2.0)
		
		
		#print xim, yim
		distanceHoriz = -cameraHeight*(focal/yim)
		relativeX = (distanceHoriz*xim)/focal
		#cv2.putText(dispimg, str(xim)+ ', ' +str(yim), (cx*4, cy*4), cv2.FONT_HERSHEY_PLAIN, 0.8, (0,0,255), thickness=1)
		cv2.putText(dispimg, str(distanceHoriz)+ ', ' +str(relativeX), (cx*4, cy*4), cv2.FONT_HERSHEY_PLAIN, 0.8, (0,0,255), thickness=1)
	cv2.imshow('Display',dispimg)

	#cv2.imshow('Show',bkelim)
	#cv2.imshow('F',F)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
cv2.destroyAllWindows()
