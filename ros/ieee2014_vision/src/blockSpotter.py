import cv2
import numpy as np
import os
try:
	from IEEE2014Functions import backgroundEliminator as be
except:
	print "Background Elimination Import Failed in blockSpotter"
def nothing(x):
	pass

def spotBlocks(img=None):
	#Dear everyone:
	#Everything is in inches because I couldn't find a metric ruler.
	if img == None:
		path =  os.path.dirname(os.path.abspath(__file__))
		img = cv2.imread(path + '/Implementation/IEEEcourseMarch2-2.png')
		cv2.imshow('image',img)
	#Notable parameters
	cameraHeight = 8.5 #inches
	focal = 2.54 #in/in
	
	#Camera specs claim 5mm or 28mm
	## ~ 0.1968 in, 1.024 in
	##The math was done at 640x360 -- Will a higher res give better results in square discovery?

	#Discrimination factor - minimum acceptable area for the block contour
	minimumArea = 100
	#Discrimination by area is not reliable on the far lower bound
	bkelim = be.eliminateBackground(img)



	blocks = np.array(bkelim[:,:,0],np.uint8)
	contours, hierarchy = cv2.findContours(blocks,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)	
	if len(contours) == 0:
		return None, None

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
				#print cv2.isContourConvex(ctr)
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])

				#cv2.drawContours(F,[ctr], -1,  (220,150,0))
				#cv2.circle(F,(cx,cy), int(area/100), (0,0,255), thickness=-1)
				centerofMass.append((cx,cy))
				#Need to control for 'rectangleness' - OR: remove non-course background

	#Coordinate Axis ASCII art
	#< is the camera
	#    Y (Height)
	# X+ |
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
	

	#           +imY	
	#           |
	#           |
	#           |
	#           |
	#+imx_______.(0,0)_______
	#           |
	#           |
	#           |
	#           |
	#           |

	#dispimg = cv2.pyrUp(cv2.pyrUp(cleanedImage))
	#cv2.circle(dispimg, (cleanedImage.shape[1]*2,cleanedImage.shape[0]*2), 10, (255,0,100),thickness = -1)
	center = np.array([img.shape[1]/2, img.shape[0]/2 ]) #(Xh, Yh)
	#dispcenter = (cleanedImage.shape[1]/2, cleanedImage.shape[0]/2)
	
	positions = []
	for com in centerofMass:
		cxy = np.array([com[0],com[1]],np.float32)
		imxy = (center - cxy)/center
		xim = imxy[0]
		yim = imxy[1]
		
		#The numbers are correction factors because my focal length is off in x and y
		distanceHoriz = -cameraHeight*(focal/yim)/1.13 #Forward +, NO REAR LOL ITS A CAMERA
		relativeX = ((distanceHoriz*xim)/focal)*1.7 #Left +; Right -
		#cv2.putText(dispimg, str(cx)+ ', ' +str(cy), (cx*4 + 10, cy*4 + 10), cv2.FONT_HERSHEY_PLAIN, 0.8, (180,20,180), thickness=1)
		#cv2.putText(dispimg, str(imxy[0])+ ', ' +str(imxy[1]), (cx*4, cy*4), cv2.FONT_HERSHEY_PLAIN, 0.8, (0,0,255), thickness=1)
		#cv2.putText(dispimg, str(distanceHoriz)+ ', ' +str(relativeX), (cx*4, cy*4), cv2.FONT_HERSHEY_PLAIN, 0.8, (0,0,255), thickness=1)
		positions.append((distanceHoriz*0.0254,relativeX*0.0254))
		#meters
	return positions, centerofMass
	
	
	
	
if __name__ == '__main__':
	cv2.namedWindow('Frame')
	cv2.namedWindow('Display', cv2.WINDOW_NORMAL)
	cv2.createTrackbar('Th1','Frame',25,50,nothing)
	cv2.createTrackbar('Th2','Frame',500,3000,nothing)
	cv2.createTrackbar('Th3','Frame',200,3000,nothing)

	##For Video
	#cap = cv2.VideoCapture(0)
	#ret, frame = cap.read()
	#cropped = frame[frame.shape[0]*0.45:frame.shape[0],:]
	#cleanedImage = cv2.GaussianBlur(cropped, (3,3), sigmaX = 1, sigmaY = 1)

	##Test images
	#img = cv2.imread('./Implementation/myalgo2099447.jpg')
	#img = cv2.imread('./Implementation/CoursePracticeMarch2.png')
	img = cv2.imread('./Implementation/IEEEcourseMarch2-2.png')
	#img = cv2.imread('./Implementation/CourseMarch2-3.png')

	#cropped = img[img.shape[0]*0.45:img.shape[0],:]
	cropped = img
	cleanedImage = cv2.GaussianBlur(cropped, (3,3), sigmaX = 1, sigmaY = 1)

	while(True):
		th1 = cv2.getTrackbarPos('Th1','Frame')
		th2 = cv2.getTrackbarPos('Th2','Frame')
		th3 = cv2.getTrackbarPos('Th3','Frame')

		
		positions,com = spotBlocks(cleanedImage)
		print positions
		for k in range(len(positions)):
			cm = com[k]
			pos = positions[k]
			cv2.circle(cleanedImage,cm, 5, (100,200,80),-1)
			cv2.putText(cleanedImage, str(pos[0])+ ', ' +str(pos[1]), (cm), cv2.FONT_HERSHEY_PLAIN, 0.8, (0,0,255), thickness=1)
		cv2.imshow('Display',cleanedImage)
	
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
	cv2.destroyAllWindows()
