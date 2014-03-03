import cv2
import numpy as np
import math
import sys

#GARBAGE DON'T USE
##TODO:
# - Add Line Intersection Generation
# - Add Polygon checking
# - - Is it a square after reasonable perspective transformation?
# - Resolve Bad Intersections
# - Remove waste ops like float/tuple conversion
## -> If execution time on-bot is a concern, port to C++ w/ python.boost

#Using Forrest Style Comments:

sys.path.append("./IEEE2014Functions/")
try:
	import jIEEE2014 as ie
except:
	print "Jacob IEEE Library import failed"

try:
	import backgroundEliminator as be
except:
	print "Background Elimination Import Failed"
	




def nothing(x):
	pass
def makeOdd(x):
	return(x + (x%2))

def detectRectangles(bwimg, title='rectangleDetection'):
	bwimg = np.uint8(bwimg)
	
	lines = cv2.HoughLinesP(bwimg, 1, math.pi/180, 15, maxLineGap = 6,  minLineLength=5)#, 100, 10)
	#mergeImg = np.uint8(np.zeros((bwimg.shape[0],bwimg.shape[1],3)))
	zeroMat = np.zeros_like(bwimg)
	mergeImg = cv2.merge((zeroMat, zeroMat,bwimg))
	if(lines != None):
		for lineNum in range(lines.shape[1]):
			line = lines[:,lineNum][0]
			#draw line
			cv2.line(mergeImg,(line[0],line[1]),(line[2],line[3]), (255, 0, 0),  thickness=1)#, thickness = 2)
	
		#Do Intersect Computations:

		if len(lines[0]) < 100:
			
			lineObjList = map(ie.lineObj,lines[0])
			for line in lineObjList:
				for line2 in lineObjList:
					
					intersectPt = line2.segIntersect(line, 20)
		
					if(intersectPt != None):
						cv2.circle(mergeImg,tuple(map(int,intersectPt)), 5, (255,0,0))
						line.groupAdd(line2)
			#Test:
			polyCandidates = []
			for line in lineObjList:
				if(len(line.group) > 1):
					#polyCandidates.append(line.group)
					for ls in line.group:
						#cv2.line(mergeImg,tuple(map(int,ls.start)),tuple(map(int,ls.end)), (0,0,255), thickness = 5 )
						try:
							#cv2.line(mergeImg,ls.intersections[0],ls.intersections[1], (0,255,0), thickness = 2)
							polyCandidates.append(line.group)
						except:
							#Not a candidate!
							pass
			for group in polyCandidates:
				inters = []
				for line in group:
				
					if len(line.intersections) < 2:
						pass
						#If the line only intersects in one place, it's a fringe
					else:
						inters.extend(line.intersections)
				if len(inters) > 2:	
					#print inters
					#cv2.fillConvexPoly(mergeImg, np.array(inters), (255, 255, 0))
					pass
	cv2.imshow(title, mergeImg)
	return 

		

 

cv2.namedWindow('Frame')

cv2.createTrackbar('Th1','Frame',25,50,nothing)
cv2.createTrackbar('Th2','Frame',500,3000,nothing)
cv2.createTrackbar('Th3','Frame',200,3000,nothing)

cv2.createTrackbar('Sigma','Frame',1, 250, nothing)
cv2.createTrackbar('Theta','Frame',1, 180, nothing)
cv2.createTrackbar('LM', 'Frame', 1, 150, nothing)


##TODO rely on actual image color information to adapt~
#img = cv2.imread('../Implementation/BlueBlockOne.jpg')
#img = cv2.imread('../Implementation/TwoBlueBlocks.png')
#img = cv2.imread('../Implementation/myalgo2101974.jpg')
img = cv2.imread('../Implementation/myalgo2099447.jpg')
cropped = img[img.shape[0]*0.45:img.shape[0],:]
#cleanedImage = cv2.GaussianBlur(cropped, (5,5), sigmaX = 1, sigmaY = 1)
cleanedImage = cropped


while(True):
	th1 = cv2.getTrackbarPos('Th1','Frame')
	th2 = cv2.getTrackbarPos('Th2','Frame')
	th3 = cv2.getTrackbarPos('Th3','Frame')
	
	sigma = cv2.getTrackbarPos('Sigma','Frame')
	theta = cv2.getTrackbarPos('Theta','Frame')
	lm = cv2.getTrackbarPos('LM','Frame')
	
	#cv2.imshow('BkgElim', be.eliminateBackground(cleanedImage))
	bkelim = be.eliminateBackground(cleanedImage)
	gaborKernel = cv2.getGaborKernel((101,101),1,np.pi*93/180.0,13,15)
	filtered = cv2.filter2D(bkelim[:,:,2], cv2.CV_32F, gaborKernel)
	#print np.max(bkelim[:,:,1]*(255-bkelim[:,:,1])
	#cv2.imshow('BackElim',filtered*(255-bkelim[:,:,0]))
	cv2.imshow('BKL', bkelim[:,:,0])
	cv2.imshow('Original', cleanedImage)
	b, g, r = cv2.split(np.copy(cleanedImage))

	flag,thresh = cv2.threshold(cleanedImage,170,255,cv2.THRESH_BINARY)
	gray = cv2.cvtColor(thresh,cv2.COLOR_BGR2GRAY)
	
	
	
	#cny = cv2.Canny(g,th2,th3, apertureSize = 3)
	
	#cv2.imshow('Gray',  gray)
	#cv2.imshow('Canny Out',cny)
	
	
	cnyfiltered = cv2.Canny(np.uint8(filtered),th2,th3,apertureSize=3)
	cannyblocks = cv2.Canny(bkelim[:,:,0],th2,th3,apertureSize=3)
	#mergeImg = cv2.merge((b,g,r))
	detectRectangles(cnyfiltered)	
	detectRectangles(cannyblocks,'bluebox')
	
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
cv2.destroyAllWindows()
cv2.waitKey(1)
