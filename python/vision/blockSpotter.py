import cv2
import numpy as np

try:
	from IEEE2014Functions import jIEEE2014 as ie
except:
	print "Jacob IEEE Library import failed"

try:
	from IEEE2014Functions import backgroundEliminator as be
except:
	print "Background Elimination Import Failed"
def nothing(x):
	pass

cv2.namedWindow('Frame')

cv2.createTrackbar('Th1','Frame',25,50,nothing)
cv2.createTrackbar('Th2','Frame',500,3000,nothing)
cv2.createTrackbar('Th3','Frame',200,3000,nothing)




	
img = cv2.imread('./Implementation/myalgo2099447.jpg')
cropped = img[img.shape[0]*0.45:img.shape[0],:]
cleanedImage = cv2.GaussianBlur(cropped, (3,3), sigmaX = 1, sigmaY = 1)

while(True):
	th1 = cv2.getTrackbarPos('Th1','Frame')
	th2 = cv2.getTrackbarPos('Th2','Frame')
	th3 = cv2.getTrackbarPos('Th3','Frame')
	

	bkelim = be.eliminateBackground(cleanedImage)

	# Generate the Gabor, isolate horizontal white lines
	gaborKernel = cv2.getGaborKernel((101,101),1,np.pi*93/180.0,13,15)
	horizontalLines = cv2.filter2D(bkelim[:,:,2], cv2.CV_32F, gaborKernel)
	cv2.imshow('Horizontal',horizontalLines)
	
	blocks = np.uint8(bkelim[:,:,0])
	print type(blocks)
	#contours, hierarchy = cv2.findContours(blocks,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	#F = np.zeros_like(cleanedImage)
	#print type(F)
	#cv2.drawContours(F,contours, -1,  (255,255,0))
	cv2.imshow('Show',bkelim)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
cv2.destroyAllWindows()
