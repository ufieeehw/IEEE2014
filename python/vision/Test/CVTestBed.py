import cv2
import numpy as np

def nothing(x):
	#Convenience Function
	pass

def normalize(m):
	m += 0-np.min(m)
	m /= np.max(m)

	return(m)

cv2.namedWindow('frame')
#cv2.namedWindow('edge')

cap = cv2.VideoCapture(0)



cap.set(3,120)
cap.set(4,120)



cv2.createTrackbar('Th1','frame',0,255,nothing)
cv2.createTrackbar('Th2','frame',0,25,nothing)
cv2.createTrackbar('Th3','frame',0,255,nothing)



def main(frame):
	# Capture frame-by-frame
	
	imRed = frame[:,:,0]
	imGreen = frame[:,:,1]
	imBlue = frame[:,:,2]
	
	edgeImg = np.uint8(np.zeros((imBlue.shape)))
	
	th1 = cv2.getTrackbarPos('Th1','frame')
	th2 = cv2.getTrackbarPos('Th2','frame')
	th3 = cv2.getTrackbarPos('Th3','frame')
	#cv2.Canny(image=imBlue,edges=edgeImg, threshold1=th1, threshold2=th2, apertureSize=5)
	
	#sift = cv2.SIFT()
	#kp = sift.detect(imBlue,None)
	"""
	fast = cv2.FastFeatureDetector()
	fast.setBool('nonmaxSuppression',1)
	fast.setInt('threshold',th1)
	fast.setInt('response',th2)
	kp = fast.detect(imRed,None)
	#print dir(kp[0])
	if(kp):
		kpr = kp[0]
		print kpr.angle
		print kpr.octave
		print kpr.pt
		print kpr.response
		print kpr.size
	"""
	draw = np.zeros(imBlue.shape,np.uint8)
	mser = cv2.MSER()
	
	#mser.setInt('threshold',th1)
	
	kp = mser.detect(imRed)
	nums = 0;
	for k in kp:
		nums += 1
		#print len(k)
		for j in k:
			draw[j[1],j[0]] = (nums*10) % 255
	
	#print "Threshold: ", fast.getInt('threshold')
	#print "nonmaxSuppression: ", fast.getBool('nonmaxSuppression')
	#print "Total Keypoints with nonmax sup", len(kp)
	
	

	#img=cv2.drawKeypoints(imBlue,kp, color=(255,0,0))
	#img = imBlue


	imReassembled = cv2.merge((imRed,imGreen,draw))
	#imThreshA = np.logical_and(imRed>th1,imGreen>th2,imBlue>th3)
	#imThresh = np.uint8(imThreshA)*255
	
	#imThresh = np.uint8(imRed>th1)*255
	
	# Display the resulting frame
	cv2.imshow('frame',imReassembled)
	#cv2.imshow('edge', imThresh)


Times = 0
comp = 15
while(True):
	Times += 1
	ret, frame = cap.read()
	if Times % comp == 0:
		
		main(frame)
	if Times >= comp*10:
		Times = 0BlockBlue
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
