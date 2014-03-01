import cv2
import numpy as np

cap = cv2.VideoCapture(0)
while(True):
	# Capture frame-by-frame
	ret, frame = cap.read()
	imRed = frame[:,:,0]
	imGreen = frame[:,:,1]
	imBlue = frame[:,:,2]
	
	print frame.shape
	icv = np.zeros((imBlue.shape[0],imBlue.shape[1]))
	print icv.shape
	
	cv2.Canny(imBlue,edges=icv, threshold1=100, threshold2=300, apertureSize=3)
	
	imBlue = np.add(imBlue, icv)
	print type(icv)
	print type(imBlue)
	print type(imRed)
	print type(frame)
	
	print imBlue.shape
	print np.max(imBlue)
	imReassembled = cv2.merge((imRed,imGreen,imBlue))
	
	
	# Display the resulting frame
	cv2.imshow('frame',imReassembled)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
