#Experimental Target Detection
import cv2
import cv2.cv as cv
import numpy as np
import sys, os
def nothing(x):
    pass
def get_center_of_mass(ctr,minimumArea=0):
    M = cv2.moments(ctr)
    cx = 0
    cy = 0
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
            return((cx,cy))
        else:
            return(None)
        

if __name__ == '__main__':

    cv2.namedWindow('Frame')
    cv2.createTrackbar('Th1','Frame',1,300,nothing)
    cv2.createTrackbar('Th2','Frame',1,3000,nothing)
    cv2.createTrackbar('Th3','Frame',200,3000,nothing)



    path = os.path.dirname(os.path.abspath(__file__))
    #path = os.path.abspath(os.path.join(path,".."))
    if len(sys.argv) > 1:
            image_name = sys.argv[1]
    else:
        image_name = 'frame0000'
        
    fex = '.jpg'
    if '.' in image_name:
        fex = ''
    input_image = cv2.imread(path + '/Debug/' + image_name + fex)
    image = cv2.GaussianBlur(input_image, (9, 9), sigmaX = 1, sigmaY = 1)
    cv2.imshow("original",image)
    HSVimage = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    
    
    #while(True):
    th1 = cv2.getTrackbarPos('Th1','Frame')
    th2 = cv2.getTrackbarPos('Th2','Frame')
    th3 = cv2.getTrackbarPos('Th3','Frame')
    
    #USE ROI
    r_low = (154, 30, 0)
    r_high = (180, 255, 255)
    red = cv2.inRange(HSVimage, r_low,r_high)
    compare = red
    
    
    dilateKernel = np.ones((7,7),np.uint8)
   
    board_rim = cv2.dilate(compare, dilateKernel, iterations = 1,anchor = (-1,-1))
    #cv2.imshow("Dilated",board_rim)
    
    
    openKernel = np.ones((10,10),np.uint8)
    board_opened = cv2.morphologyEx(board_rim,cv2.MORPH_CLOSE, openKernel)
    cv2.imshow("Opened", board_opened)
    
    square = cv2.dilate(compare,dilateKernel,iterations = 10, anchor = (-1,-1))
    inside_circle = board_rim
    
    search_image = board_opened
    cv2.imshow("Search", search_image)
    
    search_contour,hierarchy = cv2.findContours(search_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    print hierarchy
    cv2.drawContours(image,search_contour, -1, (0,255,150), thickness=-1)
    
    
    
    #cv2.HoughCircles(image, method, dp, minDist[, circles[, param1[, param2[, minRadius[, maxRadius]]]]])  circles
    
    gs = image[:,:,2]
    
    #cv2.imshow("Gray",gs)
    circles = cv2.HoughCircles(search_image, cv.CV_HOUGH_GRADIENT, dp=8, minDist=1, param1=50, param2=80, minRadius = 4, maxRadius = 40)
    #circle_image = np.zeros(gs.shape, np.uint8)
    #print list(circles[0])
    #print type(circles)
    if not circles is None:
        for circ in list(circles.tolist()[0]):
            #print circ
            point = (int(circ[0]),int(circ[1]))
            
            radius = int(circ[2])
            cv2.circle(image, point, radius, (250,100,100))
    

    cv2.imshow('Display',image)
    cv2.waitKey(0)
        #if cv2.waitKey(1) & 0xFF == ord('q'):
        #    break
        #    cv2.destroyAllWindows()
