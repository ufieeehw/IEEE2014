import cv2
import numpy as np

def nothing(x):
    pass

def makeOdd(x):
    if(not x%2):
        x += 1
    return x
  
def discoverSquares(image,canny1=64,canny2=31, discrim1=2.0, dicrim2=3.0):
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
    cv2.imshow('Test Display -- dsqrs', edges) 
    contours,hierarchy = cv2.findContours(np.array(edges,np.uint8), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE) 
    dispimg = np.array(np.zeros((image.shape[0],image.shape[1],3)),np.uint8)
    for ctr in contours:
        arcLen = cv2.arcLength(ctr,True)
        if arcLen > 0.1:
            approx = cv2.approxPolyDP(ctr,arcLen*(12/500.0), True)    
    
            if len(approx)>=4:# and cv2.isContourConvex(approx): #&&np.fabs(cv2.contourArea(approx)) > 30
                sumAngle = 0
                #cv2.drawContours(dispimg,approx,-1, (50,80,200))
                for j in range(2,len(approx)):
                    angle = angle_between(approx[j%len(approx)][0], approx[j-2][0], approx[j-1][0])
                    sumAngle += angle
                if sumAngle > 3 or sumAngle < 4:
                    #cv2.drawContours(dispimg,[approx],-1,(255,0,100))
                    finalContours.append(approx)
                            
    #print type(contours)    
    print str(len(contours) - len(finalContours))
    #print "Hello"
    #input =  cv2.imread('../Implementation/CoursePracticeMarch2.png')
    #input = cv2.imread('../Implementation/CourseMarch2-3.png')
    cv2.drawContours(dispimg,finalContours, -1, (255,0,100),thickness=1)
    return dispimg, finalContours

def discover_canny_squares(raw_image, vary_1=1,vary_2=1, vary_3=1):
    thresh_level = 2
    finalContours = []
    dilateKernel = np.ones((5,5),np.uint8)
    edges = None
    contours = None
    
    #adapt_thresh_img = cv2.adaptiveThreshold(channel,50,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, makeOdd(vary_1),vary_2)
    
    adapt_thresh_image = adapt_thresh(raw_image, vary_1,vary_2,vary_3)
    cv2.imshow("Adapt_Thresh",adapt_thresh_image)
    tests = [0,(66,109),(42,70)]
    for ind, test in enumerate(tests):
        if ind == 0:
            
            canny_edges = cv2.Canny(raw_image, vary_1, vary_2, apertureSize = 3)

            dilated_edges = cv2.dilate(canny_edges, dilateKernel, iterations = 1,anchor = (-1,-1))
            addmin = lambda o: np.uint8(o + np.min(o))
            horizontals = addmin(discoverHorizontalLines(eliminateBackground(raw_image)))
            cv2.imshow("Canny", horizontals)

        else:
            pass

def adapt_thresh(image):
    HSVimage = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    blue_lower=np.array([100, 0,0],np.uint8)
    #blue_lower=np.array([100,th2,th3],np.uint8)
    blue_upper=np.array([140,255,255],np.uint8)
    blue=cv2.inRange(HSVimage,blue_lower,blue_upper)
    #Isolation of low-high blue
    #Target element: Firing Blocks
    
    histeq = lambda input_image: cv2.equaliztHist(input_image)
    
    thresh = lambda input_image, C_val: cv2.adaptiveThreshold(input_image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, makeOdd(194),C_val)
    h,s,v = cv2.split(HSVimage)
    
    limits = (143,28,0)
    #merged =  cv2.merge(map(thresh,(h,s,v),(th2,th3,th4)))
    return thresh(v,0)



if __name__=='__main__':
    import sys, os
    #image = cv2.imread('../Implementation/TwoBlueBlocks.png')
    #input = cv2.imread('../Implementation/myalgo2101974.jpg')
    #input = cv2.imread('../Implementation/myalgo2099447.jpg')
    
    path = os.path.dirname(os.path.abspath(__file__))
    path = os.path.abspath(os.path.join(path,".."))
    if len(sys.argv) > 1:
            image_name = sys.argv[1]
    else:
        image_name = 'frame0000'
        
    fex = '.jpg'
    if '.' in image_name:
        fex = ''
            
            
    input_image = cv2.imread(path + '/Debug/' + image_name + fex)
    image = cv2.GaussianBlur(input_image, (7, 7), sigmaX = 1, sigmaY = 1)

    cv2.namedWindow('Frame')
    
    
    cv2.createTrackbar('Th1', 'Frame', 194, 255,nothing)
    cv2.createTrackbar('Th2', 'Frame', 143, 255,nothing)
    cv2.createTrackbar('Th3', 'Frame', 28, 255,nothing)
    cv2.createTrackbar('Th4', 'Frame', 0, 255, nothing)
    cv2.createTrackbar('Th5', 'Frame', 21, 255, nothing)
    HSVimage = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)



    while(True):
        th1 = cv2.getTrackbarPos('Th1','Frame')
        th2 = cv2.getTrackbarPos('Th2','Frame')
        th3 = cv2.getTrackbarPos('Th3','Frame')
        th4 = cv2.getTrackbarPos('Th4','Frame')
        th5 = cv2.getTrackbarPos('Th5','Frame')
        

        
        blue_lower=np.array([100, 0,0],np.uint8)
        #blue_lower=np.array([100,th2,th3],np.uint8)
        blue_upper=np.array([140,255,255],np.uint8)
        blue=cv2.inRange(HSVimage,blue_lower,blue_upper)
        #Isolation of low-high blue
        #Target element: Firing Blocks
        
        #histeq = lambda input_image: cv2.equaliztHist(input_image)
        
        #thresh = lambda input_image, C_val: cv2.adaptiveThreshold(input_image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, makeOdd(th1),C_val)
        #h,s,v = cv2.split(HSVimage)
        
        #merged =  cv2.merge(map(thresh,(h,s,v),(th2,th3,th4)))
        merged = adapt_thresh(image)
        cv2.imshow("Merged", merged)
        

        #HERE: TARGETING WHITE LINES!
        #acceptable_pixels = blue & 
        #cv2.imshow("Acceptable Pixels", acceptable_pixels)





        keyPress = cv2.waitKey(30) & 0xFF
        if (keyPress):
            if(keyPress == ord('q')):

                break
    cv2.destroyAllWindows()
