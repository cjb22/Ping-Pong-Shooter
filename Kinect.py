#!/usr/bin/env python
import numpy as np
import freenect
import cv2
import frame_convert2
import time


def nothing(x):
    pass

#Set up named windows
cv2.namedWindow('Depth')
# Depth = 480 x 640
#cv2.namedWindow('Video')
print('Press ESC in window to stop')

#Set up slidebar controls
cv2.createTrackbar('Low','Depth',0,255,nothing)
cv2.setTrackbarPos('Low', 'Depth',5)
cv2.createTrackbar('High','Depth',0,255,nothing)
cv2.setTrackbarPos('High', 'Depth',35)
cv2.createTrackbar('Contour Length','Depth',0,6000,nothing)
cv2.setTrackbarPos('Contour Length', 'Depth',1500)
cv2.createTrackbar('Contour Area','Depth',0,200,nothing)
cv2.setTrackbarPos('Contour Area', 'Depth',10)



#Fetch Images
hand   = cv2.imread("/home/pi/Desktop/Ping Pong Shooter/Hand.png", 0)

def get_depth():
    return frame_convert2.pretty_depth_cv(freenect.sync_get_depth()[0])

def get_video():
    return frame_convert2.video_cv(freenect.sync_get_video()[0])


#Main loop
while 1:
    #------------------------------------------------------------------------------
    #Measure start time
    start = time.time()

    #Read Trackbars
    low         = cv2.getTrackbarPos('Low', 'Depth')
    high        = cv2.getTrackbarPos('High', 'Depth')
    areaThresh  = cv2.getTrackbarPos('Contour Length', 'Depth')
    lenThresh   = cv2.getTrackbarPos('Contour Area', 'Depth')
    
    imgray = get_depth()

    edges = cv2.Canny(imgray,low,high,3)

    #Dilation and erosion 
    edKernel = np.ones((3,3),np.uint8)
    edges = cv2.dilate(edges, edKernel, iterations = 1)

    #Contour detection
    contours, hierarchy = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    #Contour filtering
    filteredContours = []
    for ctr in contours:
        approx = cv2.approxPolyDP(ctr,0.01*cv2.arcLength(ctr,True),True)
        area = cv2.contourArea(ctr)
        if area > areaThresh and len(approx) > lenThresh: 
            filteredContours.append(ctr)        

    #draw contours
    print (len(filteredContours))
    cv2.drawContours(imgray, filteredContours, -1, (0,255,0), 2)

        
    #Display the image
    cv2.imshow('Depth', imgray)
    cv2.imshow('Edges', edges)
    #cv2.imshow('Video', get_video())
    
    if cv2.waitKey(10) == 27:
        break


    #Calculate the FPS
    totTime = time.time() - start
    FPS = 1 / totTime
    #-------------------------------------------------------------------------------
    
# When everything done, release the capture
print("FPS: ", FPS)
cv2.destroyAllWindows()
