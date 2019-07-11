#!/usr/bin/env python
import numpy as np
import freenect
import cv2
import frame_convert2
import time
import math


def nothing(x):
    pass

#Set up named windows
cv2.namedWindow('Depth')
# Depth = 480 x 640
#cv2.namedWindow('Video')
print('Press ESC in window to stop')

#Set up slidebar controls
cv2.createTrackbar('Canny Low','Depth',0,255,nothing)
cv2.setTrackbarPos('Canny Low', 'Depth',5)
cv2.createTrackbar('Canny High','Depth',0,255,nothing)
cv2.setTrackbarPos('Canny High', 'Depth',35)
cv2.createTrackbar('Contour Length','Depth',0,6000,nothing)
cv2.setTrackbarPos('Contour Length', 'Depth',1500)
cv2.createTrackbar('Contour Area','Depth',0,200,nothing)
cv2.setTrackbarPos('Contour Area', 'Depth',10)
#cv2.createTrackbar('Segment Length','Depth',1,500,nothing)
#cv2.setTrackbarPos('Segment Length', 'Depth',200)
#cv2.createTrackbar('Closeness Threshold','Depth',0,300,nothing)
#cv2.setTrackbarPos('Closeness Threshold', 'Depth',80)


#Kinect Image Functions
def get_depth():
    return frame_convert2.pretty_depth_cv(freenect.sync_get_depth()[0])
def get_video():
    return frame_convert2.video_cv(freenect.sync_get_video()[0])

#Function to check if a point is within specified coordinates
xRes, yRes = 640, 480           # Width x Height  = 640 x 480
boundaryThresholdX = 30
boundaryThresholdY = 80
def insideBoundaries(x, y):
    if x < xRes - boundaryThresholdX and x > boundaryThresholdX:
        if y < yRes - boundaryThresholdY and y > boundaryThresholdY:
            return True
    return False

def findHandLineSegments(contours, segmentLength, maxEndpointCloseness):
    contourSegments = []
    for ctr in contours:
        amtOfSegments = len(ctr) / segmentLength  
        for i in range(amtOfSegments) :
            #Find lines
            line = ctr[ i * segmentLength : (i+1) * segmentLength ,:]
            x0 = line[0][0][0]
            x1 = line[-1][0][0]
            y0 = line[0][0][1]
            y1 = line[-1][0][1]
            xMid = line[segmentLength / 2][0][0]
            yMid = line[segmentLength / 2][0][1]
            distance = math.hypot(x1 - x0, y1 - y0)
            if distance < maxEndpointCloseness:
                if insideBoundaries(x0, y0) and insideBoundaries(x1, y1) and insideBoundaries(xMid, yMid):
                    contourSegments.append( line )

            #Find lines between halfwaypoints of the lines (overlapping line segments)
            if i > 0:
                line = ctr[ i * segmentLength - ( segmentLength / 2 )  : (i+1) * segmentLength - ( segmentLength / 2 ) ,:]
                x0 = line[0][0][0]
                x1 = line[-1][0][0]
                y0 = line[0][0][1]
                y1 = line[-1][0][1]
                xMid = line[segmentLength / 2][0][0]
                yMid = line[segmentLength / 2][0][1]
                distance = math.hypot(x1 - x0, y1 - y0)
                if distance < maxEndpointCloseness:
                    if insideBoundaries(x0, y0) and insideBoundaries(x1, y1) and insideBoundaries(xMid, yMid):
                        contourSegments.append( line )

    return contourSegments


    
#Main loop
while 1:
    #------------------------------------------------------------------------------
    #Measure start time
    start = time.time()

    #Read Trackbars
    cannyLow            = cv2.getTrackbarPos('Canny Low', 'Depth')
    cannyHigh           = cv2.getTrackbarPos('Canny High', 'Depth')
    ctrAreaThresh       = cv2.getTrackbarPos('Contour Length', 'Depth')
    ctrLenThresh        = cv2.getTrackbarPos('Contour Area', 'Depth')
    #segmentLength       = cv2.getTrackbarPos('Segment Length', 'Depth')
    #segLenThreshold     = cv2.getTrackbarPos('Closeness Threshold', 'Depth')

    imgray = get_depth()

    edges = cv2.Canny(imgray,cannyLow,cannyHigh,3)

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
        if area > ctrAreaThresh and len(approx) > ctrLenThresh: 
            filteredContours.append(ctr)        

    #Draw the contours
    #cv2.drawContours(imgray, filteredContours, -1, (0,255,0), 2)

    #Find line segments which might resemble a hand
    contourSegmentsClose    = findHandLineSegments(filteredContours, 300, 80)
    contourSegmentsMid      = findHandLineSegments(filteredContours, 200, 50)
    contourSegmentsFar      = findHandLineSegments(filteredContours, 80, 30)

    #Draw the line segments
    drawWithAlternatingThickness = True
    print len(contourSegmentsMid)
    if len(contourSegmentsMid) > 0:     
        i = 0        
        for seg in contourSegmentsMid:
            if drawWithAlternatingThickness:
                if i % 2 == 0:
                    cv2.drawContours(imgray, seg, -1 , (0,255,0), 2)
                else:
                    cv2.drawContours(imgray, seg, -1 , (0,255,0), 6)
                i = i + 1
            else:
                cv2.drawContours(imgray, seg, -1 , (0,255,0), 5)
            
        

    #Display the image
    cv2.imshow('Depth', imgray)
    #cv2.imshow('Edges', edges)
    #cv2.imshow('Video', get_video())
    
    if cv2.waitKey(10) == 27:
        break


    #Calculate the FPS
    totTime = time.time() - start
    FPS = 1 / totTime
    #-------------------------------------------------------------------------------
    
# When everything done, release the capture
print 'FPS: ', FPS
cv2.destroyAllWindows()
