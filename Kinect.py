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
def get_depth(img):
    return frame_convert2.pretty_depth_cv(img)
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

#Function to draw line segments
def drawSegments(segments, Thickness, drawWithAlternatingThickness = False):
    i = 0
    for seg in segments:
        if not drawWithAlternatingThickness:
            cv2.drawContours(imgray, seg, -1 , (0,255,0), Thickness)
        else:
            if i % 2 == 0:
                cv2.drawContours(imgray, seg, -1 , (0,255,0), Thickness)
            else:
                cv2.drawContours(imgray, seg, -1 , (0,255,0), Thickness * 2)
            i = i + 1

           
#Calculate actual distance in meters using a Kinect 360 reading
def distanceInMeters(kinectReading):
    #Depth sensing
    #pX, pY = 640/2, 480/2
    #distKin     = rawDepth [pY,pX]
    #dist = distanceInMeters(distKin)
    #cv2.circle(imgray,(pX,pY), 7, (0,255,0), -1)
    #cv2.putText(imgray, str(dist), (pX - 20, pY - 20), cv2.FONT_HERSHEY_SIMPLEX, 4, (0, 255, 0), 4)
    return 0.1236 * math.tan( kinectReading / 2842.5 + 1.1863)

#Function to find the center pixel location of a contour segment
def findCenterCoordinates(seg):
    coordinatesAreValid = False
    centerX = -1
    centerY = -1
    M = cv2.moments(seg)
    if M["m00"] > 0:
        centerX = int(M["m10"] / M["m00"])
        centerY = int(M["m01"] / M["m00"])  
        if centerX > 0 and centerX < 640 and centerY > 0 and centerY < 480:
            coordinatesAreValid = True
    return centerX, centerY, coordinatesAreValid

#Find hand shaped contour segments using depth filtering, center locations and the number of convex hull defects
def findHandshapes(contourSegments, Range):
    hulls = []
    minDist = -1
    maxDist = -1
    minDefects = -1
    maxDefects = -1
    

    if Range == "C" or Range == "Close":
        minDist = 0.5
        maxDist = 1.6
        minDefects = 5
        maxDefects = 13
    elif Range == "M" or Range == "Mid":
        minDist = 1.5
        maxDist = 2.5
        minDefects = 5
        maxDefects = 13
    elif Range == "F" or Range == "Far":
        minDist = 2.5
        maxDist = 4.0
        minDefects = 3
        maxDefects = 10

    

    for seg in contourSegments:

        #First check if the segment is at the right depth and at a proper pixel location
        centerX, centerY, coordinatesAreValid = findCenterCoordinates(seg)
        if coordinatesAreValid:
            centerDist = distanceInMeters( rawDepth [centerY,centerX] )
            if centerDist > minDist and centerDist < maxDist:

                #Next, check if the number of convex hull defects
                hull = cv2.convexHull(seg,returnPoints = False)
                defects = cv2.convexityDefects(seg,hull)
                amtOfHullDefects = defects.shape[0]
                if  amtOfHullDefects >= minDefects and amtOfHullDefects <= maxDefects:
                    hulls.append(seg)
    return hulls


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

    #Get the raw 11-bit integer depth image, and make an 8-bit copy for OpenCV operations 
    rawDepth = freenect.sync_get_depth()[0]
    imgray = np.copy(rawDepth)
    imgray = get_depth(imgray)

    #Use Canny algorithm to find contours on the depth image
    edges = cv2.Canny(imgray,cannyLow,cannyHigh,3)

    #Dilation and erosion to make the edges more pronounced
    edKernel = np.ones((3,3),np.uint8)
    edges = cv2.dilate(edges, edKernel, iterations = 1)

    #Contour detection
    contours, hierarchy = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    #Preliminary contour filtering
    filteredContours = []
    for ctr in contours:
        approx = cv2.approxPolyDP(ctr,0.01*cv2.arcLength(ctr,True),True)
        area = cv2.contourArea(ctr)
        if area > ctrAreaThresh and len(approx) > ctrLenThresh: 
            filteredContours.append(ctr)        

    #Draw the contours
    #cv2.drawContours(imgray, filteredContours, -1, (0,255,0), 2)

    #Find line segments which might resemble a hand
    contourSegmentsClose    = findHandLineSegments(filteredContours, 345, 80)
    contourSegmentsMid      = findHandLineSegments(filteredContours, 200, 50)
    contourSegmentsFar      = findHandLineSegments(filteredContours, 80, 30)

    #Draw the line segments
    #drawSegments(contourSegmentsClose,3,True)
    #drawSegments(contourSegmentsMid,3,True)
    #drawSegments(contourSegmentsFar,3,True)
    
    #Convex hulls of hands
    hullsClose      = findHandshapes(contourSegmentsClose, "Close")
    hullsMid        = findHandshapes(contourSegmentsMid, "Mid")
    hullsFar        = findHandshapes(contourSegmentsFar, "Far")
    
    drawSegments(hullsClose,2)
    drawSegments(hullsMid,4)
    drawSegments(hullsFar,6)
      
            
            
        
            


    

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
