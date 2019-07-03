#!/usr/bin/env python
import freenect
import cv2
import frame_convert2

cv2.namedWindow('Depth')
#cv2.namedWindow('Video')

print('Press ESC in window to stop')


def get_depth():
    return frame_convert2.pretty_depth_cv(freenect.sync_get_depth()[0])


def get_video():
    return frame_convert2.video_cv(freenect.sync_get_video()[0])

while 1:
    imgray = get_depth()
    #imgray = cv2.GaussianBlur(imgray, (3, 3), 0)

    edges = cv2.Canny(imgray,100,200,7)

    #Display the image
    cv2.imshow('Depth', imgray)
    cv2.imshow('Edges', edges)
    #cv2.imshow('Video', get_video())
    
    if cv2.waitKey(10) == 27:
        break

# When everything done, release the capture
#cap.release()
cv2.destroyAllWindows()
