# -*- coding: utf-8 -*-
"""
Created on Tue Nov 19 19:24:30 2019

@author: oscar
"""
import cv2 

cam = cv2.VideoCapture(0)
while True:
    # Read each frame in video stream
    ret, frame = cam.read()
    # Display each frame in video stream
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    
    radius = 4
    num_corners=100
    corners=cv2.goodFeaturesToTrack(gray,num_corners,.01,10)
    for i in range(corners.shape[0]):
        cv2.circle(gray, (corners[i,0,0], corners[i,0,1]), radius, (0,0,0), cv2.FILLED)
    cv2.imshow("Capture", gray)
    if not ret:
        break
# Monitor keystrokes
    k = cv2.waitKey(1)

    if k & 0xFF == ord('q'):
        # q key pressed so quit
        print("Quitting...")
        cv2.destroyAllWindows()
        break
    elif k & 0xFF == ord('c'):
        # c key pressed so capture frame to image file
        cap_name = "capture_{}.png".format(cap_cnt)
        cv2.imwrite(cap_name, frame)
        print("Saving {}!".format(cap_name))
        # Increment Capture Counter for next frame to capture
        cap_cnt += 1
cam.release()
print('Released')
    