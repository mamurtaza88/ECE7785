#!/usr/bin/env python
import rospy
import numpy as np
import cv2

cap = cv2.VideoCapture(0)

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi',fourcc, 20.0, (640,480))

while(cap.isOpened()):
    ret, frame = cap.read()
    if ret==True:
    	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        normalizedImg = np.zeros((800, 800))
        cv2.normalize(gray,  normalizedImg, 0, 255, cv2.NORM_MINMAX)
        rows = gray.shape[0]
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1,  rows / 8, param1=90, param2=40, minRadius=10, maxRadius=110)
        #frame = cv2.flip(gray,0)

        cent_x = 0
        cent_y = 0
        radius = 0

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                center = (i[0], i[1])
                # circle center
                cv2.circle(gray, center, 1, (0, 100, 100), 3)
                # circle outline
                radius = i[2]
                cv2.circle(gray, center, radius, (255, 0, 255), 3)
                cent_x = i[0]
                cent_y = i[1]
        

       #cv.imshow("detected circles", src)

        # write the flipped frame
        font = cv2.FONT_HERSHEY_SIMPLEX
        #font = cv2.FONT_HERSHEY_SIMPLEX
        #display_text = str(i[0])+','+str(i[1])
        display_text = str(cent_x)+','+str(cent_y)

        #cv2.putText(gray,display_text, (i[0], i[1]), font, 1,(255,255,255),1)
        cv2.putText(gray,display_text, (cent_x, cent_y), font, 1,(255,255,255),1)
        
        center = (cent_x, cent_y)
        print 'Pixel Location', center
        out.write(gray)
        cv2.imshow('Color Frame',frame)
        cv2.imshow('normalized Image',normalizedImg)
        cv2.imshow('detected circles',gray)
        
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

# Release everything if job is finished
cap.release()
out.release()
cv2.destroyAllWindows()
