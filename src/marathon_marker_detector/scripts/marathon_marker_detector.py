#! /usr/bin/env python
import cv2
import numpy as np

def nothing():
    pass

cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH ,320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,240) 

cv2.namedWindow('mask')

cv2.createTrackbar('H_low', 'mask', 0, 180, nothing)
cv2.setTrackbarPos('H_low', 'mask',82)
cv2.createTrackbar('S_low', 'mask', 0, 255, nothing)
cv2.setTrackbarPos('S_low', 'mask',0)
cv2.createTrackbar('V_low', 'mask', 0, 255, nothing)
cv2.setTrackbarPos('V_low', 'mask',87)
cv2.createTrackbar('H_high', 'mask', 0, 180, nothing)
cv2.setTrackbarPos('H_high', 'mask',255)
cv2.createTrackbar('S_high', 'mask', 0, 255, nothing)
cv2.setTrackbarPos('S_high', 'mask',63)
cv2.createTrackbar('V_high', 'mask', 0, 255, nothing)
cv2.setTrackbarPos('V_high', 'mask',255)
cv2.createTrackbar('Blur', 'mask', 1, 99, nothing)
cv2.setTrackbarPos('Blur', 'mask',9)

cv2.createTrackbar('Lower', 'mask', 0, 255, nothing)
cv2.setTrackbarPos('Lower', 'mask',200)

cv2.createTrackbar('Upper', 'mask', 0, 255, nothing)
cv2.setTrackbarPos('Upper', 'mask',220)

while True:
    ret, frame = cap.read()
    if cv2.getTrackbarPos('Blur', 'mask') % 2 == 0 :
        x = cv2.getTrackbarPos('Blur', 'mask') + 1
    else :
        x = cv2.getTrackbarPos('Blur', 'mask')
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (x, x), 0)

    # hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    # lower = np.array([cv2.getTrackbarPos('H_low', 'mask'), cv2.getTrackbarPos('S_low', 'mask'), cv2.getTrackbarPos('V_low', 'mask')])
    # upper = np.array([cv2.getTrackbarPos('H_high', 'mask'), cv2.getTrackbarPos('S_high', 'mask'), cv2.getTrackbarPos('V_high', 'mask')])
    # mask  = cv2.inRange(hsv,lower, upper)
    # blurred = cv2.bitwise_and(blurred, blurred, mask=mask)

    binaryIMG = cv2.Canny(blurred, cv2.getTrackbarPos('Lower', 'mask'), cv2.getTrackbarPos('Upper', 'mask'))
    dilation = cv2.dilate(binaryIMG,(5, 5),iterations = 2)

    _, cnts, _ = cv2.findContours(dilation.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for c in cnts:
        hull = cv2.convexHull(c)
        if len(hull) > 6:
            ellipse = cv2.fitEllipse(hull)
            if ellipse[0][0] > 100 and ellipse[0][0] < 220:
                print(ellipse[1])
                cv2.ellipse(frame, ellipse, (0, 255, 0), 2)


    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break
    cv2.imshow('frame', frame)
    cv2.imshow('dilation', dilation)
    # cv2.imshow('blurred', blurred)
    # cv2.imshow('mask', mask)
    cv2.imshow('binary img', binaryIMG)
cv2.destroyAllWindows()