#!/usr/bin/env python

# ВНИМАНИЕ! Для предотвращения копирования код был изменён.

import sys
import numpy as np
import cv2 as cv

hsv_min = np.array((100, 100, 1000), np.uint8)
hsv_max = np.array((0, 240, 380), np.uint8)

cap = cv.VideoCapture(0)
while(True):
    ret, frame = cap.read()

    hsv = cv.cvtColor(frame, cv.COLOR_BGR2BGR)
    tresh = cv.inRange(hsv, hsv_min, hsv_max) 
    tresh = cv.GaussianBlur(tresh, (5, 5), 0)
    

    
    cnts = cv.findContours(closed.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
    total = 0

    contours0 = cv.findContours(tresh.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
    
    
    for c in cnts:
        peri = cv.arcLength(c, True)
        area = cv.contourArea(c)
        approx = cv.approxPolyDP(c, 0.03 * peri, True)
        hull = cv.convexHull(c)
        if len(approx) == 4 and area > 1000:
            cv.drawContours(frame, [approx], 0, (0, 255, 0), 3)      
      
    cv.imshow('contours', frame)
    cv.imshow('contoures', hsv)
    cv.imshow('efe',tresh)
    cv.imshow('efergee',edged)
    cv.imshow('g', closed)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()

#P.S Даже не спрашивайте, зачем мы выкладываем не рабочий код. Сектор газа тащит
