#!/usr/bin/env python
"""
Данный алгоритм позволяет детектировать красные парралепипеды по цвету и контуру. Алгоритм работы следующий:
    1.Конвертирование RGB изобрадения с камеры в HSV
    2.Задаём ThreshHold по минимальному и максимальному значению HSV
    3.Размываем, фильтруем конутры и делаем морфологическое преобразование изображения.
    4.Находим контуры функцией findCountours
    5.Задаём контурную область
    6.Вычисляем моменты
    7.ъВырисовываем квадрат
Код спроектирован на детектирование только красного цвета.
"""
import sys
import numpy as np
import cv2 as cv

hsv_min = np.array((140, 100, 100), np.uint8)
hsv_max = np.array((190, 240, 250), np.uint8)

cap = cv.VideoCapture(0)
while(True):
    ret, frame = cap.read()

    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    tresh = cv.inRange(hsv, hsv_min, hsv_max) 
    tresh = cv.GaussianBlur(tresh, (5, 5), 0)
    edged = cv.Canny(tresh, 100, 300)

    kernel = cv.getStructuringElement(cv.MORPH_RECT, (7, 7))
    closed = cv.morphologyEx(edged, cv.MORPH_CLOSE, kernel)
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
