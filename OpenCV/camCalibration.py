#!/usr/bin/python

import cv2.cv as cv
import time

cv.NamedWindow("camera", 1)

capture = cv.CaptureFromCAM(0)

chessboard_dim = ( 9, 6 )

while True:
    img = cv.QueryFrame(capture)
    im3 = cv.QueryFrame(capture)

    found_all, corners = cv.FindChessboardCorners( img, chessboard_dim )
    print found_all, len(corners)

    cv.DrawChessboardCorners( im3, chessboard_dim, corners, found_all )

    cv.ShowImage("camera", im3)
    cv.WaitKey()
    cv.DestroyAllWindows()

    if cv.WaitKey(10) == 27:
        break
cv.DestroyAllWindows()
