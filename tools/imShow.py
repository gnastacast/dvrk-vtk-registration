#!/usr/bin/env python

import numpy as np
import cv2
import argparse
import time
if __name__ == '__main__':
    cap = cv2.VideoCapture(1)
    cap.set(3,960)
    cap.set(4,720)
    while(True):
        ret, img = cap.read()
        cv2.flip(img, -1, img) 
	img_cropped = img[130:830, 10:710]
        cv2.imshow("image", img_cropped)

        k = cv2.waitKey(1)
	
	time.sleep(.1)
        if k & 0xFF == ord('q'):
            break
