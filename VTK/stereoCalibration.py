#!/usr/bin/env python

import numpy as np
import cv2
import argparse
import cPickle as pickle
#from common import splitfn

USAGE = '''
USAGE: calib.py [--f <filename>] [--s] [--w] [--h] [--n]
'''
if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(description='Calibrate two stereo cameras')
    parser.add_argument("-s","--size", help="Size of one square on the board. Default is .025",
                        type = float, default=.025)
    parser.add_argument("--width", help="Number of inner corners along the width of board. Default is 8",
                        type = int, default=8)
    parser.add_argument("--height", help="Number of inner corners along the height of board. Default is 6",
                        type = int, default=6)
    parser.add_argument("-n","--num_boards", help="Number of boards to capture. Default is 10",
                        type = int, default=10)
    args = parser.parse_args()
    numBoards = args.num_boards
    board_w = args.width
    board_h = args.height
    board_sz = (board_w,board_h)
    board_n = board_w*board_h

    pattern_points = np.zeros( (np.prod(board_sz), 3), np.float32 )
    pattern_points[:,:2] = np.indices(board_sz).T.reshape(-1, 2)  
    pattern_points *= args.size

    cap1 = cv2.VideoCapture(1)
    cap1.set(3,1280)
    cap1.set(4,720)
    cap2 = cv2.VideoCapture(0)
    cap2.set(3,1280)
    cap2.set(4,720)

    
    success = 0
    k = 0
    found1 = False
    found2 = False

    obj_points = []
    img_points1 = []
    img_points2 = []
    while (success < numBoards):
        ret, img1 = cap1.read()
        ret, img2 = cap2.read()
        grey1 = cv2.cvtColor(img1,cv2.COLOR_BGR2GRAY)
        grey2 = cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)
        
        found1, corners1 = cv2.findChessboardCorners(img1, board_sz)
        found2, corners2 = cv2.findChessboardCorners(img2, board_sz)

        if found1:
            term = ( cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1 )
            cv2.cornerSubPix(grey1, corners1, (11, 11), (-1, -1), term)
            cv2.drawChessboardCorners(img1, board_sz, corners1, found1)

        if found2:
            term = ( cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1 )
            cv2.cornerSubPix(grey2, corners2, (11, 11), (-1, -1), term)
            cv2.drawChessboardCorners(img2, board_sz, corners2, found2)
        
        cv2.imshow("image1", img1)
        cv2.imshow("image2", img2)
        k = cv2.waitKey(1)
        if k & 0xFF == ord('q'):
            break
        if k & 0xFF == ord(' ') and found1!=0 and found2!=0:
            img_points1.append(corners1.reshape(-1, 2))
            img_points2.append(corners2.reshape(-1, 2))
            obj_points.append(pattern_points)
            success +=1
    cv2.destroyAllWindows()
    print("Starting Calibration\n")
    term = (cv2.TERM_CRITERIA_COUNT+cv2.TERM_CRITERIA_EPS, 100, 1e-6)
    flag = cv2.CALIB_SAME_FOCAL_LENGTH + cv2.CALIB_ZERO_TANGENT_DIST + cv2.cv.CV_CALIB_FIX_PRINCIPAL_POINT + cv2.cv.CV_CALIB_FIX_ASPECT_RATIO
    retval, C1, D1, C2, D2, R, T, E, F = cv2.stereoCalibrate(obj_points, img_points1, img_points2,
                                                             (1280, 720), criteria = term, flags = flag)
    data = {"retVal": retval,
            "camera_matrix _1": C1.tolist(),
            "camera_matrix _2": C2.tolist(),
            "dist_coeff_1": D1.tolist(),
            "dist_coeff_2": D2.tolist(),
            "rotation_matrix": R.tolist(),
            "translation_vector": T.tolist(),
            "essential_matrix" : E.tolist(),
            "fundamental_matrix" : F.tolist()
            })
    fname = "stereoCams.json"
    print("Saving Calibration as "+fname)
    import json
    with open(fname, "w") as f:
        json.dump(data, f)