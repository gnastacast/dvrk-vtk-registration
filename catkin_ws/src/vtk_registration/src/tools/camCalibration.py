#!/usr/bin/env python

import numpy as np
import cv2
import argparse
#from common import splitfn

USAGE = '''
USAGE: calib.py [--f <filename>] [--s] [--w] [--h] [--n]
'''
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Calibrate two stereo cameras. Press space bar to capture a chessboard')
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

    cap = cv2.VideoCapture(0)
    cap.set(3,1280)
    cap.set(4,720)

    success = 0
    k = 0
    found = False

    obj_points = []
    img_points = []

    while (success < numBoards):
        ret, img = cap.read()

        grey = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        
        found, corners = cv2.findChessboardCorners(img, board_sz)

        if found:
            term = ( cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1 )
            cv2.cornerSubPix(grey, corners, (11, 11), (-1, -1), term)
            cv2.drawChessboardCorners(img, board_sz, corners, found)
        
        cv2.imshow("image", img)
        k = cv2.waitKey(1)
        if k & 0xFF == ord('q'):
            break
        if k & 0xFF == ord(' ') and found!=0:
            img_points.append(corners.reshape(-1, 2))
            obj_points.append(pattern_points)
            success +=1
            print(str(success)+' / '+str(numBoards))
    cv2.destroyAllWindows()
    print("Starting Calibration\n")
    term = (cv2.TERM_CRITERIA_COUNT+cv2.TERM_CRITERIA_EPS, 100, 1e-6)
    flag = cv2.CALIB_ZERO_TANGENT_DIST + cv2.cv.CV_CALIB_FIX_PRINCIPAL_POINT + cv2.cv.CV_CALIB_FIX_ASPECT_RATIO
    retval, cameraMatrix, distCoeff, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points,
                                                                         (1280, 720), criteria = term, flags = flag)
    data = {"retVal": retval,
            "camera_matrix": cameraMatrix.tolist(),
            "dist_coeff": distCoeff.tolist()
           }
    fname = "singleCam.json"
    print("Saving Calibration as "+fname)
    import json
    with open(fname, "w") as f:
        json.dump(data, f)