#!/usr/bin/env python
from __future__ import print_function
import rospy
import tf
from geometry_msgs.msg import PoseStamped
import numpy as np
import cv2
import argparse
import copy
import codecs
import json


USAGE = '''
USAGE: calib.py [-s] [--width] [--height]
'''
'''
While this isn't an entirely passive way of using the davinci, it seems it will 
do the job.

First, you run 

roslaunch dvrk_nri_robot test_dvrk_full.launch foo:=0

foo:=0 for left MTM and PSM, foo:=1 for right MTM and PSM. 

in a terminal to bring up the control window. 

Then, you click home. Once it is homed, you can put the tool in as usual.

The davinci should be publishing to all the ros topics you need. 
You can see them with

rostopic list

and look at specfic ones with rostopic echo. (in a new terminal)

In order to move the PSM manually, you can press the white button on the top.

This acts as a clutch for the entire PSM (not just the carriage!) so you can 
move it around while the button is pressed in order to reposition. 

Be careful that once the button is released, it goes back into gravity
compensation mode, so only move it around while the button is pressed!
'''

'''
http://mirror.umd.edu/roswiki/doc/diamondback/api/tf/html/python/tf_python.html
PLAN:

class DaVinciPoseEstimator:
    def __init__(height,width):


'''

# Taken from http://docs.opencv.org/3.0-beta/doc/py_tutorials/py_calib3d/py_pose/py_pose.html 
def _draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)

class _PoseUpdater:
    currentPose = PoseStamped()
    def updateCB(self, data):
        self.currentPose = copy.deepcopy(data)
    def getRotationMatrix(self):
        return False

def axisFromPoints(points,center):
    axis = np.zeros(3)

    for i in range(1,len(points)):
        vec = points[i] - center
        vec = normalized(vec)
        axis += vec
    axis = normalized(axis)
    return axis

def normalized(vec):
    mag = np.linalg.norm(vec)
    return vec/mag

def registerDaVinci():
    obj_text = codecs.open('defaults/Logitech_c920/singleCam.json', 'r', encoding='utf-8').read()
    camParams = json.loads(obj_text)
    camMatrix = np.matrix(camParams['camera_matrix'])
    camDistort = np.matrix(camParams['dist_coeff'])
    parser = argparse.ArgumentParser(description='Calibrate two stereo cameras. Press space bar to capture a chessboard')
    parser.add_argument("-s","--size", help="Size of one square on the board. Default is .025",
                        type = float, default=.025)
    parser.add_argument("--width", help="Number of inner corners along the width of board. Default is 8",
                        type = int, default=8)
    parser.add_argument("--height", help="Number of inner corners along the height of board. Default is 6",
                        type = int, default=6)
    args = parser.parse_args()
    numBoards = 5;
    board_w = args.width
    board_h = args.height
    board_sz = (board_w,board_h)
    board_n = board_w*board_h

    pattern_points = np.zeros( (np.prod(board_sz), 3), np.float32 )
    pattern_points[:,:2] = np.indices(board_sz).T.reshape(-1, 2)  
    pattern_points *= args.size

    axis = np.float32([[args.size,0,0], [0,args.size,0], [0,0,-args.size]]).reshape(-1,3)

    cap = cv2.VideoCapture(0)
    cap.set(3,1280)
    cap.set(4,720)

    k = 0
    boards_found = 0

    img_points = None
    rotations = np.zeros((numBoards,3))
    translations = np.zeros((numBoards,3))

    print(rotations)

    # Find chessboard
    while boards_found<numBoards:
        ret, img = cap.read()
        k = cv2.waitKey(1)
        grey = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(img, board_sz)

        if found:
            term = ( cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1 )
            cv2.cornerSubPix(grey, corners, (11, 11), (-1, -1), term)
            cv2.drawChessboardCorners(img, board_sz, corners, found)
            if k & 0xFF == ord('q'):
                print('Registration cancelled')
                quit()
            if k & 0xFF == ord(' '):
                # Find the rotation and translation vectors.
                rots, trans, inliers = cv2.solvePnPRansac(pattern_points, corners, camMatrix, camDistort)
                rotations[boards_found] = rots.ravel()
                translations[boards_found] = trans.ravel()
                boards_found += 1
                print(boards_found)
        cv2.imshow("image", img)

    # project 3D points to image plane
    rvecs = np.mean(rotations,axis=0)
    tvecs = np.mean(translations,axis=0)
    quaternion = tf.transformations.quaternion_from_euler(rvecs[0],rvecs[1],rvecs[2])
    print('rotation:',rvecs)
    print('translation:',tvecs)
    imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, camMatrix, camDistort)
    img_points = corners.reshape(-1, 2)
    k = cv2.waitKey(1)

    print (rvecs,tvecs)


    #poseUpdater = _PoseUpdater()

    rospy.init_node('daVinci_listener', anonymous=True)
    # Initialize the subscriber
    #rospy.Subscriber("dvrk_psm/cartesian_pose_current", PoseStamped , poseUpdater.updateCB)

    rate = rospy.Rate(10) # 10hz

    touched = 0;

    poses = np.empty((board_n,3))
    #'''
    
    listener = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()
    listener.waitForTransform('/world', '/two_tool_wrist_caudier_link_2_left',
                              rospy.Time(0), rospy.Duration(3.0))
    def getDaVinciPose():
        t = rospy.Time.now()
        broadcaster.sendTransform((0,0.025,0),(0,0,0,1),t,
                                   '/dvrk_registration',
                                   '/two_tool_wrist_caudier_link_2_left')
        listener.waitForTransform('/world', '/dvrk_registration',
                              t, rospy.Duration(3.0))
        return listener.lookupTransform('/world', '/dvrk_registration', t)

    colTouches = np.zeros((board_w,3))
    rowTouches = np.zeros((board_h,3))
    #print(trans,rot)
    colsTouched = 0
    while colsTouched<board_w:
        index = colsTouched
        ret, img = cap.read()
        _draw(img,corners,imgpts)
        corner = (corners[index][0][0],corners[index][0][1])
        cv2.circle(img, corner, 10, (255,0,0),2)
        cv2.imshow("image", img)
        k = cv2.waitKey(1)
        if k & 0xFF == ord('q'):
            print('Registration cancelled')
            quit()
        if k & 0xFF == ord(' '):
            (trans,rot) = getDaVinciPose()
            colTouches[colsTouched]=np.array(trans)
            colsTouched += 1

    for touch in colTouches:
        print(np.linalg.norm(colTouches[0]-touch))

    rowsTouched=0
    while rowsTouched < board_h:
        index = rowsTouched*board_w
        ret, img = cap.read()
        _draw(img,corners,imgpts)
        corner = (corners[index][0][0],corners[index][0][1])
        cv2.circle(img, corner, 10, (255,0,0),2)
        cv2.imshow("image", img)
        k = cv2.waitKey(1)
        if k & 0xFF == ord('q'):
            print('Registration cancelled')
            quit()
        if k & 0xFF == ord(' '):
            (trans,rot) = getDaVinciPose()
            rowTouches[rowsTouched]=trans
            rowsTouched += 1

    center = colTouches[0]+rowTouches[0]
    center = center/2

    zAxis = axisFromPoints(colTouches,center)
    yAxis = axisFromPoints(rowTouches,center)

    rotMatrix = np.zeros((3,3))
    rotMatrix[:,2] = zAxis
    rotMatrix[:,0] = normalized(np.cross(zAxis,yAxis))
    rotMatrix[:,1] = normalized(np.cross(zAxis,rotMatrix[:,0]))
    rotation = tf.transformations.quaternion_from_matrix(rotMatrix)

    broadcaster.sendTransform(center,rotation,t, '/chessboard', '/world')

# Important frames:
# two_tool_wrist_caudier_link_2_left -> start of left pincer of claw
# two_tool_wrist_caudier_link_2_right  -> start of right pincer of claw
# dvrk_mtm/phantom exists with parent left_top_panel -> phantom object
if __name__ == '__main__':
    registerDaVinci()

'''
((-0.5451212838991569, -0.5058970185847791, 0.6325751091590994), (0.8759879898029738, -0.07719158340663901, -0.002464589990055419, -0.4761096795574546))
((-0.5403361228519282, -0.48245861604227236, 0.6314906528164369), (-0.8513704080161877, 0.08426415295080071, -0.010699040202976295, 0.5176422620116807))
((-0.5374248899947152, -0.4588927418929035, 0.6309259799621545), (-0.8197341245173626, 0.09224662013808642, -0.019888855753932046, 0.5649167722697316))
((-0.5333222533663377, -0.43567159979266373, 0.6302806731976667), (-0.782304633732002, 0.09723031559354628, -0.031831415451887744, 0.6144367231549128))
((-0.5293650510975042, -0.41213322561565346, 0.6290560839928268), (-0.7386111929101026, 0.1018343009230659, -0.042408443575926, 0.6650449644775105))
((-0.5248713212851487, -0.39027150160572344, 0.6282939457031567), (-0.6942223525741121, 0.10323792201532636, -0.05262628762663344, 0.710371543978967))
'''