#!/usr/bin/env python
from __future__ import print_function
import rospy
import tf2_ros
import geometry_msgs.msg
import numpy as np
import cv2
import argparse
import copy
import codecs
import json
import rosbag
from tf import transformations


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
def _drawAxes(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)

def _axisFromPoints(points,center):
    axis = np.zeros(3)
    for i in range(1,len(points)):
        vec = points[i] - center
        vec = _normalized(vec)
        axis += vec
    axis = _normalized(axis)
    return axis

def _normalized(vec):
    ''' Normalizes a vector'''
    mag = np.linalg.norm(vec)
    return vec/mag

def _getDVRKPos(robotName, broadcaster, tfBuffer):
    parentFrame = robotName + '_tool_wrist_sca_shaft_link'
    timeStamp = rospy.Time.now()
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = parentFrame
    t.child_frame_id = "dvrk_registration"
    t.header.stamp = timeStamp
    t.transform.translation.x = 0
    t.transform.translation.y = 0.0254 * 0.39 #From wrist shaft to tip
    t.transform.translation.z = 0
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0
    broadcaster.sendTransform(t)
    trans = tfBuffer.lookup_transform('world', 'dvrk_registration',
                                      timeStamp, rospy.Duration(3.0))
    return trans.transform.translation

def _makeTransformMsg(translation, quaternion, name, parent):
    msg = geometry_msgs.msg.TransformStamped()
    msg.header.frame_id = parent
    msg.child_frame_id = name
    msg.transform.translation.x = translation[0]
    msg.transform.translation.y = translation[1]
    msg.transform.translation.z = translation[2]
    msg.transform.rotation.x = quaternion[0]
    msg.transform.rotation.y = quaternion[1]
    msg.transform.rotation.z = quaternion[2]
    msg.transform.rotation.w = quaternion[3]
    return msg

def _getChessboardTransform(capture, board_sz, pattern, camMatrix, camDistort):
    numBoards = 5
    k = 0
    boards_found = 0
    rotations = np.zeros((numBoards,3))
    translations = np.zeros((numBoards,3))
    started = False
    # Find chessboard
    while boards_found < numBoards and  not rospy.is_shutdown():
        ret, img = capture.read()
        img = cv2.flip(img,-1)
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
                started = True;
            #if k & 0xFF == ord(' '):
            # Find the rotation and translation vectors.
            rots, trans, inliers = cv2.solvePnPRansac(pattern, corners, camMatrix, camDistort)
            rotations[boards_found] = rots.ravel()
            translations[boards_found] = trans.ravel()
            if started:
                boards_found += 1
        cv2.imshow('image', img)

    # return rotation and translation
    rvecs = np.mean(rotations,axis=0)
    tvecs = np.mean(translations,axis=0)
    return rvecs, tvecs, corners

def registerDVRK(slave, boardW, boardH, squareSize, camMatrix, camDistort):
    # Initialize ROS node
    rospy.init_node('dvrk_registration', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # Initialize TF listener
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    broadcaster = tf2_ros.TransformBroadcaster()

    # Setup video capture
    cap = cv2.VideoCapture(0)
    cap.set(3,1280)
    cap.set(4,720)

    # Create parameters for chessboard
    board_sz = (boardW , boardH)
    board_n = boardW * boardH
    pattern = np.zeros( (np.prod(board_sz), 3), np.float32 )
    pattern[:,:2] = np.indices(board_sz).T.reshape(-1, 2)
    pattern *= squareSize
    # Find chessboard using OpenCV
    rot, trans, corners = _getChessboardTransform(cap, board_sz, pattern,
                                         camMatrix, camDistort)
    # Make chessboard transformation matrix
    camToChessboard = transformations.compose_matrix(angles = rot, 
                                                     translate = trans);
    chessboardToCam = np.linalg.inv(camToChessboard)

    # Create a 3D axis for visualizing chessboard transform
    axis = np.float32([[squareSize,0,0], [0,squareSize,0], [0,0,-squareSize]])
    axis = axis.reshape(-1,3)
    imgpts, jac = cv2.projectPoints(axis, rot, trans, camMatrix, camDistort)

    # Record touches in DVRK frame in order to register chessboard
    touched = 0;
    poses = np.empty((board_n,3))
    numCorners = 4;
    colTouches = np.zeros((numCorners,3))
    rowTouches = np.zeros((numCorners,3))
    colsTouched = 0

    while colsTouched < numCorners and not rospy.is_shutdown():
        if(slave == 'PSM2'):
            index = colsTouched
        else :
            index = boardW * rowsTouched - colsTouched - 1
        ret, img = cap.read()
        img = cv2.flip(img,-1)
        _drawAxes(img,corners,imgpts)
        corner = (corners[index][0][0],corners[index][0][1])
        cv2.circle(img, corner, 10, (255,0,0),2)
        cv2.imshow('image', img)
        k = cv2.waitKey(1)
        if k & 0xFF == ord('q'):
            print('Registration cancelled')
            quit()
        if k & 0xFF == ord(' '):
            trans = _getDVRKPos(slave, broadcaster, tfBuffer)
            colTouches[colsTouched] = np.array((trans.x, trans.y, trans.z))
            colsTouched += 1

    rowsTouched=0
    while rowsTouched < numCorners and  not rospy.is_shutdown():
        if(slave == 'PSM2'):
            index = boardW * rowsTouched
        else :
            index = boardW * rowsTouched + boardH - 1
        ret, img = cap.read()
        img = cv2.flip(img,-1)
        _drawAxes(img,corners,imgpts)
        corner = (corners[index][0][0],corners[index][0][1])
        cv2.circle(img, corner, 10, (255,0,0),2)
        cv2.imshow('image', img)
        k = cv2.waitKey(1)
        if k & 0xFF == ord('q'):
            print('Registration cancelled')
            quit()
        if k & 0xFF == ord(' '):
            trans = _getDVRKPos(slave, broadcaster, tfBuffer)
            rowTouches[rowsTouched] = np.array((trans.x, trans.y, trans.z))
            rowsTouched += 1
            
    translation = colTouches[0]+rowTouches[0]
    translation = translation/2
    xAxis = _axisFromPoints(colTouches,translation)
    yAxis = _axisFromPoints(rowTouches,translation)
    rotMatrix = np.identity(4)
    rotMatrix[0:3,0] = xAxis
    rotMatrix[0:3,2] = _normalized(np.cross(xAxis,yAxis))
    rotMatrix[0:3,1] = _normalized(np.cross(xAxis,rotMatrix[0:3,2]))
    rotMatrix[0:3,0] = _normalized(np.cross(rotMatrix[0:3,1],rotMatrix[0:3,2]))
    rotation = transformations.quaternion_from_matrix(rotMatrix)
    tChessboardBase = _makeTransformMsg(translation, rotation, 
                                        "chessboardBase", "world")

    rotation = transformations.quaternion_from_matrix(chessboardToCam)
    tChessboard = _makeTransformMsg(np.array([0,0,0]), transformations.quaternion_from_euler(np.pi,np.pi/2.0,0),
                                "chessboard", "chessboardBase")
    translation = chessboardToCam[0:3,3].copy();
    translation[0] *= -1;
    translation[1] *= -1
    tCameraBase = _makeTransformMsg(translation, rotation, 
                                "cameraBase", "chessboardBase")
    tCamera = _makeTransformMsg(np.array([0,0,0]), transformations.quaternion_from_euler(-np.pi/2,np.pi,0),
                                "camera", "cameraBase")
    if not rospy.is_shutdown():
        
        timeStamp = rospy.Time.now()
        tChessboardBase.header.stamp = timeStamp
        tCameraBase.header.stamp = timeStamp
        broadcaster.sendTransform(tChessboardBase)
        tChessboard.header.stamp = timeStamp
        broadcaster.sendTransform(tChessboard)
        broadcaster.sendTransform(tCameraBase)
        tCamera.header.stamp = timeStamp
        broadcaster.sendTransform(tCamera)
        bag = rosbag.Bag('dvrkRegistration.bag', 'w')
        trans = tfBuffer.lookup_transform('world', 'chessboard',
                                          tChessboard.header.stamp, rospy.Duration(3.0))
        bag.write(slave + "_to_chessboard", trans)
        trans = tfBuffer.lookup_transform('world', 'camera',
                                          tChessboard.header.stamp, rospy.Duration(3.0))
        bag.write(slave + "_to_camera", trans)

        bag.close()
    
        while not rospy.is_shutdown():
            trans = tfBuffer.lookup_transform('camera', slave + '_tool_wrist_sca_shaft_link',
                                               rospy.Time(0), rospy.Duration(3.0))
            transCam = trans.transform.translation;
            pt = np.zeros((1,3))
            pt[0,0] = transCam.x
            pt[0,1] = transCam.y
            pt[0,2] = transCam.z
            imgpts, jac = cv2.projectPoints(pt, rvecs, tvecs, camMatrix, camDistort)
            ret, img = cap.read()
            img = cv2.flip(img,-1)
            cv2.circle(img,imgpts,(0,0,255))
    
    # Close openCV window
    cv2.destroyWindow('image')
    cv2.waitKey(-1)
    cv2.imshow('image', img)

if __name__ == '__main__':
    import os
    filepath = os.path.dirname(os.path.realpath(__file__))

    parser = argparse.ArgumentParser(description='Calibrate two stereo cameras. Press space bar to capture a chessboard')
    parser.add_argument('-s','--size', help = 'Size of one square on the board. Default is 0.0254 meters (one inch)',
                        type = float, default = 0.0254)
    parser.add_argument('--width', help='Number of inner corners along the width of board. Default is 8',
                        type = int, default = 9)
    parser.add_argument('--height', help='Number of inner corners along the height of board. Default is 6',
                        type = int, default = 6)
    parser.add_argument('-c', '--calibration', help='json file containing camera calibration from openCV',
                        type = str, default = filepath+'/../launch/single_cam_calibration.yaml')
    args, unknown = parser.parse_known_args()

    board_w = args.width
    board_h = args.height

    objText = codecs.open(args.calibration, 'r', encoding='utf-8').read()
    camParams = json.loads(objText)
    camMatrix = np.matrix(camParams['camera_matrix'])
    camDistort = np.matrix(camParams['dist_coeff'])
    registerDVRK('PSM2', board_w, board_h, args.size, camMatrix, camDistort)
