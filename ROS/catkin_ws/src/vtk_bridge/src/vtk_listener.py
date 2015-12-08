#!/usr/bin/env python
from __future__ import print_function

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import time

import vtk
from vtk.util.numpy_support import vtk_to_numpy

import vtk

oldPos = []
 
class vtkTimerCallback():
    def __init__(self):
        self.timer_count = 0

    def execute(self,obj,event):
        global oldPos
        iren = obj
        self.timer_count += 1
        rospy.spinOnce()
        if len(oldPos)>0:
            print("SET POSE")
            #cylinderActor.SetPosition(oldPos[0],oldPos[1],oldPos[2])            
            iren.GetRenderWindow().Render()


def callback(data):
    global oldPos
    global actor
    if(oldPos != data.position):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.position)
        oldPos = data.position
        actor.SetOrientation(oldPos[3],oldPos[4],oldPos[5])
    
def vtk_listener():
    global renWin
    global oldPos
    global cylinderActor
    global ren
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('vtk_listener', anonymous=True)

    rospy.Subscriber("joint_states", JointState , callback)
    #Create a sphere
    sphereSource = vtk.vtkSphereSource()
    sphereSource.SetCenter(0.0, 0.0, 0.0)
    sphereSource.SetRadius(5)

    #Create a mapper and actor
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(sphereSource.GetOutputPort())
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    prop = actor.GetProperty()

    # Setup a renderer, render window, and interactor
    renderer = vtk.vtkRenderer()
    renderWindow = vtk.vtkRenderWindow()
    #renderWindow.SetWindowName("Test")

    renderWindow.AddRenderer(renderer);
    renderWindowInteractor = vtk.vtkRenderWindowInteractor()
    renderWindowInteractor.SetRenderWindow(renderWindow)

    #Add the actor to the scene
    renderer.AddActor(actor)
    renderer.SetBackground(1,1,1) # Background color white

    #Render and interact
    renderWindow.Render()

    # Initialize must be called prior to creating timer events.
    renderWindowInteractor.Initialize()

    # Sign up to receive TimerEvent
    cb = vtkTimerCallback()
    cb.actor = actor
    renderWindowInteractor.AddObserver('TimerEvent', cb.execute)
    timerId = renderWindowInteractor.CreateRepeatingTimer(100);
    #start the interaction and timer
    renderWindowInteractor.Start()
    # spin() simply keeps python from exiting until this node is stopped
    


if __name__ == '__main__':
    vtk_listener()