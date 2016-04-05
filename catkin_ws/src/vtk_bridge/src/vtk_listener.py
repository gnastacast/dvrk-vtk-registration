#!/usr/bin/env python
from __future__ import print_function

import rospy
from sensor_msgs.msg import JointState

import vtk
from vtk.util.numpy_support import vtk_to_numpy

import vtk

# Global actor variable that will be passed between functions
actor = vtk.vtkActor()
 
class vtkTimerCallback():
    def __init__(self):
        pass
    def execute(self,obj,event):
        # We use a short sleep to update the rospy thread
        rospy.sleep(.001)
        iren = obj
        iren.GetRenderWindow().Render()

def subscribeCB(data):
    global actor
    oldPos = data.position
    actor.SetPosition(oldPos[0],oldPos[1],oldPos[2])
    actor.SetOrientation(oldPos[3],oldPos[4],oldPos[5])
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.position)

def vtk_listener():
    global actor
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('vtk_listener', anonymous=True)

    # Initialize the subscriber
    rospy.Subscriber("joint_states", JointState , subscribeCB)

    #Create a sphere
    sphereSource = vtk.vtkSphereSource()
    sphereSource.SetCenter(0.0, 0.0, 0.0)
    sphereSource.SetRadius(5)

    #Create a mapper and actor
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(sphereSource.GetOutputPort())
    actor.SetMapper(mapper)

    # Setup a renderer, render window, and interactor
    renderer = vtk.vtkRenderer()
    renderWindow = vtk.vtkRenderWindow()

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
    renderWindowInteractor.AddObserver('TimerEvent', cb.execute)
    timerId = renderWindowInteractor.CreateRepeatingTimer(50);
    #start the interaction and timer
    renderWindowInteractor.Start()
    
if __name__ == '__main__':
    vtk_listener()