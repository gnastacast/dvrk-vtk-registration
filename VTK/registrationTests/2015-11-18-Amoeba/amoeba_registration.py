#!/usr/bin/env python
 
# This simple example shows how to do basic rendering and pipeline
# creation.
 
import vtk
from vtk.util.numpy_support import vtk_to_numpy
from amoebaAnnealed3 import amoeba
import os
import time

def iterateStates(filepath):
	with open(filepath,"r") as fh:
		l = [ map(str,line.split('\n')) for line in fh]
	for idx in range(0,len(l)):
		s = map(float, l[idx][0].split(' '))
		outputTransform = vtk.vtkTransform()
		outputTransform.PostMultiply()
		outputTransform.RotateY(s[4])
		outputTransform.RotateX(s[3])
		outputTransform.RotateZ(s[5])
		outputTransform.Translate(s[0],s[1],s[2])
		l[idx] = outputTransform;
	return l

def iterateTransforms(transforms, filepath) :
	text = ""
	for idx in range(0,len(transforms)) :
		# Turn transform into state vector
		pos = transforms[idx].GetPosition()
		rot = transforms[idx].GetOrientation()
		s = [pos[0],pos[1],pos[2],rot[0],rot[1],rot[2]]
		for stateIdx in range(0,len(s)-1) :
			text += str(s[stateIdx]) + " "
		text += str(s[len(s)-1])+'\n'
	print("Saving state to: %s" % filepath)
	with open(filepath, "w") as fh:
		fh.write(text)

def loadImage(filename) :
	''' Takes a path to a .png image as a string and
		returns a vtkImageData object
	'''
	imageData = vtk.vtkImageData()
	pngReader = vtk.vtkPNGReader()
	if(not(pngReader.CanReadFile(filename))):
		print("Error reading .png file "+filename)
		return
	pngReader.SetFileName(filename)
	pngReader.Update()
	imageData = pngReader.GetOutput()
	return imageData

def afunc(var,data=None): 
	cylinderActor.SetPosition(var[0],var[1],var[2])
	cylinderActor.SetOrientation(var[3],var[4],var[5])
	renWin.Render()
	renWin.SetOffScreenRendering(True)

	new_im = vtk.vtkWindowToImageFilter()
	new_im.SetInput(renWin)
	new_im.Update()


	# Compare images
	idiff = vtk.vtkImageDifference()
	idiff.SetInputConnection(new_im.GetOutputPort())
	idiff.SetImage(original_image)
	idiff.Update()
	err = renWin.GetSize()[0]*renWin.GetSize()[1] - idiff.GetThresholdedError()

	return err

filename = "suzanne.stl"
 
reader = vtk.vtkSTLReader()
reader.SetFileName(filename)
 
mapper = vtk.vtkPolyDataMapper()
if vtk.VTK_MAJOR_VERSION <= 5:
    mapper.SetInput(reader.GetOutput())
else:
    mapper.SetInputConnection(reader.GetOutputPort())

manualRegs = iterateStates("registrationTests/reg_manual.txt")

cylinderActor = vtk.vtkActor()
cylinderActor.SetMapper(mapper)
cylinderActor.GetProperty().SetColor(0,0,0)

# Create the graphics structure. The renderer renders into the render
# window. The render window interactor captures mouse events and will
# perform appropriate camera or actor manipulation depending on the
# nature of the events.
ren = vtk.vtkRenderer()
renWin = vtk.vtkRenderWindow()
renWin.AddRenderer(ren)
 
 # Add the actors to the renderer, set the background and size
ren.AddActor(cylinderActor)
ren.SetBackground(1, 1, 1)
renWin.SetSize(640, 640)
#renWin.SetOffScreenRendering(True)

camera = vtk.vtkCamera()
camera.SetFocalPoint(0,0,0)
camera.SetPosition(0,0,200)
camera.SetViewUp(0,1,0)

ren.SetActiveCamera(camera)

amoebaRegs = []
annealedIterations = 50
text = ""

for idx in range(0,len(manualRegs)) :

	original_image = loadImage("registrationTests/bw_"+str(1000+idx)+".png")

	for innerIdx in range(0,annealedIterations) :
		print "Registration #"+str(idx)+" Iteration #"+str(innerIdx)
		pos = manualRegs[idx].GetPosition()
		rot = manualRegs[idx].GetOrientation()
		s0 = [pos[0],pos[1],pos[2],rot[0],rot[1],rot[2]]
		cylinderActor.SetPosition(pos)
		cylinderActor.SetOrientation(rot)
		startTime = time.clock()
		s,fvalue,iteration = amoeba(s0,[5]*len(s0),afunc,ftolerance=.0001,xtolerance=.0001,itmax=400)
		totalTime = time.clock()-startTime
		print("Time: " + str(totalTime))
		print("Error: " + str(renWin.GetSize()[0]*renWin.GetSize()[1]-fvalue) +" Iteration:" + str(iteration))

		for stateIdx in range(0,len(s)-1) :
			text += str(s[stateIdx]) + " "
		text += str(s[len(s)-1])+'\n'

filepath = "registrationTests/reg_anneal.txt"
print("Saving state to: %s" % filepath)
with open(filepath, "w") as fh:
	fh.write(text)

 
