#!/usr/bin/env python
 
# This simple example shows how to do basic rendering and pipeline
# creation.
 
import vtk
from vtk.util.numpy_support import vtk_to_numpy
from math import pi,atan2
import amoebaAnnealed

filename = "suzanne.stl"
 
reader = vtk.vtkSTLReader()
reader.SetFileName(filename)
 
mapper = vtk.vtkPolyDataMapper()
if vtk.VTK_MAJOR_VERSION <= 5:
    mapper.SetInput(reader.GetOutput())
else:
    mapper.SetInputConnection(reader.GetOutputPort())
 
cylinderActor = vtk.vtkActor()
cylinderActor.SetMapper(mapper)
cylinderActor.GetProperty().SetColor(0,0,0)
#cylinderActor.SetPosition(30,-60,0)
 
# Create the graphics structure. The renderer renders into the render
# window. The render window interactor captures mouse events and will
# perform appropriate camera or actor manipulation depending on the
# nature of the events.
ren = vtk.vtkRenderer()
renWin = vtk.vtkRenderWindow()
renWin.SetOffScreenRendering(1)
renWin.AddRenderer(ren)
iren = vtk.vtkRenderWindowInteractor()
iren.SetRenderWindow(renWin)
 
 # Add the actors to the renderer, set the background and size
ren.AddActor(cylinderActor)
cylinderActor.SetPosition(0,0,0)
cylinderActor.SetOrientation(190,50,10)
cylinderActor.SetPosition(10,30,50)
ren.SetBackground(1, 1, 1)
renWin.SetSize(640*2, 480*2)
 
# This allows the interactor to initalize itself. It has to be
# called before an event loop.
iren.Initialize()
 
# We'll zoom in a little by accessing the camera and invoking a "Zoom"
# method on it.
ren.ResetCamera()
#ren.GetActiveCamera().SetPosition(0,0,200)
#ren.GetActiveCamera().SetFocalPoint(0,0,0)

# Apply camera View Angle

# ./calibration -w 6 -h 9 -s 18.44

CamMatrix = [ 1.0050698029341766e+03, 0., 3.1856271152098861e+02, 0.,
       1.0083493367812179e+03, 2.0815656163588574e+02, 0., 0., 1. ]

focalLengthX = CamMatrix[0]
focalLengthY = CamMatrix[4]
principalPointX = CamMatrix[2]
principalPointY = CamMatrix[5]

image_width = 640
image_height = 480
window_height = renWin.GetSize()[0]
window_width = renWin.GetSize()[1]
if(window_height != image_height):
	factor = window_height/image_height
	focalLengthY = CamMatrix[4] * factor

viewAngle = 2 * atan2( ( window_height / 2 ) , focalLengthY ) * 180 / pi
ren.GetActiveCamera().SetViewAngle(viewAngle)

# set Window Center 
px = 0;
width = 0;

py = 0;
height = 0;

if( image_width != window_width | image_height != window_height ) :

	factor = window_height/image_height

	px = factor * principalPointX
	width = window_width
	expectedWindowSize = round(factor * image_width)

	if( expectedWindowSize != window_width ) :
		diffX = (window_width - expectedWindowSize) / 2;
		px = px + diffX;

	py = factor * principalPointY
	height = window_height

else :

	px = principalPointX
	width = image_width;

	py = principalPointY
	height = image_height;


cx = width - px
cy = py

windowCenterX = cx / ( ( width-1)/2 ) - 1
windowCenterY = cy / ( ( height-1)/2 ) - 1

ren.GetActiveCamera().SetWindowCenter(windowCenterX, windowCenterY)
ren.ResetCameraClippingRange();

renWin.Render()
#iren.Start()

# Turn render window into image
original_im = vtk.vtkWindowToImageFilter()
original_im.SetInput(renWin)
original_im.Update()
original_image = original_im.GetOutput()

print cylinderActor.GetOrientation();

cylinderActor.SetOrientation(0,0.0,0.0)

def afunc(var,data=None): 
	cylinderActor.SetOrientation(var[0],var[1],var[2])
	cylinderActor.SetPosition(var[3], var[4], var[5])
	renWin.Render()

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

iterations = 3
base = 50

for idx in range(0,iterations) :

	amoebaAnnealed.amoeba([cylinderActor.GetOrientation()[0],
						   cylinderActor.GetOrientation()[1],
						   cylinderActor.GetOrientation()[2],
						   cylinderActor.GetPosition()[0],
						   cylinderActor.GetPosition()[1],
						   cylinderActor.GetPosition()[2]],
						   [base/(2.0**idx)]*6,afunc)

	print cylinderActor.GetOrientation();

print ren.GetActiveCamera().GetOrientation();

print cylinderActor.GetOrientation();

 
