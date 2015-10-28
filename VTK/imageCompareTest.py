#!/usr/bin/env python
 
# This simple example shows how to do basic rendering and pipeline
# creation.
 
import vtk
from vtk.util.numpy_support import vtk_to_numpy
import amoeba

filename = "teapot.stl"
 
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
cylinderActor.SetOrientation(30,0,10)
 
# Create the graphics structure. The renderer renders into the render
# window. The render window interactor captures mouse events and will
# perform appropriate camera or actor manipulation depending on the
# nature of the events.
ren = vtk.vtkRenderer()
renWin = vtk.vtkRenderWindow()
renWin.AddRenderer(ren)
iren = vtk.vtkRenderWindowInteractor()
iren.SetRenderWindow(renWin)
 
 # Add the actors to the renderer, set the background and size
ren.AddActor(cylinderActor)
ren.SetBackground(1, 1, 1)
renWin.SetSize(800, 800)
 
# This allows the interactor to initalize itself. It has to be
# called before an event loop.
iren.Initialize()
 
# We'll zoom in a little by accessing the camera and invoking a "Zoom"
# method on it.
ren.ResetCamera()
ren.GetActiveCamera().Zoom(2)
renWin.Render()
iren.Start()

# Turn render window into image
original_im = vtk.vtkWindowToImageFilter()
original_im.SetInput(renWin)
original_im.Update()
original_image = original_im.GetOutput()

print cylinderActor.GetOrientation();

cylinderActor.SetOrientation(0.0,0.0,0.0)

def afunc(var,data=None): 
	cylinderActor.SetOrientation(var[0],var[1],var[2])
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

amoeba.amoeba([0.0,0.0,0.0],[10,10,10],afunc)

print cylinderActor.GetOrientation();

 
