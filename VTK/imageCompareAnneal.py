from __future__ import print_function
import math
import random
from simanneal import Annealer
import pickle

import vtk
from vtk.util.numpy_support import vtk_to_numpy

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
 
# Create the graphics structure. The renderer renders into the render
# window.
ren = vtk.vtkRenderer()
renWin = vtk.vtkRenderWindow()
#renWin.SetOffScreenRendering(True)
renWin.AddRenderer(ren)
 
 # Add the actors to the renderer, set the background and size
ren.AddActor(cylinderActor)
cylinderActor.SetPosition(0,0,0)
cylinderActor.SetOrientation(190,50,10)
ren.SetBackground(1, 1, 1)

windowScale = 1;

renWin.SetSize(640/2, 480/2)
 
# We'll zoom in a little by accessing the camera and invoking a "Zoom"
# method on it.
ren.GetActiveCamera().SetPosition(0,0,200)

renWin.Render()
#iren.Start()

# Turn render window into image
original_im = vtk.vtkWindowToImageFilter()
original_im.SetInput(renWin)
original_im.Update()
original_image = original_im.GetOutput()

print("Actual orientation")
print(cylinderActor.GetOrientation())

class AnnealingRegistration(Annealer):
	
	# pass extra data (the distance matrix) into the constructor
	def __init__(self, state):
		self.Tmax = 340
		self.Tmin = .1
		self.steps = 2000
		self.save_state_on_exit = False
		super(AnnealingRegistration, self).__init__(state)  # important! 

	#Swaps two cities in the route
	def move(self):
		dist = self.T
		for idx in range(len(self.state)) :
			self.state[idx] += (random.random()-.5)*dist
			self.state[idx] = (self.state[idx]+180)%360 -180
	#Calculates the length of the route."""
	def energy(self):
		cylinderActor.SetOrientation(self.state[0],self.state[1],self.state[2])
		renWin.Render()

		new_im = vtk.vtkWindowToImageFilter()
		new_im.SetInput(renWin)
		new_im.Update()


		# Compare images
		idiff = vtk.vtkImageDifference()
		idiff.SetInputConnection(new_im.GetOutputPort())
		idiff.SetImage(original_image)
		idiff.Update()
		e = idiff.GetThresholdedError()
		return e

if __name__ == '__main__':
	for idx in range(0,1000)
	init_state = [0,0,0]
	tsp = AnnealingRegistration(init_state)
	tsp.copy_strategy = "slice"  
	state, e = tsp.anneal()
	print(state)