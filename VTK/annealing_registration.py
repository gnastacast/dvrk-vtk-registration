from simanneal import Annealer
import random
import vtk

'''
PLAN:

ROS NODE - SERVICE

manual_registration.py
TAKES AN IMAGE AND RETURNS A TRANSFORMATION MATRIX

annealing_registration.py
RECEIVES A TRANSFORMATION MATRIX AND AN IMAGE AND RETURNS A REFINED TRANSFORMATION MATRIX

'''

def register(stlPath, startMatrix=None, imagePath=None, camera=None):
	
	# Read in STL file
	reader = vtk.vtkSTLReader()
	reader.SetFileName(stlPath)
	mapper = vtk.vtkPolyDataMapper()
	if vtk.VTK_MAJOR_VERSION <= 5:
		mapper.SetInput(reader.GetOutput())
	else:
		mapper.SetInputConnection(reader.GetOutputPort())

	# Create the actor we will be registering
	mainActor = vtk.vtkActor()
	mainActor.SetMapper(mapper)
	mainActor.GetProperty().SetColor(0,0,0)
	mainActor.SetPosition(0,20,-200)
	mainActor.SetOrientation(50,0,0)
	
	# Setup virtual camera
	camStatic = vtk.vtkCamera()
	if camera != None :
		camStatic.SetFocalPoint(camera.GetFocalPoint())
		camStatic.SetPosition(camera.GetPosition())
		camStatic.SetViewUp(camera.GetViewUp())
	else :
		#Setup default camera parameters
		camStatic.SetFocalPoint(0,0,0)
		camStatic.SetPosition(0,0,200)
		camStatic.SetViewUp(0,1,0)

	# Create render window
	renWin = vtk.vtkRenderWindow()

	# Create renderers
	numViews = 1
	eyeDistance = 0
	for i in range(numViews):
		ren = vtk.vtkRenderer()
		cam = vtk.vtkCamera()
		cam.SetFocalPoint(camStatic.GetFocalPoint()[0]+eyeDistance*(i-.5),
						  camStatic.GetFocalPoint()[1],
						  camStatic.GetFocalPoint()[2])
		cam.SetPosition(camStatic.GetPosition()[0]+eyeDistance*(i-.5),
						camStatic.GetPosition()[1],
						camStatic.GetPosition()[2])
		renWin.AddRenderer(ren)
		ren.SetActiveCamera(cam)
		ren.SetViewport(i*(1/numViews),0,(i+1)*(1/numViews),1)
		ren.SetBackground(1,1,1)
		renWin.SetOffScreenRendering(True)
		renWin.AddRenderer(ren)
		ren.AddActor(mainActor)

	#Accquire image to register to
	if imagePath != None :
		original_image = loadImage(imagePath)
		renWin.SetSize(original_image.GetDimensions())
	else :
		renWin.SetSize(640/2*numViews,480/2)
		# Randomly place main actor
		mainActor.SetPosition(random.random()*100-100,
							  random.random()*100-100,
							  random.random()*200-200)
		mainActor.SetOrientation(random.random()*360-180,
								 random.random()*360-180,
								 random.random()*360-180)
		renWin.Render()
		# Turn render window into image
		original_im = vtk.vtkWindowToImageFilter()
		original_im.SetInput(renWin)
		original_im.Update()
		original_image = original_im.GetOutput()

	if startMatrix == None:
		state = [0,0,0,0,0,0]
	else :
		tempTransform = vtk.vtkTransform()
		tempTransform.SetMatrix(startMatrix)
		state = []
		state.DeepCopy(tempTransform.GetPosition()+tempTransform.GetOrientation())

	# Perform simulated annealing
	tsp = AnnealingRegistration(state,mainActor,renWin,original_image)		
	tsp.copy_strategy = "slice"  
	state, e = tsp.anneal()

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

class AnnealingRegistration(Annealer):

	# pass extra data (for rendering and comparing) into the constructor
	def __init__(self, state, actor, renderer, image):
		self.actor = actor
		self.renderer = renderer
		self.image = image
		self.Tmax = 350
		self.Tmin = .1
		self.steps = 5000
		self.save_state_on_exit = False
		super(AnnealingRegistration, self).__init__(state)  # important! 

	# Rotates and translates randomly based on temperature
	def move(self):
		for idx in range(3) :
			self.state[idx] += (random.random()-.5)*self.T
			self.state[idx] = (self.state[idx]+180)%360 -180
			self.state[idx+3] += (random.random()-.5)*self.T**.5

	#Calculates difference between images
	def energy(self):
		self.actor.SetOrientation(self.state[0],self.state[1],self.state[2])
		self.actor.SetPosition(self.state[3],self.state[4],self.state[5])
		self.renderer.Render()

		new_im = vtk.vtkWindowToImageFilter()
		new_im.SetInput(self.renderer)
		new_im.Update()

		# Compare images
		idiff = vtk.vtkImageDifference()
		idiff.SetInputConnection(new_im.GetOutputPort())
		idiff.SetImage(self.image)
		idiff.Update()
		e = idiff.GetThresholdedError()
		return e

if __name__ == "__main__":
	register('suzanne.stl')

