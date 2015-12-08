from sys import exit
import vtk
import random
from simanneal import Annealer

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

class RegistrationObject(object) :

	ren = vtk.vtkRenderer()
	bgRen = vtk.vtkRenderer()
	renWin = vtk.vtkRenderWindow()
	mainActor = vtk.vtkActor()
	bgActor = vtk.vtkImageActor()
	camStatic = vtk.vtkCamera()
	colorImage = vtk.vtkImageData()
	bwImage = vtk.vtkImageData()
	iren = vtk.vtkRenderWindowInteractor()
	actualTransform = vtk.vtkTransform()


	def __init__(self, stlFile, camData=None, bwImage=None, colorImage=None):

		# Read in STL file
		reader = vtk.vtkSTLReader()
		reader.SetFileName(stlFile)
		mapper = vtk.vtkPolyDataMapper()
		if vtk.VTK_MAJOR_VERSION <= 5:
			mapper.SetInput(reader.GetOutput())
		else:
			mapper.SetInputConnection(reader.GetOutputPort())

		# Initialize the actor we will be registering
		self.mainActor.SetMapper(mapper)
		self.mainActor.GetProperty().SetColor(0,0,0)

		self.renWin.SetNumberOfLayers(2)

		# Setup foreground renderer
		self.renWin.AddRenderer(self.ren)
		self.ren.SetLayer(1)
		self.ren.AddActor(self.mainActor)
		self.bgRen.SetBackground(1,1,1)

		# Setup background renderer
		self.renWin.AddRenderer(self.bgRen)
		self.bgRen.SetLayer(0)
		self.bgRen.AddActor(self.bgActor)
		self.bgRen.InteractiveOff()
		self.bgRen.SetBackground(1,1,1)

		# Initialize the scene camera
		if camData != None:
			# TODO: Read in camera data
			self.camStatic.SetFocalPoint(0,0,0)
			self.camStatic.SetPosition(0,0,200)
			self.camStatic.SetViewUp(0,1,0)
		else:
			#Setup default camera parameters
			self.camStatic.SetFocalPoint(0,0,0)
			self.camStatic.SetPosition(0,0,200)
			self.camStatic.SetViewUp(0,1,0)
		self.ren.GetActiveCamera().SetFocalPoint(self.camStatic.GetFocalPoint())
		self.ren.GetActiveCamera().SetPosition(self.camStatic.GetPosition())
		self.ren.GetActiveCamera().SetViewUp(self.camStatic.GetViewUp())
		# Initialize interactor
		self.iren.SetRenderWindow(self.renWin)
		style = self._RegistrationInteractorStyle(self.camStatic)
		self.iren.SetInteractorStyle(style)
		self.iren.Initialize()
		# Load images from files
		if bwImage != None and colorImage != None:
			self.bwImage = loadImage(bwImage)
			self.colorImage = loadImage(colorImage)
			# Set render window size to match the image
			bwImageDims = self.bwImage.GetDimensions()
			colorImageDims = self.colorImage.GetDimensions()
			if bwImageDims != colorImageDims:
				print("Provided images have different dimensions")
				sys.exit()
			self.renWin.SetSize(bwImageDims[0], bwImageDims[1])
		# If no image exists, generate one
		else:
			print("Generating images color.png and bw.png")
			#self.bwImage,self.colorImage = self.generateImages(320,240)
			self.bwImage,self.colorImage = self.generateImages(640,480)
		
		#self.ren.AddActor(self.mainActor)
		#self.bgRen.AddActor(self.bgActor)

		#self.renWin.SetNumberOfLayers(1)


		# Setup background renderer
		#self.bgRen.SetLayer(0)
		#self.bgRen.InteractiveOff()
		#self.bgRen.SetBackground(1,1,1)
		

	def generateImages(self, xDim, yDim):
		''' Generates a random image to register to when none is available
			  Input:
			    xDim: width of image to generate in pixels.
			    yDim: height of image to generate in pixels.
			    imagePath: a string representing a file path to save the
			               generated image to
			  Optional input:
			    color: boolean representing whether the image will be in color
			           or a black and white silhouette
			  Output:
			    bg_image: vtkImageData object containing generated render
				This function also writes bg_image out to a png file
		'''
		self.renWin.SetSize(xDim,xDim)
		# Setup background
		self.bgRen.RemoveAllViewProps();
		self.bgRen.SetBackground(1,1,1);
		self.bgRen.Render();
		# Setup foreground
		self.ren.GetActiveCamera().SetClippingRange(.1, 1000)
		self.mainActor.GetProperty().SetRepresentationToSurface()
		# Randomly place main actor
		self.mainActor.SetPosition(random.random()*100-50,
								   random.random()*100-50,
								   random.random()*200-200)
		self.mainActor.SetOrientation(random.random()*360-180,
									  random.random()*360-180,
									  random.random()*360-180)
		print("Generating random registration image")
		print("  Actor position: " + str(self.mainActor.GetPosition()))
		print("  Actor rotation: "+ str(self.mainActor.GetOrientation()))
		print("")
		self.actualTransform = vtk.vtkTransform()
		self.actualTransform.SetMatrix(self.mainActor.GetMatrix())

		# Needed to stop bug where other parts of screen turn up in render
		self.renWin.SetOffScreenRendering(True)

		# Render black and white silhouette scene
		self.mainActor.GetProperty().SetColor(0,0,0)
		self.renWin.Render()
		
		# Turn render window into image
		bw_im = vtk.vtkWindowToImageFilter()
		bw_im.SetInput(self.renWin)
		bw_im.Update()
		bwImage = bw_im.GetOutput()
		# Write color image to file
		writer = vtk.vtkPNGWriter()
		writer.SetFileName('bw.png')
		writer.SetInput(bwImage)
		writer.Write()

		# Render color scene
		self.mainActor.GetProperty().SetColor(0,1,0)
		self.renWin.Render()
		# Turn render window into image
		color_im = vtk.vtkWindowToImageFilter()
		color_im.SetInput(self.renWin)
		color_im.Update()
		colorImage = color_im.GetOutput()
		# Write black and white silhouette image to file
		writer.SetFileName('color.png')
		writer.SetInput(colorImage)
		writer.Write()

		# Reset mainActor
		self.mainActor.SetPosition(0,0,0)
		self.mainActor.SetOrientation(0,0,0)
		
		return bwImage, colorImage
		

	def _makeImageBackground(self, image) :
		''' Input:
			  image (vtkImageData) = image to place in background
			Output (camera,imageActor):
			  camera (vtkCamera) = camera object for background renderer
			  imageActor (vtkImageActor) = plane textured with provided image
		'''
		imageActor = vtk.vtkImageActor()
		if vtk.VTK_MAJOR_VERSION <= 5 :
		  imageActor.SetInput(image);
		else :
		  imageActor.SetInputData(image);

		origin = image.GetOrigin()
		spacing = image.GetSpacing()
		extent = image.GetExtent()

		camera = vtk.vtkCamera()
		camera.ParallelProjectionOn()

		xc = origin[0] + 0.5*(extent[0] + extent[1])*spacing[0]
		yc = origin[1] + 0.5*(extent[2] + extent[3])*spacing[1]
		yd = (extent[3] - extent[2] + 1)*spacing[1];
		d = camera.GetDistance()
		camera.SetParallelScale(0.5*yd)
		camera.SetFocalPoint(xc,yc,0.0)
		camera.SetPosition(xc,yc,d);

		return camera, imageActor

	def manualRegistration(self, inputTransform=None):
		''' Function for automatic registration of an STL file against an image
			  Optional Input
				inputTransform (vtkTransform) = starting homogenous transform of
												mainActor object
			  Output:
				outputTransform (vtkTransform) = final homogenous transform of
				                            	 mainActor object
		'''

		print("Beginning manual registration")
		
		if inputTransform != None :
			# Set mainActor transform
			self.mainActor.SetPosition(inputMatrix.GetPosition())
			self.mainActor.SetOrientation(inputMatrix.GetOrientation())
		else :
			self.mainActor.SetPosition(0,0,0)
			self.mainActor.SetOrientation(0,0,0)
		
		# Setup background
		camera,self.bgActor = self._makeImageBackground(self.colorImage)
		# Setup camera
		self.bgActor.VisibilityOn()
		self.bgRen.AddActor(self.bgActor)
		self.bgRen.SetActiveCamera(camera)
		
		# Setup foreground
		self.ren.GetActiveCamera().SetFocalPoint(self.camStatic.GetFocalPoint())
		self.ren.GetActiveCamera().SetPosition(self.camStatic.GetPosition())
		self.ren.GetActiveCamera().SetViewUp(self.camStatic.GetViewUp())
		self.mainActor.GetProperty().SetColor(0,0,1)
		self.mainActor.GetProperty().SetOpacity(1);
		
		# Turn onscreen rendering back on
		self.renWin.SetOffScreenRendering(False)
		self.iren.Start()
		
		outputTransform = vtk.vtkTransform()
		outputTransform.SetMatrix(self.camStatic.GetViewTransformMatrix())
		outputTransform.Inverse()
		camMatrix = self.ren.GetActiveCamera().GetViewTransformMatrix()
		outputTransform.Concatenate(camMatrix)

		print("Registered position:")
		print("  " + str(outputTransform.GetPosition()))
		print("Registered rotation:")
		print("  " + str(outputTransform.GetOrientation()))

		return outputTransform
		
	def annealingRegistration(self, inputTransform=None):
		''' Function for automatic registration of an STL file against an image
			using simulated annealing
			  Optional Input
				inputTransform (vtkTransform) = starting homogenous transform of
												mainActor object
			  Output:
				outputTransform (vtkTransform) = final homogenous transform of
				                            	 mainActor object
		'''
		print("Beginning simulated annealing registration")
		self.renWin.SetOffScreenRendering(True)
		if inputTransform != None :
			# Set mainActor transform
			self.mainActor.SetOrientation(inputTransform.GetOrientation())
			self.mainActor.SetPosition(inputTransform.GetPosition())
		# Setup background
		self.bgActor.VisibilityOff()
		# Setup foreground
		self.ren.SetActiveCamera(vtk.vtkCamera())
		self.ren.GetActiveCamera().SetFocalPoint(self.camStatic.GetFocalPoint())
		self.ren.GetActiveCamera().SetPosition(self.camStatic.GetPosition())
		self.ren.GetActiveCamera().SetViewUp(self.camStatic.GetViewUp())
		self.ren.GetActiveCamera().SetClippingRange(.1,1000)

		self.mainActor.GetProperty().SetColor(0,0,0)
		self.mainActor.GetProperty().SetOpacity(1)
		self.mainActor.GetProperty().SetRepresentationToSurface()

		# Turn transform into state vector
		pos = self.mainActor.GetPosition()
		rot = self.mainActor.GetOrientation()
		s = [pos[0],pos[1],pos[2],rot[0],rot[1],rot[2]]

		# Perform simulated annealing
		tsp = _RegAnnealer(s,self.mainActor,self.renWin,self.bwImage)	
		tsp.Tmax = 15
		tsp.Tmin = .01
		tsp.steps= 2000	
		tsp.copy_strategy = "slice"  
		s, e = tsp.anneal()
		print(s)
		# Turn state vector into transform
		outputTransform = vtk.vtkTransform()
		outputTransform.PostMultiply()
		outputTransform.RotateY(s[4])
		outputTransform.RotateX(s[3])
		outputTransform.RotateZ(s[5])
		outputTransform.Translate(s[0],s[1],s[2])
		print("Registered position:")
		print("  " + str(outputTransform.GetPosition()))
		print("Registered rotation:")
		print("  " + str(outputTransform.GetOrientation()))

		return outputTransform
		
	class _RegistrationInteractorStyle(vtk.vtkInteractorStyleTrackballCamera):
		''' A modified vtk interactor style for manual registration that resets
			the camera when 'c' is pressed
			  Input:
			    camera: vtkCamera object that allows us to reset the camera 
		'''
		camStatic = vtk.vtkCamera()
		# Listen for keypress
		def __init__(self,camera,parent=None):
			self.camStatic
			self.AddObserver("KeyPressEvent", self.keyPressEvent)
		def keyPressEvent(self,obj,event):
			global camStatic
			inter = self.GetInteractor()
			key = inter.GetKeyCode()
			# If 'c' key is pressed, reset the camera
			if key=='c' or key =='C':
				ren = self.GetCurrentRenderer()
				ren.GetActiveCamera().SetFocalPoint(camStatic.GetFocalPoint())
				ren.GetActiveCamera().SetPosition(camStatic.GetPosition())
				ren.GetActiveCamera().SetViewUp(camStatic.GetViewUp())
				inter.Render()


class _RegAnnealer(Annealer):
	''' Class defining the annealer for simulated annealing registration
		  Input
			state (Float Array) = An array of length 6 defining the position
								  [0:2] and the orientation [3:5] of the
								  actor we are registering
			actor (vtkActor) = Mesh we are trying to register to the image
			rw (vtkRenderWindow) = Window that we use to render and compare
			image (vtkImage) = image we compare the render to
			tmax (float) = Starting temperature of the system. During
						   annealing this maps 1:1 to  
	'''
	# pass extra data (for rendering and comparing) into the constructor
	def __init__(self, state, actor, rw, image):
		self.actor = actor
		self.rw = rw
		self.image = image
		self.Tmax = 350
		self.Tmin = .1
		self.steps = 5000
		self.save_state_on_exit = False
		super(_RegAnnealer, self).__init__(state)  # important! 

	# Rotates and translates randomly based on temperature
	def move(self):
		for idx in range(3) :
			self.state[idx] += (random.random()-.5)*self.T
			self.state[idx] = (self.state[idx]+180)%360 -180
			self.state[idx+3] += (random.random()-.5)*self.T#**.5

	#Calculates difference between images
	def energy(self):
		self.actor.SetPosition(self.state[0],self.state[1],self.state[2])
		self.actor.SetOrientation(self.state[3],self.state[4],self.state[5])
		self.rw.Render()

		new_im = vtk.vtkWindowToImageFilter()
		new_im.SetInput(self.rw)
		new_im.Update()

		# Compare images
		idiff = vtk.vtkImageDifference()
		idiff.SetInputConnection(new_im.GetOutputPort())
		idiff.SetImage(self.image)
		idiff.Update()
		e = idiff.GetThresholdedError()
		return e

#if __name__ == "__main__":
#	register('suzanne.stl')