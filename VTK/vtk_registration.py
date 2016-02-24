#!/usr/bin/env python
 
import vtk
import time
import numpy as np
from amoeba_annealer import amoeba
import os.path

def registerAmoeba(stlPath, imagePath_L, camMatrix_L, initTransform, showRender = False, imgScale = 1, iterations = 400):
 	''' Function for fine registration of an .stl file

		Input:
			stlPath (str) = Path to an STL file to register
		    imagePath_L (str) = Path to an image to register the STL against.
								image should be a black silhouette on a white
								background.
		    camMatrix_L (np.matrix)	= 4x4 camera intrinsic matrix, usually generated
		    						  by OpenCV calibration
			initTransform (vtkTransform) = Initial transformation from camera_L to object

		Optional Input:
			showRender (bool) = Indicates whether to render on screen
								Default is False
			imgScale (float)  =	Scales image up to a particular size for registration
							  	Default is 1
			iterations (int)  = Number of iterations for amoeba_annealer
								Default is 400

		Output:
		    outputTransform (vtkTransform) = transform from camera to object


		- All distances are in meters and all rotations are in degrees

		Example:

		   import vtk_registration
		   print vtk_registration.register("suzanne.stl","suzanne.jpg",camera)

		Version 1.0 2016-March-04 N. Zevallos
	'''

	#Read in the stl
	reader = vtk.vtkSTLReader()
	reader.SetFileName(stlPath)
 
	mapper = vtk.vtkPolyDataMapper()
	if vtk.VTK_MAJOR_VERSION <= 5:
	    mapper.SetInput(reader.GetOutput())
	else:
	    mapper.SetInputConnection(reader.GetOutputPort())

	stlActor = vtk.vtkActor()
	stlActor.SetMapper(mapper)
	stlActor.GetProperty().SetColor(0,0,0)

	# Create the graphics structure. The renderer renders into the render
	# window.
	ren = vtk.vtkRenderer()
	renWin = vtk.vtkRenderWindow()
	renWin.AddRenderer(ren)
 
	# Read in image to register to
	original_image = _loadImage(imagePath_L)
	imgDims = original_image.GetDimensions()[0:2]

	# Add the actors to the renderer, set the background and size
	ren.AddActor(stlActor)
	ren.SetBackground(1, 1, 1)
	windowDims = [int(dim * imgScale) for dim in imgDims]
	renWin.SetSize(windowDims[0],windowDims[1])
	renWin.SetOffScreenRendering(not(showRender))

	# Scale our input image to the window size for comparison
	# Some help from the people at invesalius
	# https://github.com/tatiana/invesalius/blob/master/invesalius/data/imagedata_utils.py
	extent = original_image.GetExtent()
	spacing = original_image.GetSpacing()
	if abs(extent[1]-extent[3]) < abs(extent[3]-extent[5]):
		f = extent[1]
	elif abs(extent[1]-extent[5]) < abs(extent[1] - extent[3]):
		f = extent[1]
	elif abs(extent[3]-extent[5]) < abs(extent[1] - extent[3]):
		f = extent[3]
	else:
		f = extent[1]

	factor = windowDims[0]/float(f+1)

	scaledDown = vtk.vtkImageResample()
	scaledDown.SetInput(original_image)
	scaledDown.SetAxisMagnificationFactor(0, factor)
	scaledDown.SetAxisMagnificationFactor(1, factor)
	scaledDown.SetOutputSpacing(spacing[0] * factor, spacing[1] * factor, spacing[2])
	scaledDown.Update()

	# Setup the camera
	camera = _setupCamera(camMatrix_L,imgDims,windowDims)
	ren.SetActiveCamera(camera)

	# Set up state from initial transformation
	pos = initTransform.GetPosition()
	rot = initTransform.GetOrientation()
	s0 = [pos[0],pos[1],pos[2],rot[0],rot[1],rot[2]]

	stlActor.SetPosition(pos)
	stlActor.SetOrientation(rot)

 	def _afunc(var,data=None): 
		stlActor.SetPosition(var[0],var[1],var[2])
		stlActor.SetOrientation(var[3],var[4],var[5])
		renWin.Render()

		new_im = vtk.vtkWindowToImageFilter()
		new_im.SetInput(renWin)
		new_im.Update()

		# Compare images
		idiff = vtk.vtkImageDifference()
		idiff.SetInputConnection(new_im.GetOutputPort())
		idiff.SetImage(scaledDown.GetOutput())
		idiff.Update()
		err = renWin.GetSize()[0]*renWin.GetSize()[1] - idiff.GetThresholdedError()

		return err

	angleScale = 10
	translationScale = .01
	scales = [translationScale]*3 + [angleScale]*3
	startTime = time.clock()
	s,fvalue,iteration = amoeba(s0,scales,_afunc,ftolerance=.0001,xtolerance=.0001,itmax=iterations)
	totalTime = time.clock()-startTime
	print("Time: " + str(totalTime))
	print("Error: " + str(renWin.GetSize()[0]*renWin.GetSize()[1]-fvalue) +" Iteration:" + str(iteration))

	outputTransform = vtk.vtkTransform()
	outputTransform.PostMultiply()
	outputTransform.RotateY(s[4])
	outputTransform.RotateX(s[3])
	outputTransform.RotateZ(s[5])
	outputTransform.Translate(s[0],s[1],s[2])

	return outputTransform


# Performs manual registration of an STL file
def registerManual(stlPath, imagePath_L, camMatrix_L, initTransform=None, imgScale = 1):
	''' Function for rough user registration of an .stl file

		Input:
			stlPath (str) = path to an STL file to register
		    imagePath_L (str) = path to an image to register the STL against
		    camMatrix_L (np.matrix)	= 4x4 camera intrinsic matrix, usually generated
		    						  by OpenCV calibration
		Optional Input:
			initTransform (vtkTransform) = Initial transformation from camera_L to object
										   Default is pos (XYZ) = (0,0,.15)
										   			  rot (XYZ euler) = (180,0,0)

		Output:
		    outputTransform (vtkTransform) = transform from camera to object

		- All distances are in meters and all rotations are in degrees

		Example:

		   import vtk_registration
		   print vtk_registration.registerManual("suzanne.stl","suzanne.jpg",camera)

		Version 1.0 2016-March-04 N. Zevallos
	'''

	# Create render window and interactor
	renWin = vtk.vtkRenderWindow()

	# Create background renderer for static object
	backgroundRen = vtk.vtkRenderer()
	backgroundRen.SetLayer(0)
	backgroundRen.InteractiveOff()
	backgroundRen.SetBackground(1, 1, 1)
	renWin.AddRenderer(backgroundRen)
	
	# Load in a PNG file onto background
	print imagePath_L
	imgData = _loadImage(imagePath_L)
	bgCamera, backgroundActor = _makeImagePlane(imgData)
	# Setup camera
	backgroundRen.AddActor(backgroundActor)
	backgroundRen.SetActiveCamera(bgCamera)

	imgDims = imgData.GetDimensions()[0:2]
	windowDims = [int(dim * imgScale) for dim in imgDims]
	windowDims = imgDims
	renWin.SetSize(windowDims[0],windowDims[1])
	renWin.SetNumberOfLayers(2)
	iren = vtk.vtkRenderWindowInteractor()
	iren.SetRenderWindow(renWin)
	iren.SetInteractorStyle(_RegistrationInteractorStyle())
	
	# Create renderer for interaction
	ren=vtk.vtkRenderer()
	ren.SetLayer(1)
	renWin.AddRenderer(ren)
	# Setup camera
	camera = _setupCamera(camMatrix_L,imgDims,windowDims)
	ren.SetActiveCamera(camera)

	# Read in STL file
	reader = vtk.vtkSTLReader()
	reader.SetFileName(stlPath)
	mapper = vtk.vtkPolyDataMapper()
	if vtk.VTK_MAJOR_VERSION <= 5:
		mapper.SetInput(reader.GetOutput())
	else:
		mapper.SetInputConnection(reader.GetOutputPort())

	# Create actor that we will position manually
	stlActor = vtk.vtkActor()
	stlActor.SetMapper(mapper)
	stlActor.GetProperty().SetColor(0,1,0)
	stlActor.GetProperty().SetOpacity(1);
	if initTransform == None:
		stlActor.SetPosition(0,0,.15)
		stlActor.SetOrientation(180,0,0)
	else: 
		stlActor.SetPosition(initTransform.GetPosition())
		stlActor.SetOrientation(initTransform.GetOrientation())
	# Add the actor
	ren.AddActor(stlActor)

	# This allows the interactor to initalize itself. It has to be
	# called before an event loop.
	iren.Initialize()
	renWin.Render()

	iren.Start()

	outputTransform = vtk.vtkTransform()
	outputTransform.SetMatrix(stlActor.GetMatrix())

	dist = outputTransform.GetPosition()[2]
	print("cam distance: " + str(dist*39.3701) + ' inches')

	return outputTransform

# Loads a PNG image and returns vtkImageData
def _loadImage(filename) :
	''' Takes a path to a .png image as a string and
		returns a vtkImageData object
	'''
	imageData = vtk.vtkImageData()
	extension = os.path.splitext(filename)[1]
	if extension == ".JPG" or extension == ".jpg":
		imgReader = vtk.vtkJPEGReader()
	elif extension == ".PNG" or extension == ".png":
		imgReader = vtk.vtkPNGReader()
	else:
		raise TypeError("file appears to be neither a .PNG nor a .JPG")
		return
	if(not(imgReader.CanReadFile(filename))):
		raise TypeError("Error reading " + extension + "file "+filename)
		return
	imgReader.SetFileName(filename)
	imgReader.Update()
	imageData = imgReader.GetOutput()
	return imageData

# Loads an image and returns a plane object and a camera pointed towards it
def _makeImagePlane(imageData) :
	''' Takes a path to a .png image as a string and
		returns a plane textured with that image
		and a camera for the background renderer
		http://www.vtk.org/Wiki/VTK/Examples/Cxx/Images/BackgroundImage
		was the basis for this function
	'''
	imageActor = vtk.vtkImageActor()
	if vtk.VTK_MAJOR_VERSION <= 5 :
	  imageActor.SetInput(imageData);
	else :
	  imageActor.SetInputData(imageData);

	origin = imageData.GetOrigin()
	spacing = imageData.GetSpacing()
	extent = imageData.GetExtent()

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

# Sets up a camera based on a camera intrinsics matrix from OpenCV
def _setupCamera(camMatrix, imageDims, windowDims) :
	''' Takes a camera matrix derived from OpenCV
		returns a camera for registration
		http://stackoverflow.com/questions/25539898/how-to-apply-the-camera-pose-transformation-computed-using-epnp-to-the-vtk-camer
		was a great help
	'''
	# Initialize camera
	camera = vtk.vtkCamera()

	# Set view angle
	focalLengthY = camMatrix[1,1]
	if( windowDims[1] != imageDims[1] ):
		factor = float(windowDims[1])/float(imageDims[1])
		focalLengthY = camMatrix[1,1] * factor
	viewAngle = 2 * np.arctan( (windowDims[1] / 2.0 ) / focalLengthY ) * 180 / np.pi
	camera.SetViewAngle(viewAngle)

	#Set window center
	py = 0;
	height = 0;

	if imageDims[0] != windowDims[0] or imageDims[1] != windowDims[1] :

		factor = float(windowDims[1])/float(imageDims[1])
		px = factor * camMatrix[0,2]
		width = windowDims[0]
		expectedWindowSize = np.round(factor * float(imageDims[0]));
		if( expectedWindowSize != windowDims[0] ):
			diffX = (windowDims[0] - expectedWindowSize) / 2;
			px = px + diffX

		py = factor * camMatrix[1,2]
		height = windowDims[1]

	else :
		px = camMatrix[0,2]
		width = imageDims[0]

		py = camMatrix[1,2]
		height = imageDims[1]

	cx = width - px;
	cy = py

	windowCenter = [0,0]

	windowCenter[0] = cx / ( ( width-1)/2 ) - 1 
	windowCenter[1] = cy / ( ( height-1)/2 ) - 1

	camera.SetWindowCenter(windowCenter[0],windowCenter[1])

	camera.SetPosition(0,0,0)
	camera.SetFocalPoint(0,0,1)
	return camera

# Reads states from a file as a list of transforms. State files are in the form
# 	posX_1 posY_1 posZ_1 rotX_1 rotY_1 rotZ_1
# 	posX_2 posY_2 posZ_2 rotX_2 rotY_2 rotZ_2
# 	...
# Where positions are in meters and rotations are XYZ euler rotations in degrees
def readStates(filepath):
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

# Saves a list of transforms as a state file
def saveStates(transforms, filepath) :
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

# Class defining a modified interactor style for manual registration
class _RegistrationInteractorStyle(vtk.vtkInteractorStyleTrackballActor):
 
	def __init__(self,parent=None):
		self.AddObserver("KeyPressEvent", self.keyPressEvent)
		self.AddObserver("RightButtonPressEvent", self.rightButtonPressEvent)
		self.AddObserver("RightButtonReleaseEvent", self.rightButtonReleaseEvent)

	# This removes camera dolly actions and replaces them with object dolly actions
	# This way the camera stays still no matter what you click.
	def rightButtonPressEvent(self,obj,event):
		self.OnMiddleButtonDown()
		return
	def rightButtonReleaseEvent(self,obj,event):
		self.OnMiddleButtonUp()
		return

	# Special keypress event to totally exit the application
	def keyPressEvent(self,obj,event):
		inter = self.GetInteractor()
		key = inter.GetKeyCode()

		if key=='c' or key =='C':
			print "Registration cancelled"
			quit()

if __name__ == "__main__":
	from glob import glob

	''' PANASONIC CAM PARAMS
	camMatrix = np.matrix( [[  9.07378018e+03,  0.00000000e+00,  1.88750000e+03],
							[  0.00000000e+00,  9.07378018e+03,  1.25950000e+03],
							[  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
	'''
	''' CANNON S100 CAM PARAMS
	camMatrix = np.matrix( [[  1.30602653e+04,   0.00000000e+00,   1.99950000e+03],
							[  0.00000000e+00,   1.30602653e+04,   1.49950000e+03],
							[  0.00000000e+00,   0.00000000e+00,   1.00000000e+00]])
	'''
	import codecs, json 
	obj_text = codecs.open('stereoCams.json', 'r', encoding='utf-8').read()
	camParams = json.loads(obj_text)
	camMatrix = np.matrix(camParams['camera_matrix _1'])

	if not(os.path.isfile('ImagesStereo/reg_manual.txt')):
		img_mask = 'ImagesStereo/stereo_*_L.jpg'
		img_names = sorted(glob(img_mask))
		transforms = []
		for filename in img_names:
			transforms.append(registerManual('objects/Dovetail.stl',filename,camMatrix))
		saveStates(transforms,'ImagesStereo/reg_manual.txt')

	initTransforms = readStates('ImagesStereo/reg_manual.txt')

	for transform in initTransforms:
		#a = np.array(ren.GetActiveCamera().GetPosition())
		#b = np.array(stlActor.GetPosition())
		dist = transform.GetPosition()[2]
		print("cam distance: " + str(dist*39.3701) + ' inches')

	img_mask = 'ImagesStereo/Edited/*.jpg'
	img_names = sorted(glob(img_mask))
	print img_names
	transforms = []
	for i in range(len(img_names)):
		filename = img_names[i]
		transform = initTransforms[i]
		transforms.append(registerAmoeba('objects/Dovetail.stl',filename,camMatrix,transform,showRender=True))
	
	#saveStates(transforms,'ImagesStereo/reg_anneal.txt')

	finalTransforms = readStates('ImagesStereo/reg_anneal.txt')

	for transform in finalTransforms:
		dist = transform.GetPosition()[2]
		print("cam distance: " + str(dist*39.3701) + ' inches')
