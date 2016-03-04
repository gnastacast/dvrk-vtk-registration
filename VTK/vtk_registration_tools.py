#!/usr/bin/env python
 
import vtk
import time
import numpy as np
from amoeba_annealer import amoeba
import cv2

class vtkRegistration:
	def __init__(self, stlPath, imgData, renWin_L, camMatrix, renWin_R = None):
		# Set render window
		self.renWin = renWin_L
		self.renWin.SetNumberOfLayers(2)

		# Create foreground renderer for stl object
		self.ren=vtk.vtkRenderer()
		self.ren.SetLayer(1)
		self.renWin.AddRenderer(self.ren)

		# Read in STL file
		reader = vtk.vtkSTLReader()
		reader.SetFileName(stlPath)
		mapper = vtk.vtkPolyDataMapper()
		if vtk.VTK_MAJOR_VERSION <= 5:
			mapper.SetInput(reader.GetOutput())
		else:
			mapper.SetInputConnection(reader.GetOutputPort())
		# Create actor that we will position manually
		self.stlActor = vtk.vtkActor()
		self.stlActor.SetMapper(mapper)
		self.stlActor.GetProperty().SetColor(0,1,0)
		self.stlActor.GetProperty().SetOpacity(1)
		self.stlActor.SetPosition(0,0,.15)
		self.stlActor.SetOrientation(180,0,0)
		# Add the actor
		self.ren.AddActor(self.stlActor)

		# Read in vtkImageData
		self.imgData = imgData
		imgDims = self.imgData.GetDimensions()[0:2]
		windowDims = imgDims
		self.renWin.SetSize(windowDims[0],windowDims[1])
		self.camMatrix = camMatrix
		camera = _setupCamera(self.camMatrix,imgDims,windowDims)
		self.ren.SetActiveCamera(camera)

		# Create background renderer for static object
		self.backgroundRen = vtk.vtkRenderer()
		self.backgroundRen.SetLayer(0)
		self.backgroundRen.InteractiveOff()
		self.backgroundRen.SetBackground(1, 1, 1)
		# Create background object for showing video feed.
		bgCamera, self.backgroundActor = _makeImagePlane(self.imgData)
		# Setup camera
		self.backgroundRen.AddActor(self.backgroundActor)
		self.backgroundRen.SetActiveCamera(bgCamera)
		# Add background renderer to window
		self.renWin.AddRenderer(self.backgroundRen)

	def register(self, showRender = True, imgScale = 1, iterations = 400):
	 	''' Function for fine registration of an .stl file

			Optional Input:
				showRender (bool) = Indicates whether to render on screen
									Default is True
				imgScale (float)  =	Scales image up to a particular size for registration
								  	Default is 1
				iterations (int)  = Number of iterations for amoeba_annealer
									Default is 400

			Output:
			    outputTransform (vtkTransform) = transform from camera to object


			- All distances are in meters and all rotations are in degrees
		'''

	 	def _afunc(var,data=None): 
			self.stlActor.SetPosition(var[0],var[1],var[2])
			self.stlActor.SetOrientation(var[3],var[4],var[5])
			self.renWin.Render()

			new_im = vtk.vtkWindowToImageFilter()
			new_im.SetInput(self.renWin)
			new_im.Update()

			# Compare images
			idiff = vtk.vtkImageDifference()
			idiff.SetInputConnection(new_im.GetOutputPort())
			idiff.SetImage(threshedImg.GetOutput())
			idiff.Update()
			err = self.renWin.GetSize()[0]*self.renWin.GetSize()[1] - idiff.GetThresholdedError()

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

		self.stlActor.SetPosition(s[0],s[1],s[2])
		self.stlActor.SetOrientation(s[3],s[4],s[5])

		return outputTransform

# Loads an image and returns a plane object and a camera pointed towards it
def _makeImagePlane(imageData) :
	''' Takes a vtkImageData object and
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

		Input: 
			camMatrix (np.ndarray) = 4x4 camera intrinsic matrix
			imageDims ((float, float)) 	= width and height of image to register to
			imageDims ((float, float)) 	= width and height of render window

		Output:
		    camera (vtk.vtkCamera) = camera to use for registration
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

def makeCamMatrix(fov, sensorDiagonal, sensorWidth, imgDims):
	''' Makes camera matrix using FOV, near and far distances
		based on http://phototour.cs.washington.edu/focal.html

		Input: 
			fov (float) 				= Camera field of view in degrees
			imageDims ((float, float)) 	= width and height of image to register to

		Output:
		    camMatrix (np.ndarray) = 3x3 camera intrinsic matrix
	'''

	# Start with identity matrix
	camMatrix = np.identity(3)
	# Aspect ratio is height over width
	aspect = float(imgDims[0])/float(imgDims[1])
	# FOV = 2 arctan (diagonal / (2 f))
	# FOV/2 = arctan (diagonal / (2 f))
	# tan(FOV/2)=diagonal/(2 f)
	# diagonal/tan(FOV/2) = 2f
	# diagonal/(tan(FOV/2)*2) = f
	fov_in_mm = np.abs(sensorDiagonal/float(np.tan(fov/2)*2))
	#focal length in pixels = (image width in pixels) * (focal length in mm) / (CCD width in mm)
	fov_X_in_px = imgDims[0]*fov_in_mm/float(sensorWidth)
	# check for bad parameters
	print fov_X_in_px
	if ( fov_X_in_px <= 0 or aspect <= 0 ):
		raise ValueError("Either FOV or aspect ratio are <= 0")
	camMatrix[1][1] = fov_X_in_px
	camMatrix[0][0] = camMatrix[1][1] / aspect
	camMatrix[2][0] = imgDims[0]/2.0
	camMatrix[2][1] = imgDims[1]/2.0

	return camMatrix;

