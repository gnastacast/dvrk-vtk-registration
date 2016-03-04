#!/usr/bin/env python
 
import sys
import vtk
import numpy as np
import cv2
from PyQt4 import QtCore, QtGui
from vtk.qt4.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from vtk.util import numpy_support
from vtk_registration_tools import vtkRegistration, makeCamMatrix
from subprocess import call
from time import sleep

def numpyToVTKImage(numpyData,VTKImageData):
    rgb = cv2.cvtColor(numpyData, cv2.COLOR_BGR2RGB)
    VTK_data = numpy_support.numpy_to_vtk(num_array=rgb[::-1].ravel(),deep=True, array_type=vtk.VTK_CHAR)
    VTKImageData.GetPointData().SetScalars(VTK_data)
    return VTKImageData

def vtkImageToNumpy(VTKImageData,numpyData):
    size = VTKImageData.GetDimensions()
    numpyData = numpy_support.vtk_to_numpy(VTK_data)
    numpyData = numpyData.reshape((size[1],size[0],3))
    return numpyData

def setupVTKImage(imgDims):
    imData = vtk.vtkImageData()
    imData.SetDimensions(imgDims[0], imgDims[1], 1) #set dimensions as necessary
    imData.SetOrigin(0,0,0) #set origin as necessary
    imData.SetSpacing(1, 1, 1) #set spacing as necessary
    imData.SetNumberOfScalarComponents( 3 )
    imData.SetScalarTypeToUnsignedChar()
    imData.Update()
    return imData

class _MaskingTool:
    def __init__(self,img):
        self.boxStart = (0,0)
        self.boxEnd = (0,0)
        self.drawingBox = False
        self.drawingLine = False
        self.masking = False
        self.changed = False
        self.brushPos = (0,0)
        self.adding = True
        self.mask = np.ones(img.shape[:2],np.uint8)*cv2.GC_PR_FGD

    def draw(self,img):
        cv2.rectangle(img,self.boxStart,self.boxEnd,(0,255,0),3)
        if self.drawingLine == True:
            if self.adding:
                col = int(cv2.GC_FGD)
            else:
                col = int(cv2.GC_BGD)
            cv2.circle(self.mask,self.brushPos, 5, (col), -1)
        img[self.mask==cv2.GC_FGD] = (0,255,0)
        img[self.mask==cv2.GC_BGD] = (0,0,255)

    def reset(self):
        self.__init__(self.mask)

class _VTKUpdateThread(QtCore.QThread):

    VTK_updated = QtCore.pyqtSignal(object)

    def __init__(self, capture, imData, maskingTool, registration):
        QtCore.QThread.__init__(self)
        self.running = True
        self.imData = imData
        self.capture = capture
        self.maskingTool = maskingTool
        self.registering = False
        self.register = registration
        self.actor = registration.stlActor

    def updateVideo(self):
        ret, frame = self.capture.read()
        maskedFrame = frame.copy()
        self.imData = numpyToVTKImage(frame,self.imData)

    def run(self):
        ret, frame = self.capture.read()
        maskedFrame = frame.copy()
        drawFrame = frame.copy()
        while self.running:
            if self.registering:
                self.updateVideo()
                self.VTK_updated.emit("Updated!")
                print ('Began fine registration')
                #imageWriter = vtk.vtkPNGWriter()
                #imageWriter.SetFileName('testImage.png')
                #imageWriter.SetInputConnection(self.scaledZBuff.GetOutputPort());
                #imageWriter.Write();
                print self.actor.GetPosition()
                from time import sleep
                counter = 0
                while counter<100:
                    sleep(.05)
                    self.actor.GetProperty().SetOpacity(counter/100.0)
                    self.VTK_updated.emit("Updated!")
                    counter+=1
                self.actor.GetProperty().SetOpacity(1)
                self.registering=False
            # If we are in masking mode, draw masking
            # objects and don't update the video
            elif self.maskingTool.masking:
                if self.maskingTool.changed:
                    maskedFrame = self._removeBG(frame)
                    self.maskingTool.changed = False;
                drawFrame = maskedFrame.copy()
                self.maskingTool.draw(drawFrame)
                self.imData = numpyToVTKImage(drawFrame,self.imData)
            # Otherwise update the video
            else:
                self.updateVideo()
            # Tell the main thread we've updated
            self.VTK_updated.emit("Updated!")

    def _removeBG(self, frame):
        startCorner = self.maskingTool.boxStart
        endCorner = self.maskingTool.boxEnd
        img = frame.copy()
        mask = np.ones(img.shape[:2],np.uint8)*cv2.GC_BGD
        mask[startCorner[1]:endCorner[1], startCorner[0]:endCorner[0]] = \
            self.maskingTool.mask[startCorner[1]:endCorner[1],startCorner[0]:endCorner[0]]
        rect = startCorner+endCorner
        tmp1 = np.zeros((1, 13 * 5))
        tmp2 = np.zeros((1, 13 * 5))
        print "Started Grabcut"
        startCorner = self.maskingTool.masking=False
        cv2.grabCut(img,mask,rect,tmp1,tmp2,5,mode=cv2.GC_INIT_WITH_MASK)
        startCorner = self.maskingTool.masking=True
        print "Finished Grabcut"
        mask2 = np.where((mask==2)|(mask==0),0.25,1)
        img[:,:,0:2] = img[:,:,0:2]*mask2[:,:,np.newaxis]
        return img.astype('uint8')

class MainWindow(QtGui.QMainWindow):
 
    def __init__(self, parent = None):
        QtGui.QMainWindow.__init__(self, parent)
 
        frame = QtGui.QFrame()
        vl = QtGui.QVBoxLayout()
        grid_vl  =  QtGui.QGridLayout()

        # Set up fine registration button
        registerButton = QtGui.QPushButton('Fine Registration', self)
        registerButton.clicked.connect(self._handleRegisterButton)
        grid_vl.addWidget(registerButton, 0, 0)

        # Set up calibrateCamera registration button
        if(sys.platform[0:5]=='linux'):
            cameraSetButton = QtGui.QPushButton('Set camera exposure and focus', self)
            cameraSetButton.clicked.connect(self.handleCameraSetButton)
            grid_vl.addWidget(cameraSetButton, 0,1)

        # Set up masking button
        maskButton = QtGui.QPushButton('Start/Stop masking', self)
        maskButton.clicked.connect(self.handleMaskImageButton)
        grid_vl.addWidget(maskButton,0,2)
        vl.addLayout(grid_vl)

        # Set up video capture
        cap = cv2.VideoCapture(0)
        self.imgDims = (1280,720)
        cap.set(3,self.imgDims[0])
        cap.set(4,self.imgDims[1])
        ret, img = cap.read()
        self.imgData = setupVTKImage(self.imgDims)
        print img.shape
        print self.imgData.GetDimensions()
        # Set up VTK widget
        self.vtkWidget = QVTKRenderWindowInteractor(frame)
        self.vtkWidget.setMinimumWidth(self.imgDims[0])
        self.vtkWidget.setMaximumWidth(self.imgDims[0])
        self.vtkWidget.setMinimumHeight(self.imgDims[1])
        self.vtkWidget.setMaximumHeight(self.imgDims[1])
        vl.addWidget(self.vtkWidget)
        # Read camera matrix for VTK
        import codecs, json 
        obj_text = codecs.open('singleCam.json', 'r', encoding='utf-8').read()
        camParams = json.loads(obj_text)
        camMatrix = np.matrix(camParams['camera_matrix'])
        # Read in STL to register
        self.registration = vtkRegistration('objects/Dovetail.stl', self.imgData, self.vtkWidget.GetRenderWindow(), camMatrix)
        # Set up Zbuffer for registration
        zBuffFilter = vtk.vtkWindowToImageFilter()
        zBuffFilter.SetInput(self.vtkWidget.GetRenderWindow())
        zBuffFilter.SetInputBufferTypeToZBuffer()

        self.scaledZBuff = vtk.vtkImageShiftScale()
        self.scaledZBuff.SetOutputScalarTypeToUnsignedChar()
        self.scaledZBuff.SetInputConnection(zBuffFilter.GetOutputPort())
        self.scaledZBuff.SetShift(0)
        self.scaledZBuff.SetScale(255)

        # Set up masking
        self.maskingTool = _MaskingTool(img)

        # Set up video update thread
        self.videoUpdate = _VTKUpdateThread(cap,self.imgData,self.maskingTool, self.registration)
        self.videoUpdate.VTK_updated.connect(self.vtkWidget.GetRenderWindow().Render)
        self.videoUpdate.start()     
        # Set up registration thread
        #self.registrationUpdate = _RegistrationUpdateThread(self.scaledZBuff,self.registration)
        #self.registrationUpdate.registration_updated.connect(self.vtkWidget.GetRenderWindow().Render)
        #self.registrationUpdate.registration_finished.connect(self.restartVideo)

        # Finalize layout
        frame.setLayout(vl)
        self.setCentralWidget(frame)

        # Set up interactor
        self.iren = self.vtkWidget.GetRenderWindow().GetInteractor()
        self.iren.SetInteractorStyle(RegistrationInteractorStyle(self.maskingTool))
        self.iren.Initialize()
        self.show()

    def restartVideo(self):
        self.videoUpdate.running = True
        self.videoUpdate.start()

    def _handleRegisterButton(self):
        self.registration.stlActor.VisibilityOn()  
        self.maskingTool.masking = False   
        self.videoUpdate.registering = True

    def handleMaskImageButton(self):
        # Don't switch to mask mode if registration is happening
        if self.videoUpdate.registering == True:
            return
        if self.maskingTool.masking:
            self.registration.stlActor.VisibilityOn()
        else:
            self.maskingTool.reset()
            self.registration.stlActor.VisibilityOff()
        self.maskingTool.masking = not(self.maskingTool.masking)

    def handleCameraSetButton(self):

        # Don't switch to mask mode if registration is happening
        if self.videoUpdate.registering == True:
            return

        # Turn on all automatic features
        call(["v4l2-ctl", "--set-ctrl", "exposure_auto=3"])
        call(["v4l2-ctl", "--set-ctrl", "focus_auto=1"])
        call(["v4l2-ctl", "--set-ctrl", "white_balance_temperature_auto=1"])
        # Wait for features to do their duty
        sleep(1)
        # Turn off all automatic features
        call(["v4l2-ctl", "--set-ctrl", "exposure_auto=1"])
        call(["v4l2-ctl", "--set-ctrl", "focus_auto=0"])
        call(["v4l2-ctl", "--set-ctrl", "white_balance_temperature_auto=0"])


# Class defining a modified interactor style for manual registration
class RegistrationInteractorStyle(vtk.vtkInteractorStyleTrackballActor):
 
    def __init__(self,maskingTool):
        #self.AddObserver("KeyPressEvent", self.keyPressEvent)
        self.AddObserver("RightButtonPressEvent", self.rightButtonPressEvent)
        self.AddObserver("RightButtonReleaseEvent", self.rightButtonReleaseEvent)
        self.AddObserver("LeftButtonPressEvent", self.leftButtonPressEvent)
        self.AddObserver("LeftButtonReleaseEvent", self.leftButtonReleaseEvent)
        self.AddObserver("MouseMoveEvent", self.mouseMoveEvent)
        self.maskingTool = maskingTool

    # This removes camera dolly actions and replaces them with object dolly actions
    # This way the camera stays still no matter what you click.
    def rightButtonPressEvent(self,obj,event):
        self.OnMiddleButtonDown()
        return
    def rightButtonReleaseEvent(self,obj,event):
        self.OnMiddleButtonUp()
        return

    def leftButtonPressEvent(self,obj,event):
        if self.maskingTool.masking:
            interactor = self.GetInteractor()
            mousePos = interactor.GetEventPosition()
            mousePos = (mousePos[0], interactor.GetSize()[1] - mousePos[1])
            if interactor.GetShiftKey():
                self.maskingTool.brushPos = mousePos
                self.maskingTool.drawingLine = True
                self.maskingTool.adding = True
                return
            if interactor.GetControlKey():
                self.maskingTool.brushPos = mousePos
                self.maskingTool.drawingLine = True
                self.maskingTool.adding = False
                return
            self.maskingTool.drawingBox = True
            self.maskingTool.boxStart = mousePos
            self.maskingTool.boxEnd = mousePos
            return
        self.OnLeftButtonDown()
        return
    def leftButtonReleaseEvent(self,obj,event):
        if self.maskingTool.masking:
            if self.maskingTool.drawingBox == True:
                self.maskingTool.drawingBox = False
                if all(v == 0 for v in self.maskingTool.boxStart+self.maskingTool.boxEnd):
                    return
                self.maskingTool.changed = True
                print self.maskingTool.boxStart, self.maskingTool.boxEnd
                return
            if self.maskingTool.drawingLine == True:
                self.maskingTool.drawingLine = False
                self.maskingTool.changed = True
                return
        self.OnLeftButtonUp()
        return

    def mouseMoveEvent(self,obj,event):
        if self.maskingTool.masking:
            interactor = self.GetInteractor()
            mousePos = interactor.GetEventPosition()
            mousePos = (mousePos[0], interactor.GetSize()[1] - mousePos[1])
            if self.maskingTool.masking and self.maskingTool.drawingBox == True:
                self.maskingTool.boxEnd = mousePos
                return
            if self.maskingTool.drawingLine == True:
                self.maskingTool.brushPos = mousePos
                return
        self.OnMouseMove()
        return

if __name__ == "__main__":
 
    app = QtGui.QApplication(sys.argv)
    
    window = MainWindow()
 
    sys.exit(app.exec_())