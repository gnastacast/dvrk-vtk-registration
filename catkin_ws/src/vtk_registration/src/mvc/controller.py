import time
from sys import platform
from subprocess import call
import numpy as np
import cv2
import rospy
from geometry_msgs.msg import Transform
from PyQt4 import QtCore
from vtkTools import numpyToVtkImage
from vtkTools import vtkImageToNumpy
from amoeba_annealer import amoeba

class MainController(object):

    def __init__(self, model):
        self.model = model
        # Video update thread
        self.bgUpdater = self._BackgroundUpdateThread(model)
        self.bgUpdater.start()
        # Variables for drawing mask
        self.drawingBox = False
        self.drawingLine = False
        self.brushColor = int(cv2.GC_FGD)
        self.pub = rospy.Publisher('chatter', Transform, queue_size=1)
        rospy.init_node('vtk_registration', anonymous=True)


    # called from view class
    def changeMasking(self, checked):
        # Turn masking variable in model off and on
        self.model.masking = checked
        self.model.announce_update()
        # Hide actor while masking
        if checked:
            self.model.stlActor.VisibilityOff()
        else:
            self.model.stlActor.VisibilityOn()

    def changeAutoCamera(self,checked):
        ''' Hacky function that turns auto exposure and auto white white balance
            off and on. This is camera and operating system specific so it will
            probably need to be changed in order to work with your setup.
        '''
        # Only try this if on linux.
        if not('linux' in platform):
            return
        if checked:
            # Turn on all automatic features
            call(["v4l2-ctl", "--set-ctrl", "exposure_auto=3"])
            call(["v4l2-ctl", "--set-ctrl", "focus_auto=1"])
            call(["v4l2-ctl", "--set-ctrl", "white_balance_temperature_auto=1"])
        else:
            # Turn off all automatic features
            call(["v4l2-ctl", "--set-ctrl", "exposure_auto=1"])
            call(["v4l2-ctl", "--set-ctrl", "focus_auto=0"])
            call(["v4l2-ctl", "--set-ctrl", "white_balance_temperature_auto=0"])
    
    def drawOnMask(self, pos):
        cv2.circle(self.model.drawnMask,pos, 5, (self.brushColor), -1)
    
    def mouseEvent(self,eventType,interactor):
        # Get position of event
        mousePos = interactor.GetEventPosition()
        mousePos = (mousePos[0], interactor.GetSize()[1] - mousePos[1])

        if eventType == 'mouseMove':
            if self.drawingBox == True:
                # Set rectangular masking area
                self.model.rect = self.model.rect[0:2]+mousePos
            if self.drawingLine == True:
                self.drawOnMask(mousePos)

        elif eventType == 'leftButtonPress':
            if interactor.GetShiftKey():
                # Mark as foreground
                self.drawingLine = True
                self.brushColor = int(cv2.GC_FGD)
                self.drawOnMask(mousePos)
            elif interactor.GetControlKey():
                # Mark as background
                self.drawingLine = True
                self.brushColor = int(cv2.GC_BGD)
                self.drawOnMask(mousePos)
            else:
                # Begin marking rectangular masking area
                self.model.drawnMask = np.ones(self.model.imgDims[::-1],np.uint8)*cv2.GC_PR_FGD
                self.drawingBox = True
                self.model.rect = mousePos + mousePos

        elif eventType == 'leftButtonRelease':
            # Check rectangular mask area is not zero
            rect = self.model.rect
            area = (rect[2] - rect[0]) * (rect[3] - rect[1])
            if area!=0:
                # Update masked frame and clear drawing variables
                self.model.maskedFrame = self._removeBG(self.model.videoFrame)
                self.drawingBox = False
                self.drawingLine = False

    def _removeBG(self, frame):
        ''' This function uses grabcut to mask out foreground and background
        '''
        # Stop update to stop commands from piling up
        self.bgUpdater.running = False
        # Begin with a new mask that is all background
        img = frame.copy()
        mask = np.ones(img.shape[:2],np.uint8)*cv2.GC_BGD
        # Replace everything inside the box selection with data from the mask
        rect = (x1,y1,x2,y2) = self.model.rect
        mask[y1:y2, x1:x2] = self.model.drawnMask[y1:y2,x1:x2]
        # Perform grabcut algorithm to separate background and foreground
        tmp1 = np.zeros((1, 13 * 5))
        tmp2 = np.zeros((1, 13 * 5))
        cv2.grabCut(img,mask,rect,tmp1,tmp2,5,mode=cv2.GC_INIT_WITH_MASK)
        # Update model's mask
        self.model.mask = np.where((mask==2)|(mask==0),0,255)
        # Redden areas that are masked out
        mask2 = np.where((mask==2)|(mask==0),0.25,1)
        img[:,:,0:2] = img[:,:,0:2]*mask2[:,:,np.newaxis]
        # Restart update
        self.bgUpdater.running = True
        self.bgUpdater.start()
        return img.astype('uint8')

    def register(self):
        ''' This function uses downhill simplex and simulated annealing to 
            perform fine registration of the STL to its real world counterpart
        '''
        # Stop update to stop commands from piling up
        self.bgUpdater.running = False
        self.bgUpdater.wait()
        self.changeMasking(False)
        # Get renderers
        ren = None
        bgRen = None
        rendererCollection = self.model.renWin.GetRenderers()
        ren = rendererCollection.GetItemAsObject(0)
        bgRen = rendererCollection.GetItemAsObject(1)
        # Set renderers for fast rendering
        ren.ResetCameraClippingRange()
        self.model.stlActor.GetProperty().SetRepresentationToSurface()
        bgRen.Clear()
        bgRen.DrawOff()
        # Initialize state
        pos = self.model.stlActor.GetPosition()
        rot = self.model.stlActor.GetOrientation()
        s0 = [pos[0],pos[1],pos[2],rot[0],rot[1],rot[2]]

        def _distFunc(var,data=None): 
            self.model.stlActor.SetPosition(var[0],var[1],var[2])
            self.model.stlActor.SetOrientation(var[3],var[4],var[5])
            ren.ResetCameraClippingRange()
            bgRen.Clear()
            self.model.renWin.Render()
            frame = vtkImageToNumpy(self.model.zBuff.GetOutput())
            frame = np.where((frame==255),0,255)
            diff = np.absolute(np.subtract(frame[:,:,0],self.model.mask))
            #cv2.imwrite( "testImage.jpg", diff)
            err = self.model.imgDims[0]*self.model.imgDims[1] - np.sum(diff)/255
            return err

        iterations = 400
        angleScale = 10
        translationScale = .01
        scales = [translationScale]*3 + [angleScale]*3
        startTime = time.clock()
        s,fvalue,iteration = amoeba(s0,scales,_distFunc,ftolerance=.0001,
                                    xtolerance=.0001,itmax=iterations)
        totalTime = time.clock()-startTime
        print("Time: " + str(totalTime))
        print("Error: "+str(self.model.imgDims[0]*self.model.imgDims[1]-fvalue)+
              " Iteration:" + str(iteration))

        #   Publish result to ros node
        transformMsg = Transform()
        pos = self.model.stlActor.GetPosition()
        transformMsg.translation.x = pos[0]
        transformMsg.translation.y = pos[1]
        transformMsg.translation.z = pos[2]
        rot = self.model.stlActor.GetOrientationWXYZ()
        transformMsg.rotation.x = rot[0]
        transformMsg.rotation.y = rot[1]
        transformMsg.rotation.z = rot[2]
        transformMsg.rotation.w = rot[3]
        self.pub.publish(transformMsg)

        # Restart update
        bgRen.DrawOn()
        self.bgUpdater.running = True
        self.bgUpdater.start()
        
    class _BackgroundUpdateThread(QtCore.QThread):

        VTK_updated = QtCore.pyqtSignal(object)

        def __init__(self, model):
            QtCore.QThread.__init__(self)
            self.running = True
            self.model = model
            self.checkVideoRateMS = 100

        def run(self):
            while self.running:
                if self.model.masking:
                    self.updateMasking()
                else:
                    self.updateVideo()

        def updateVideo(self):
            if (self.model.cap.isOpened()):
                ret, frame = self.model.cap.read()
                self.model.videoFrame = frame.copy()
                self.model.maskedFrame = frame.copy()
            else:
                frame = self.model.videoFrame.copy()
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(frame,'No Video',(20,20), font, 1,(255,255,255))
                numpyToVtkImage(frame,self.model.bgImage)
                time.sleep(0.001*self.checkVideoRateMS)
                self.model.cap = cv2.VideoCapture(0)
                self.model.cap.set(3,self.model.imgDims[0])
                self.model.cap.set(4,self.model.imgDims[1])

            numpyToVtkImage(frame,self.model.bgImage)
            self.VTK_updated.emit(1)

        def updateMasking(self):
            boxStart = self.model.rect[0:2]
            boxEnd = self.model.rect[2:4]
            tempFrame = self.model.maskedFrame.copy()
            cv2.rectangle(tempFrame,boxStart,boxEnd,(0,255,0),3)
            tempFrame[self.model.drawnMask==cv2.GC_FGD] = (0,255,0)
            tempFrame[self.model.drawnMask==cv2.GC_BGD] = (0,0,255)
            numpyToVtkImage(tempFrame,self.model.bgImage)
            self.VTK_updated.emit(1)