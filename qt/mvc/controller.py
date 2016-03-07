import numpy as np
import cv2
from PyQt4 import QtCore, QtGui
from vtkTools import numpyToVtkImage
from subprocess import call
import time

class MainController(object):

    def __init__(self, model):
        self.model = model
        self.videoUpdater = self._VideoUpdateThread(model)
        self.videoUpdater.start()
        self.drawingBox = False
        self.drawingLine = False
        self.brushColor = int(cv2.GC_FGD)
        self.needsUpdate = False;

    # called from view class
    def changeMasking(self, checked):
        # put control logic here
        self.model.masking = checked
        self.model.announce_update()
        if checked:
            self.model.stlActor.VisibilityOff()
        else:
            self.model.stlActor.VisibilityOn()

    def changeAutoCamera(self,checked):
        print checked
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
        cv2.circle(self.model.mask,pos, 5, (self.brushColor), -1)

    def mouseEvent(self,eventType,interactor):

        # if not in masking mode, do nothing
        if not(self.model.masking):
            return
        # Get position of event
        mousePos = interactor.GetEventPosition()
        mousePos = (mousePos[0], interactor.GetSize()[1] - mousePos[1])

        if eventType == 'mouseMove':
            if self.drawingBox == True:
                self.model.rect = self.model.rect[0:2]+mousePos
            if self.drawingLine == True:
                self.drawOnMask(mousePos)

        elif eventType == 'leftButtonPress':
            if interactor.GetShiftKey():
                self.drawingLine = True
                self.brushColor = int(cv2.GC_FGD)
                self.drawOnMask(mousePos)
            elif interactor.GetControlKey():
                self.drawingLine = True
                self.brushColor = int(cv2.GC_BGD)
                self.drawOnMask(mousePos)
            else:
                self.drawingBox = True
                self.model.rect = mousePos + mousePos

        elif eventType == 'leftButtonRelease':
            if self.drawingBox == True:
                self.model.mask = np.ones(self.model.imgDims[::-1],np.uint8)*cv2.GC_PR_FGD
            self.drawingBox = False
            self.drawingLine = False
            self.needsUpdate = True

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

    class _VideoUpdateThread(QtCore.QThread):

        VTK_updated = QtCore.pyqtSignal(object)

        def __init__(self, model):
            QtCore.QThread.__init__(self)
            self.running = True
            self.model = model

        def run(self):
            while self.running:
                if self.model.masking:
                    self.updateMasking()
                else:
                    self.updateVideo()

        def updateVideo(self):
            ret, frame = self.model.cap.read()
            self.model.videoFrame = frame.copy()
            numpyToVtkImage(frame,self.model.bgImage)
            self.VTK_updated.emit(1)

        def updateMasking(self):
            boxStart = self.model.rect[0:2]
            boxEnd = self.model.rect[2:4]
            tempFrame = self.model.videoFrame.copy()
            cv2.rectangle(tempFrame,boxStart,boxEnd,(0,255,0),3)
            tempFrame[self.model.mask==cv2.GC_FGD] = (0,255,0)
            tempFrame[self.model.mask==cv2.GC_BGD] = (0,0,255)
            numpyToVtkImage(tempFrame,self.model.bgImage)
            self.VTK_updated.emit(1)
    
    