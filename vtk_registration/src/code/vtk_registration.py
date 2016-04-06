#!/usr/bin/env python
import os
import sys
from PyQt4 import QtGui
from model import MainModel
from controller import MainController
from views.main_view import MainView

class vtkRegistration(QtGui.QApplication):
    def __init__(self, imgDims, stlPath, camMatrix, sys_argv):
        super(vtkRegistration, self).__init__(sys_argv)
        self.model = MainModel(imgDims, stlPath, camMatrix)
        self.controller = MainController(self.model)
        self.view = MainView(self.model, self.controller)
        self.view.show()

if __name__ == '__main__':
    imgDims = (1280,720)
    stlpath = 'defaults/objects/Dovetail.stl'
    # Read camera matrix for VTK
    import codecs, json
    import numpy as np
    obj_text = codecs.open('defaults/Logitech_c920/singleCam.json', 'r', encoding='utf-8').read()
    camParams = json.loads(obj_text)
    camMatrix = np.matrix(camParams['camera_matrix'])
    app = vtkRegistration(imgDims, stlpath, camMatrix, sys.argv)
    sys.exit(app.exec_())
    app.controller.videoUpdater.running=False
    app.controller.videoUpdater.wait()