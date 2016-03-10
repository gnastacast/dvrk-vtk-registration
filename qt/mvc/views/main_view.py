from PyQt4 import QtGui, QtCore
from vtk import vtkInteractorStyleTrackballActor
from vtk.qt4.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from gen.ui_main_view import Ui_MainView

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    _fromUtf8 = lambda s: s

class MainView(QtGui.QMainWindow):

    # properties to read/write widget values
    @property
    def masking(self):
        return self.ui.actionMasking.isChecked()
    @masking.setter
    def masking(self, value):
        self.ui.actionMasking.setChecked(value)
    @property
    def autoCamera(self):
        return self.ui.actionAutoCamera.isChecked()
    @autoCamera.setter
    def autoCamera(self, value):
        self.ui.actionAutoCamera.setChecked(value)


    def __init__(self, model, mainCtrl):
        self.model = model
        self.mainCtrl = mainCtrl
        super(MainView, self).__init__()
        self.buildUI()
        # Set up controller update thread
        self.mainCtrl.bgUpdater.VTK_updated.connect(self.renderVTK)
        # register func with model for future model update announcements
        self.model.subscribe_update_func(self.updateUiFromModel)  

    def buildUI(self):
        self.ui = Ui_MainView()
        self.ui.setupUi(self)
        # connect signals to methods
        self.ui.actionMasking.changed.connect(self.onMasking)
        self.ui.actionAutoCamera.changed.connect(self.onSetCameraParams)
        self.ui.actionFineRegistration.triggered.connect(self.onFineRegistration)
        # Replace vertical layout placeholder with a QVTKRenderWindowInteractor
        self.ui.qvtkWidget = QVTKRenderWindowInteractor(self.ui.centralwidget)
        self.ui.qvtkWidget.setObjectName(_fromUtf8("qvtkWidget"))
        imgDims = self.model.bgImage.GetDimensions()
        self.ui.qvtkWidget.setMinimumWidth(imgDims[0])
        self.ui.qvtkWidget.setMaximumWidth(imgDims[0])
        self.ui.qvtkWidget.setMinimumHeight(imgDims[1])
        self.ui.qvtkWidget.setMaximumHeight(imgDims[1])
        self.ui.horizontalLayout.addWidget(self.ui.qvtkWidget)
        # Add this render window to the model
        self.model.setRenWin(self.ui.qvtkWidget.GetRenderWindow())
        self.model.renWin = self.ui.qvtkWidget.GetRenderWindow()
        # Replace default interactor style to send messages to controller
        self.iren = self.ui.qvtkWidget.GetRenderWindow().GetInteractor()
        self.iren.SetInteractorStyle(MainViewInteractorStyle(self.mainCtrl))

    def onMasking(self):
        self.mainCtrl.changeMasking(self.masking)

    def onSetCameraParams(self):
        self.mainCtrl.changeAutoCamera(self.autoCamera)

    def onFineRegistration(self):
        self.mainCtrl.register()

    def updateUiFromModel(self):
        self.masking = self.model.masking

    def renderVTK(self):
        self.ui.qvtkWidget.GetRenderWindow().Render()

    def closeEvent(self,event):
        self.mainCtrl.bgUpdater.running = False
        self.mainCtrl.bgUpdater.wait()
        self.mainCtrl.bgUpdater.terminate()
        self.mainCtrl.bgUpdater.wait()
        self.iren.TerminateApp()
        QtGui.qApp.processEvents()
        super(MainView, self).closeEvent(event)

    def show(self):
        ''' This adds to the default show function to initialize the interactor
            after MainView is shown to avoid a segfault caused if qvtkWidget's
            interactor is initialied before the window is shown.
        '''
        super(MainView, self).show()
        self.iren.Initialize()


class MainViewInteractorStyle(vtkInteractorStyleTrackballActor):
 # Class defining a modified interactor style for registration
    def __init__(self, controller):
        #self.AddObserver("KeyPressEvent", self.keyPressEvent)
        self.AddObserver("RightButtonPressEvent", self.rightButtonPressEvent)
        self.AddObserver("RightButtonReleaseEvent", self.rightButtonReleaseEvent)
        self.AddObserver("LeftButtonPressEvent", self.leftButtonPressEvent)
        self.AddObserver("LeftButtonReleaseEvent", self.leftButtonReleaseEvent)
        self.AddObserver("MouseMoveEvent", self.mouseMoveEvent)
        self.controller = controller
    # This removes camera dolly actions and replaces them with object dolly actions
    # This way the camera stays still no matter what you click.
    def rightButtonPressEvent(self,obj,event):
        self.OnMiddleButtonDown()
        return
    def rightButtonReleaseEvent(self,obj,event):
        self.OnMiddleButtonUp()
        return
    # This sends press events to the controller
    def leftButtonPressEvent(self,obj,event):
        # if not in masking mode or not updating, do nothing
        if self.controller.model.masking and self.controller.bgUpdater.running:
            self.controller.mouseEvent('leftButtonPress',self.GetInteractor())
        elif self.controller.bgUpdater.running:
            self.OnLeftButtonDown()
        return
    def leftButtonReleaseEvent(self,obj,event):
        if self.controller.model.masking and self.controller.bgUpdater.running:
            self.controller.mouseEvent('leftButtonRelease',self.GetInteractor())
        elif self.controller.bgUpdater.running:
            self.OnLeftButtonUp()
        return
    def mouseMoveEvent(self,obj,event):
        if self.controller.model.masking and self.controller.bgUpdater.running:
            self.controller.mouseEvent('mouseMove',self.GetInteractor())
        elif self.controller.bgUpdater.running:
            self.OnMouseMove()
        return
