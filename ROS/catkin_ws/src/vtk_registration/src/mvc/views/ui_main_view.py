# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mvc/views/main_view.ui'
#
# Created: Sat Mar  5 18:07:14 2016
#      by: PyQt4 UI code generator 4.9.1
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    _fromUtf8 = lambda s: s

class Ui_MainView(object):
    def setupUi(self, MainView):
        MainView.setObjectName(_fromUtf8("MainView"))
        MainView.resize(772, 521)
        self.centralwidget = QtGui.QWidget(MainView)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.horizontalLayout = QtGui.QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.qvtkWidget = QtGui.QVBoxLayout()
        self.qvtkWidget.setObjectName(_fromUtf8("qvtkWidget"))
        self.horizontalLayout.addLayout(self.qvtkWidget)
        MainView.setCentralWidget(self.centralwidget)
        self.statusbar = QtGui.QStatusBar(MainView)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainView.setStatusBar(self.statusbar)
        self.toolBar = QtGui.QToolBar(MainView)
        self.toolBar.setObjectName(_fromUtf8("toolBar"))
        MainView.addToolBar(QtCore.Qt.TopToolBarArea, self.toolBar)
        self.actionMasking = QtGui.QAction(MainView)
        self.actionMasking.setCheckable(True)
        self.actionMasking.setObjectName(_fromUtf8("actionMasking"))
        self.actionFineRegistration = QtGui.QAction(MainView)
        self.actionFineRegistration.setObjectName(_fromUtf8("actionFineRegistration"))
        self.actionAutoCamera = QtGui.QAction(MainView)
        self.actionAutoCamera.setCheckable(True)
        self.actionAutoCamera.setChecked(False)
        self.actionAutoCamera.setObjectName(_fromUtf8("actionAutoCamera"))
        self.toolBar.addAction(self.actionMasking)
        self.toolBar.addAction(self.actionFineRegistration)
        self.toolBar.addAction(self.actionAutoCamera)

        self.retranslateUi(MainView)
        QtCore.QMetaObject.connectSlotsByName(MainView)

    def retranslateUi(self, MainView):
        MainView.setWindowTitle(QtGui.QApplication.translate("MainView", "VTK registration", None, QtGui.QApplication.UnicodeUTF8))
        self.toolBar.setWindowTitle(QtGui.QApplication.translate("MainView", "toolBar", None, QtGui.QApplication.UnicodeUTF8))
        self.actionMasking.setText(QtGui.QApplication.translate("MainView", "Masking View", None, QtGui.QApplication.UnicodeUTF8))
        self.actionFineRegistration.setText(QtGui.QApplication.translate("MainView", "Fine Registration", None, QtGui.QApplication.UnicodeUTF8))
        self.actionAutoCamera.setText(QtGui.QApplication.translate("MainView", "Camera Auto Adjust", None, QtGui.QApplication.UnicodeUTF8))

