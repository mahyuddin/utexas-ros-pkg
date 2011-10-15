# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'teleop_mainWindow.ui'
#
# Created: Sat Oct 15 17:45:40 2011
#      by: PyQt4 UI code generator 4.8.3
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    _fromUtf8 = lambda s: s

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(229, 82)
        MainWindow.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.stopButton = QtGui.QPushButton(self.centralwidget)
        self.stopButton.setGeometry(QtCore.QRect(70, 50, 80, 25))
        self.stopButton.setFocusPolicy(QtCore.Qt.NoFocus)
        self.stopButton.setStyleSheet(_fromUtf8("background-color: rgb(255, 0, 0);\n"
"font: 75 12pt \"Sans Serif\";\n"
"color: rgb(255, 255, 255);"))
        self.stopButton.setObjectName(_fromUtf8("stopButton"))
        self.fwdLabel = QtGui.QLabel(self.centralwidget)
        self.fwdLabel.setGeometry(QtCore.QRect(0, 10, 91, 16))
        self.fwdLabel.setObjectName(_fromUtf8("fwdLabel"))
        self.turnLabel = QtGui.QLabel(self.centralwidget)
        self.turnLabel.setGeometry(QtCore.QRect(0, 30, 81, 16))
        self.turnLabel.setObjectName(_fromUtf8("turnLabel"))
        self.fwdVel = QtGui.QLabel(self.centralwidget)
        self.fwdVel.setGeometry(QtCore.QRect(60, 10, 31, 16))
        self.fwdVel.setObjectName(_fromUtf8("fwdVel"))
        self.turnVel = QtGui.QLabel(self.centralwidget)
        self.turnVel.setGeometry(QtCore.QRect(60, 30, 31, 16))
        self.turnVel.setObjectName(_fromUtf8("turnVel"))
        self.fwdInc = QtGui.QDoubleSpinBox(self.centralwidget)
        self.fwdInc.setGeometry(QtCore.QRect(160, 8, 62, 22))
        self.fwdInc.setFocusPolicy(QtCore.Qt.ClickFocus)
        self.fwdInc.setKeyboardTracking(True)
        self.fwdInc.setMinimum(0.02)
        self.fwdInc.setMaximum(1.0)
        self.fwdInc.setSingleStep(0.02)
        self.fwdInc.setObjectName(_fromUtf8("fwdInc"))
        self.label = QtGui.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(120, 10, 31, 16))
        self.label.setObjectName(_fromUtf8("label"))
        self.turnInc = QtGui.QDoubleSpinBox(self.centralwidget)
        self.turnInc.setGeometry(QtCore.QRect(160, 28, 62, 22))
        self.turnInc.setFocusPolicy(QtCore.Qt.ClickFocus)
        self.turnInc.setMinimum(0.02)
        self.turnInc.setMaximum(1.0)
        self.turnInc.setSingleStep(0.02)
        self.turnInc.setObjectName(_fromUtf8("turnInc"))
        self.label_2 = QtGui.QLabel(self.centralwidget)
        self.label_2.setGeometry(QtCore.QRect(120, 30, 31, 16))
        self.label_2.setObjectName(_fromUtf8("label_2"))
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QtGui.QApplication.translate("MainWindow", "Teleop", None, QtGui.QApplication.UnicodeUTF8))
        self.stopButton.setText(QtGui.QApplication.translate("MainWindow", "STOP", None, QtGui.QApplication.UnicodeUTF8))
        self.fwdLabel.setText(QtGui.QApplication.translate("MainWindow", "Forward:", None, QtGui.QApplication.UnicodeUTF8))
        self.turnLabel.setText(QtGui.QApplication.translate("MainWindow", "Turn:", None, QtGui.QApplication.UnicodeUTF8))
        self.fwdVel.setText(QtGui.QApplication.translate("MainWindow", "0.0", None, QtGui.QApplication.UnicodeUTF8))
        self.turnVel.setText(QtGui.QApplication.translate("MainWindow", "0.0", None, QtGui.QApplication.UnicodeUTF8))
        self.label.setText(QtGui.QApplication.translate("MainWindow", "Inc:", None, QtGui.QApplication.UnicodeUTF8))
        self.label_2.setText(QtGui.QApplication.translate("MainWindow", "Inc:", None, QtGui.QApplication.UnicodeUTF8))

