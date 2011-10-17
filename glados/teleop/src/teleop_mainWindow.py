# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'teleop_mainWindow.ui'
#
# Created: Mon Oct 17 11:33:31 2011
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
        MainWindow.resize(247, 103)
        MainWindow.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.stopButton = QtGui.QPushButton(self.centralwidget)
        self.stopButton.setGeometry(QtCore.QRect(80, 70, 80, 25))
        self.stopButton.setFocusPolicy(QtCore.Qt.NoFocus)
        self.stopButton.setStyleSheet(_fromUtf8("background-color: rgb(255, 0, 0);\n"
"font: 75 12pt \"Sans Serif\";\n"
"color: rgb(255, 255, 255);"))
        self.stopButton.setObjectName(_fromUtf8("stopButton"))
        self.fwdLabel = QtGui.QLabel(self.centralwidget)
        self.fwdLabel.setGeometry(QtCore.QRect(0, 30, 91, 16))
        self.fwdLabel.setObjectName(_fromUtf8("fwdLabel"))
        self.turnLabel = QtGui.QLabel(self.centralwidget)
        self.turnLabel.setGeometry(QtCore.QRect(0, 50, 81, 16))
        self.turnLabel.setObjectName(_fromUtf8("turnLabel"))
        self.fwdVelFrac = QtGui.QLabel(self.centralwidget)
        self.fwdVelFrac.setGeometry(QtCore.QRect(60, 30, 31, 16))
        self.fwdVelFrac.setObjectName(_fromUtf8("fwdVelFrac"))
        self.turnVelFrac = QtGui.QLabel(self.centralwidget)
        self.turnVelFrac.setGeometry(QtCore.QRect(60, 50, 31, 16))
        self.turnVelFrac.setObjectName(_fromUtf8("turnVelFrac"))
        self.fwdInc = QtGui.QDoubleSpinBox(self.centralwidget)
        self.fwdInc.setGeometry(QtCore.QRect(175, 26, 51, 22))
        self.fwdInc.setFocusPolicy(QtCore.Qt.ClickFocus)
        self.fwdInc.setKeyboardTracking(True)
        self.fwdInc.setDecimals(1)
        self.fwdInc.setMinimum(0.2)
        self.fwdInc.setMaximum(10.0)
        self.fwdInc.setSingleStep(0.2)
        self.fwdInc.setProperty(_fromUtf8("value"), 0.2)
        self.fwdInc.setObjectName(_fromUtf8("fwdInc"))
        self.turnInc = QtGui.QDoubleSpinBox(self.centralwidget)
        self.turnInc.setGeometry(QtCore.QRect(175, 46, 51, 22))
        self.turnInc.setFocusPolicy(QtCore.Qt.ClickFocus)
        self.turnInc.setDecimals(1)
        self.turnInc.setMinimum(0.2)
        self.turnInc.setMaximum(10.0)
        self.turnInc.setSingleStep(0.2)
        self.turnInc.setProperty(_fromUtf8("value"), 0.2)
        self.turnInc.setObjectName(_fromUtf8("turnInc"))
        self.incLabel = QtGui.QLabel(self.centralwidget)
        self.incLabel.setGeometry(QtCore.QRect(169, 10, 71, 16))
        self.incLabel.setObjectName(_fromUtf8("incLabel"))
        self.fracLabel = QtGui.QLabel(self.centralwidget)
        self.fracLabel.setGeometry(QtCore.QRect(65, 10, 31, 16))
        self.fracLabel.setObjectName(_fromUtf8("fracLabel"))
        self.absLabel = QtGui.QLabel(self.centralwidget)
        self.absLabel.setGeometry(QtCore.QRect(116, 10, 31, 16))
        self.absLabel.setObjectName(_fromUtf8("absLabel"))
        self.fwdVelAbs = QtGui.QLabel(self.centralwidget)
        self.fwdVelAbs.setGeometry(QtCore.QRect(100, 28, 41, 20))
        self.fwdVelAbs.setObjectName(_fromUtf8("fwdVelAbs"))
        self.turnVelAbs = QtGui.QLabel(self.centralwidget)
        self.turnVelAbs.setGeometry(QtCore.QRect(100, 48, 41, 20))
        self.turnVelAbs.setObjectName(_fromUtf8("turnVelAbs"))
        self.turnUnits = QtGui.QLabel(self.centralwidget)
        self.turnUnits.setGeometry(QtCore.QRect(133, 50, 31, 16))
        self.turnUnits.setObjectName(_fromUtf8("turnUnits"))
        self.fwdUnits = QtGui.QLabel(self.centralwidget)
        self.fwdUnits.setGeometry(QtCore.QRect(133, 30, 31, 16))
        self.fwdUnits.setObjectName(_fromUtf8("fwdUnits"))
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QtGui.QApplication.translate("MainWindow", "Teleop", None, QtGui.QApplication.UnicodeUTF8))
        self.stopButton.setText(QtGui.QApplication.translate("MainWindow", "STOP", None, QtGui.QApplication.UnicodeUTF8))
        self.fwdLabel.setText(QtGui.QApplication.translate("MainWindow", "Forward:", None, QtGui.QApplication.UnicodeUTF8))
        self.turnLabel.setText(QtGui.QApplication.translate("MainWindow", "Turn:", None, QtGui.QApplication.UnicodeUTF8))
        self.fwdVelFrac.setText(QtGui.QApplication.translate("MainWindow", "0.0", None, QtGui.QApplication.UnicodeUTF8))
        self.turnVelFrac.setText(QtGui.QApplication.translate("MainWindow", "0.0", None, QtGui.QApplication.UnicodeUTF8))
        self.incLabel.setText(QtGui.QApplication.translate("MainWindow", "Inc (Frac/s)", None, QtGui.QApplication.UnicodeUTF8))
        self.fracLabel.setText(QtGui.QApplication.translate("MainWindow", "Frac", None, QtGui.QApplication.UnicodeUTF8))
        self.absLabel.setText(QtGui.QApplication.translate("MainWindow", "Abs", None, QtGui.QApplication.UnicodeUTF8))
        self.fwdVelAbs.setText(QtGui.QApplication.translate("MainWindow", " 00.00", None, QtGui.QApplication.UnicodeUTF8))
        self.turnVelAbs.setText(QtGui.QApplication.translate("MainWindow", " 00.00", None, QtGui.QApplication.UnicodeUTF8))
        self.turnUnits.setText(QtGui.QApplication.translate("MainWindow", "rad/s", None, QtGui.QApplication.UnicodeUTF8))
        self.fwdUnits.setText(QtGui.QApplication.translate("MainWindow", "cm/s", None, QtGui.QApplication.UnicodeUTF8))

