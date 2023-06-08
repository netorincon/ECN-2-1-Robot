# -*- coding: utf-8 -*-
from PyQt5 import QtCore, QtGui, QtWidgets
from qwt.text import QwtTextLabel
import os, signal
import time

windowHeight=680
windowWidth=420


class Ui_Form(object):
    end=False
    def launchFile(self):
        #Check radio button state
        manual_speed_mode=self.manual_speed_mode_radio.isChecked()
        manual_pos_mode=self.manual_pos_mode_radio.isChecked()
        controller_mode=self.control_mode_radio.isChecked()
        self.mode=""
        if(manual_speed_mode):
            self.mode="mode:="+"velocity"
        elif(manual_pos_mode):
            self.mode="mode:="+"position"
        elif(controller_mode):
            self.mode="mode:="+"controller"

        if(self.launchP.processId()!=0):
            os.kill(self.launchP.processId(), signal.SIGINT)
            self.launchP.waitForFinished(-1)

        if(self.simulation_radio.isChecked()):
            self.launchP.start("ros2", ["launch", "workstation", "simulation.launch.py", self.mode])
        else:
            self.launchP.start("ros2", ["launch", "workstation", "real_world.launch.py", self.mode])

        self.umSlider.setEnabled(manual_speed_mode)
        self.b1d_slider.setEnabled(manual_speed_mode)
        self.b2d_slider.setEnabled(manual_speed_mode)
        self.reset_speed_button.setEnabled(manual_speed_mode)
        
        self.b1_slider.setEnabled(manual_pos_mode)
        self.b2_slider.setEnabled(manual_pos_mode)
        self.reset_pos_button.setEnabled(manual_pos_mode)

        self.simulation_radio.setDisabled(True)
        self.real_world_radio.setDisabled(True)
        self.launch_button.setDisabled(True)
        self.control_mode_radio.setDisabled(True)
        self.manual_speed_mode_radio.setDisabled(True)
        self.manual_pos_mode_radio.setDisabled(True)
        self.end_button.setEnabled(True)
        return
    
    def endLaunch(self):
        self.end=True
        self.reset_pos_sliders()
        self.reset_speed_sliders()
        
        if(self.launchP.processId()!=0):
            os.kill(self.launchP.processId(), signal.SIGINT)
            self.launchP.waitForFinished(-1)
            os.system("ros2 service call /reset_robot_state")
        time.sleep(0.5)
        self.simulation_radio.setDisabled(False)
        self.real_world_radio.setDisabled(False)
        self.launch_button.setDisabled(False)
        self.end_button.setDisabled(True)
        self.control_mode_radio.setEnabled(True)
        self.manual_speed_mode_radio.setEnabled(True)
        self.manual_pos_mode_radio.setEnabled(True)
        self.end=False

        return
    
    def exit(self):
        self.endLaunch()
        return
    
    def reset_speed_sliders(self):
        self.umSlider.setProperty("value", 0)
        self.um_sb.setValue(0)
        self.b1d_slider.setProperty("value", 0)
        self.b1d_sb.setValue(0)
        self.b2d_slider.setProperty("value", 0)
        self.b2d_sb.setValue(0)
        return
        
    def reset_pos_sliders(self):
        self.b1_slider.setProperty("value", 0)
        self.beta1_sb.setValue(0)
        self.b2_slider.setProperty("value", 0)
        self.beta2_sb.setValue(0)
        return
    
    def updateBeta1Slider(self):
        self.b1_slider.setValue(int(self.beta1_sb.value()))
        return
    
    def updateBeta1Box(self):
        self.beta1_sb.setValue(self.b1_slider.value())
        return

    def updateBeta2Slider(self):
        self.b2_slider.setValue(int(self.beta2_sb.value()))
        return
    
    def updateBeta2Box(self):
        self.beta2_sb.setValue(self.b2_slider.value())
        return
    
    def updateUmSlider(self):
        self.umSlider.setValue(self.um_sb.value()*100)
        return
    
    def updateUmBox(self):
        self.um_sb.setValue(self.umSlider.value()/100)
        return
    
    def updateB1dSlider(self):
        self.b1d_slider.setValue(self.b1d_sb.value())
        return
    
    def updateB1dBox(self):
        self.b1d_sb.setValue(self.b1d_slider.value())
        return
    
    def updateB2dSlider(self):
        self.b2d_slider.setValue(self.b2d_sb.value())
        return
    
    def updateB2dBox(self):
        self.b2d_sb.setValue(self.b2d_slider.value())
        return
    
    def switchToPositionMode(self):
        self.b1_slider.setEnabled(True)
        self.b2_slider.setEnabled(True)
        self.reset_pos_button.setEnabled(True)
        self.umSlider.setDisabled(True)
        self.b1d_slider.setDisabled(True)
        self.b2d_slider.setDisabled(True)
        self.reset_speed_button.setDisabled(True)
        self.beta1_sb.setEnabled(True)
        self.beta2_sb.setEnabled(True)
        self.um_sb.setDisabled(True)
        self.b1d_sb.setDisabled(True)
        self.b2d_sb.setDisabled(True)
        if(not self.launch_button.isEnabled() and not self.end_button.isEnabled()):          
            self.simulation_radio.setEnabled(True)
            self.real_world_radio.setEnabled(True)
        return
    
    def switchToSpeedMode(self):
        self.b1_slider.setDisabled(True)
        self.b2_slider.setDisabled(True)
        self.reset_pos_button.setDisabled(True)
        self.umSlider.setEnabled(True)
        self.b1d_slider.setEnabled(True)
        self.b2d_slider.setEnabled(True)
        self.reset_speed_button.setEnabled(True)
        self.beta1_sb.setDisabled(True)
        self.beta2_sb.setDisabled(True)
        self.um_sb.setEnabled(True)
        self.b1d_sb.setEnabled(True)
        self.b2d_sb.setEnabled(True)
        if(not self.launch_button.isEnabled() and not self.end_button.isEnabled()):          
            self.simulation_radio.setEnabled(True)
            self.real_world_radio.setEnabled(True)
        return

    def switchToControllerMode(self):
        self.b1_slider.setDisabled(True)
        self.b2_slider.setDisabled(True)
        self.reset_pos_button.setDisabled(True)
        self.umSlider.setDisabled(True)
        self.b1d_slider.setDisabled(True)
        self.b2d_slider.setDisabled(True)
        self.reset_speed_button.setDisabled(True)
        self.beta1_sb.setDisabled(True)
        self.beta2_sb.setDisabled(True)
        self.um_sb.setDisabled(True)
        self.b1d_sb.setDisabled(True)
        self.b2d_sb.setDisabled(True)
        if(not self.launch_button.isEnabled() and not self.end_button.isEnabled()):          
            self.simulation_radio.setEnabled(True)
            self.real_world_radio.setEnabled(True)
        return

    def setupUi(self, Form):
        self.launchP=QtCore.QProcess()
        Form.setObjectName("Form")
        Form.resize(windowWidth, windowHeight)
        Form.setWindowFlags(QtCore.Qt.WindowStaysOnTopHint)
        self.frame = QtWidgets.QFrame(Form)
        self.frame.setGeometry(QtCore.QRect(10, 10, windowWidth-20, windowHeight-60))
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.gridLayoutWidget = QtWidgets.QWidget(self.frame)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(0, 0, windowWidth-20, windowHeight-65))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.gridLayout = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setContentsMargins(10, 10, 10, 0)
        self.gridLayout.setObjectName("gridLayout")
        self.ControlModeGroup = QtWidgets.QButtonGroup(Form)
        self.ControlModeGroup.setObjectName("ControlModeGroup")

        self.control_mode_radio = QtWidgets.QRadioButton(self.gridLayoutWidget)
        self.control_mode_radio.setCheckable(True)
        self.control_mode_radio.setChecked(False)
        self.control_mode_radio.setObjectName("control_mode_radio")
        self.ControlModeGroup.addButton(self.control_mode_radio)
        self.gridLayout.addWidget(self.control_mode_radio, 1, 0, 1, 1)

        self.line = QtWidgets.QFrame(self.gridLayoutWidget)
        self.line.setFrameShadow(QtWidgets.QFrame.Raised)
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setObjectName("line")
        self.gridLayout.addWidget(self.line, 2, 0, 1, 1)

        self.manual_pos_block=QtWidgets.QVBoxLayout()
        self.manual_pos_block.setContentsMargins(0, 10, 0, 10)
        self.manual_pos_block.setObjectName("manual_pos_block")
        self.manual_pos_mode_radio = QtWidgets.QRadioButton(self.gridLayoutWidget)
        self.manual_pos_mode_radio.setObjectName("manual_pos_mode_radio")
        self.ControlModeGroup.addButton(self.manual_pos_mode_radio)
        self.manual_pos_block.addWidget(self.manual_pos_mode_radio) 
        self.beta1text = QwtTextLabel(self.gridLayoutWidget)
        self.beta1text.setObjectName("beta1text")  
        self.manual_pos_block.addWidget(self.beta1text)

        self.beta1_control=QtWidgets.QHBoxLayout()
        self.beta1_sb=QtWidgets.QDoubleSpinBox(self.gridLayoutWidget)
        self.beta1_sb.setRange(-180, 179)
        self.beta1_sb.setSingleStep(1)
        self.beta1_sb.setDisabled(True)
        self.beta1_control.addWidget(self.beta1_sb)

        self.b1_slider = QtWidgets.QSlider(self.gridLayoutWidget)
        self.b1_slider.setEnabled(False)
        self.b1_slider.setMinimum(-180)
        self.b1_slider.setMaximum(179)
        self.b1_slider.setProperty("value", 0)
        self.b1_slider.setOrientation(QtCore.Qt.Horizontal)
        self.b1_slider.setObjectName("b1_slider")
        self.beta1_control.addWidget(self.b1_slider)
        self.manual_pos_block.addLayout(self.beta1_control)

        self.beta2text = QwtTextLabel(self.gridLayoutWidget)
        self.beta2text.setObjectName("beta2text")    
        self.manual_pos_block.addWidget(self.beta2text)
        self.beta2_control=QtWidgets.QHBoxLayout()
        self.beta2_control.setObjectName("beta2_control") 
        self.beta2_sb=QtWidgets.QDoubleSpinBox(self.gridLayoutWidget)
        self.beta2_sb.setObjectName("beta2_sb") 
        self.beta2_sb.setRange(-180, 179)
        self.beta2_sb.setSingleStep(1)
        self.beta2_sb.setDisabled(True)
        self.beta2_control.addWidget(self.beta2_sb,)
        self.b2_slider = QtWidgets.QSlider(self.gridLayoutWidget)
        self.b2_slider.setEnabled(False)
        self.b2_slider.setMinimum(-180)
        self.b2_slider.setMaximum(179)
        self.b2_slider.setProperty("value", 0)
        self.b2_slider.setOrientation(QtCore.Qt.Horizontal)
        self.b2_slider.setObjectName("b2_slider")
        self.beta2_control.addWidget(self.b2_slider)
        self.manual_pos_block.addLayout(self.beta2_control)
        self.reset_pos_button = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.reset_pos_button.setEnabled(False)
        self.reset_pos_button.setObjectName("reset_pos_button")
        self.manual_pos_block.addWidget(self.reset_pos_button)
        self.gridLayout.addLayout(self.manual_pos_block, 3, 0, 1, 1)

        self.line_2 = QtWidgets.QFrame(self.gridLayoutWidget)
        self.line_2.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_2.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_2.setObjectName("line_2")
        self.gridLayout.addWidget(self.line_2, 4, 0, 1, 1)

        self.manual_control_block=QtWidgets.QVBoxLayout()
        self.manual_control_block.setContentsMargins(0, 10, 0, 10)
        self.manual_control_block.setObjectName("manual_control_block")
        self.manual_speed_mode_radio = QtWidgets.QRadioButton(self.gridLayoutWidget)
        self.manual_speed_mode_radio.setObjectName("manual_speed_mode_radio")
        self.ControlModeGroup.addButton(self.manual_speed_mode_radio)
        self.manual_control_block.addWidget(self.manual_speed_mode_radio)
        self.TextLabel = QwtTextLabel(self.gridLayoutWidget)
        self.TextLabel.setObjectName("TextLabel")
        self.manual_control_block.addWidget(self.TextLabel)

        self.um_control=QtWidgets.QHBoxLayout()
        self.um_sb=QtWidgets.QDoubleSpinBox(self.gridLayoutWidget)
        self.um_sb.setRange(-1, 1)
        self.um_sb.setSingleStep(0.01)
        self.um_sb.setDisabled(True)
        self.um_control.addWidget(self.um_sb)

        self.umSlider = QtWidgets.QSlider(self.gridLayoutWidget)
        self.umSlider.setEnabled(False)
        self.umSlider.setMinimum(-100)
        self.umSlider.setMaximum(100)
        self.umSlider.setProperty("value", 0)
        self.umSlider.setOrientation(QtCore.Qt.Horizontal)
        self.umSlider.setObjectName("umSlider")
        self.um_control.addWidget(self.umSlider)
        self.manual_control_block.addLayout(self.um_control)
        
        self.beta1dot = QwtTextLabel(self.gridLayoutWidget)
        self.beta1dot.setObjectName("beta1dot")
        self.manual_control_block.addWidget(self.beta1dot)
        self.b1d_control=QtWidgets.QHBoxLayout()
        self.b1d_sb=QtWidgets.QDoubleSpinBox(self.gridLayoutWidget)
        self.b1d_sb.setRange(-180, 179)
        self.b1d_sb.setSingleStep(1)
        self.b1d_sb.setDisabled(True)
        self.b1d_control.addWidget(self.b1d_sb)
        self.b1d_slider = QtWidgets.QSlider(self.gridLayoutWidget)
        self.b1d_slider.setEnabled(False)
        self.b1d_slider.setMinimum(-180)
        self.b1d_slider.setMaximum(179)
        self.b1d_slider.setPageStep(10)
        self.b1d_slider.setProperty("value", 0)
        self.b1d_slider.setOrientation(QtCore.Qt.Horizontal)
        self.b1d_slider.setTickPosition(QtWidgets.QSlider.NoTicks)
        self.b1d_slider.setObjectName("b1d_slider")
        self.b1d_control.addWidget(self.b1d_slider)
        self.manual_control_block.addLayout(self.b1d_control)

        self.beta2dot = QwtTextLabel(self.gridLayoutWidget)
        self.beta2dot.setObjectName("beta2dot")
        self.manual_control_block.addWidget(self.beta2dot)
        self.b2d_control=QtWidgets.QHBoxLayout()
        self.b2d_sb=QtWidgets.QDoubleSpinBox(self.gridLayoutWidget)
        self.b2d_sb.setRange(-180, 179)
        self.b2d_sb.setSingleStep(1)
        self.b2d_sb.setDisabled(True)
        self.b2d_control.addWidget(self.b2d_sb)
        self.b2d_slider = QtWidgets.QSlider(self.gridLayoutWidget)
        self.b2d_slider.setEnabled(False)
        self.b2d_slider.setMinimum(-180)
        self.b2d_slider.setMaximum(179)
        self.b2d_slider.setSliderPosition(0)
        self.b2d_slider.setOrientation(QtCore.Qt.Horizontal)
        self.b2d_slider.setObjectName("b2d_slider")
        self.b2d_control.addWidget(self.b2d_slider)
        self.manual_control_block.addLayout(self.b2d_control)
        self.reset_speed_button = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.reset_speed_button.setEnabled(False)
        self.reset_speed_button.setObjectName("reset_speed_button")
        self.manual_control_block.addWidget(self.reset_speed_button)
        self.gridLayout.addLayout(self.manual_control_block, 5, 0, 1, 1)

        self.line_3 = QtWidgets.QFrame(self.gridLayoutWidget)
        self.line_3.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_3.setFrameShadow(QtWidgets.QFrame.Raised)
        self.line_3.setObjectName("line_3")
        self.line_3.setLineWidth(2)
        self.gridLayout.addWidget(self.line_3, 6, 0, 1, 1)
        
        self.runtimeLayout=QtWidgets.QVBoxLayout()
        self.runtimeLayout.setContentsMargins(0, 10, 0, -1)
        self.runtimeLayout.setObjectName("runtimeLayout")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.Runtime = QwtTextLabel(self.gridLayoutWidget)
        self.Runtime.setEnabled(False)
        self.Runtime.setObjectName("Runtime")
        self.horizontalLayout.addWidget(self.Runtime)
        self.simulation_radio = QtWidgets.QRadioButton(self.gridLayoutWidget)
        self.simulation_radio.setEnabled(False)
        self.simulation_radio.setCheckable(True)
        self.simulation_radio.setChecked(False)
        self.simulation_radio.setObjectName("simulation_radio")
        self.RuntimeGroup = QtWidgets.QButtonGroup(Form)
        self.RuntimeGroup.setObjectName("RuntimeGroup")
        self.RuntimeGroup.addButton(self.simulation_radio)
        self.horizontalLayout.addWidget(self.simulation_radio)
        self.real_world_radio = QtWidgets.QRadioButton(self.gridLayoutWidget)
        self.real_world_radio.setEnabled(False)
        self.real_world_radio.setObjectName("real_world_radio")
        self.RuntimeGroup.addButton(self.real_world_radio)
        self.horizontalLayout.addWidget(self.real_world_radio)
        self.launch_button = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.launch_button.setEnabled(False)
        self.launch_button.setObjectName("launch_button")
        self.end_button = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.end_button.setEnabled(False)
        self.end_button.setObjectName("end_button")
        self.runtimeLayout.addLayout(self.horizontalLayout)
        self.runtimeLayout.addWidget(self.launch_button)
        self.runtimeLayout.addWidget(self.end_button)
        self.gridLayout.addLayout(self.runtimeLayout,7, 0, 1, 1)
   
        self.close_button = QtWidgets.QPushButton(Form)
        self.close_button.setGeometry(QtCore.QRect(windowWidth-100, windowHeight-40, 89, 25))
        self.close_button.setObjectName("close_button")

        self.retranslateUi(Form)
        self.manual_pos_mode_radio.clicked['bool'].connect(self.switchToPositionMode)
        self.manual_speed_mode_radio.clicked['bool'].connect(self.switchToSpeedMode)
        self.control_mode_radio.clicked['bool'].connect(self.switchToControllerMode)

        self.simulation_radio.clicked['bool'].connect(self.launch_button.setEnabled)
        self.real_world_radio.clicked['bool'].connect(self.launch_button.setEnabled)
        self.launch_button.clicked['bool'].connect(self.simulation_radio.setDisabled)
        self.launch_button.clicked['bool'].connect(self.real_world_radio.setDisabled)
        self.launch_button.clicked['bool'].connect(self.launchFile)
        self.reset_speed_button.clicked['bool'].connect(self.reset_speed_sliders)
        self.reset_pos_button.clicked['bool'].connect(self.reset_pos_sliders)
        self.end_button.clicked['bool'].connect(self.endLaunch)
        self.close_button.clicked['bool'].connect(self.exit)
        self.close_button.clicked['bool'].connect(Form.close)

        self.beta1_sb.valueChanged['double'].connect(self.updateBeta1Slider)
        self.b1_slider.valueChanged['int'].connect(self.updateBeta1Box)

        self.beta2_sb.valueChanged['double'].connect(self.updateBeta2Slider)
        self.b2_slider.valueChanged['int'].connect(self.updateBeta2Box)

        self.um_sb.valueChanged['double'].connect(self.updateUmSlider)
        self.umSlider.valueChanged['int'].connect(self.updateUmBox)

        self.b1d_sb.valueChanged['double'].connect(self.updateB1dSlider)
        self.b1d_slider.valueChanged['int'].connect(self.updateB1dBox)

        self.b2d_sb.valueChanged['double'].connect(self.updateB2dSlider)
        self.b2d_slider.valueChanged['int'].connect(self.updateB2dBox)
                
        QtCore.QMetaObject.connectSlotsByName(Form)
        Form.setTabOrder(self.control_mode_radio, self.simulation_radio)
        Form.setTabOrder(self.simulation_radio, self.real_world_radio)
        Form.setTabOrder(self.real_world_radio, self.manual_speed_mode_radio)
        Form.setTabOrder(self.manual_speed_mode_radio, self.umSlider)
        Form.setTabOrder(self.umSlider, self.b1d_slider)
        Form.setTabOrder(self.b1d_slider, self.close_button)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "1-2 robot control GUI"))
        self.manual_speed_mode_radio.setText(_translate("Form", "Manual speed mode"))
        self.manual_pos_mode_radio.setText(_translate("Form", "Manual position mode"))
        self.Runtime.setPlainText(_translate("Form", "Environment: "))
        self.simulation_radio.setText(_translate("Form", "Simulation"))
        self.real_world_radio.setText(_translate("Form", "Real world"))
        self.TextLabel.setPlainText(_translate("Form", "Um control input"))
        self.beta1dot.setPlainText(_translate("Form", "Delta 1"))
        self.beta2dot.setPlainText(_translate("Form", "Delta 2"))
        self.beta1text.setPlainText(_translate("Form", "Delta 1"))
        self.beta2text.setPlainText(_translate("Form", "Delta 2"))
        self.reset_speed_button.setText(_translate("Form", "Reset sliders"))
        self.reset_pos_button.setText(_translate("Form", "Reset sliders"))
        self.control_mode_radio.setText(_translate("Form", "Controller mode"))
        self.launch_button.setText(_translate("Form", "Launch"))
        self.end_button.setText(_translate("Form", "End"))
        self.close_button.setText(_translate("Form", "Close"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())
