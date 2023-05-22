from PyQt5 import QtCore, QtGui, QtWidgets
from qwt.text import QwtTextLabel
import os, signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from control_input.msg import ControlInput
from control_input.msg import PositionCommand
import math 


class publisherNode(Node):
    def __init__(self, ui):
        super().__init__('slider_publisher')
        self.ui = ui
        self.vel_publisher = self.create_publisher(ControlInput, 'vel_cmd', 10)
        self.pos_publisher = self.create_publisher(PositionCommand, 'pos_cmd', 10)
        self.mode_publisher = self.create_publisher(String, 'mode', 10)
        timer_period = 0.05  # seconds
        self.modeMsg=String()
        self.mode=""
        self.previousMode=""
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.changeMode()
        if(self.ui.manual_speed_mode_radio.isChecked()):   
            msg = ControlInput()
            msg.um=float(self.ui.um_sb.value())
            msg.beta1dot=float(self.ui.b1d_sb.value())
            msg.beta2dot=float(self.ui.b2d_sb.value())
            self.vel_publisher.publish(msg)

        elif(self.ui.manual_pos_mode_radio.isChecked()):
            msg=PositionCommand()
            msg.d1=self.ui.beta1_sb.value()*math.pi/180
            msg.d2=self.ui.beta2_sb.value()*math.pi/180
            self.pos_publisher.publish(msg)

    def changeMode(self):
        if(self.ui.manual_speed_mode_radio.isChecked()):
            self.mode="speed"
        elif(self.ui.manual_pos_mode_radio.isChecked()):
            self.mode="position"
        else:
            self.mode="speed"
        if(self.previousMode!=self.mode):
            self.previousMode=self.mode        
            self.modeMsg.data=self.mode
        self.mode_publisher.publish(self.modeMsg)
        return
