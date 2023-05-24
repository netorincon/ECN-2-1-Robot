from PyQt5 import QtCore, QtGui, QtWidgets
from qwt.text import QwtTextLabel
import os, signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from control_input.msg import PositionCommand
from control_input.msg import ControlInput
import math 


class publisherNode(Node):
    def __init__(self, ui):
        super().__init__('slider_publisher')
        self.ui = ui
        self.position_publisher = self.create_publisher(PositionCommand, 'position_cmd', 10)
        self.velocity_publisher = self.create_publisher(ControlInput, 'control_cmd', 10)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if(self.ui.manual_speed_mode_radio.isChecked()):   
            msg=ControlInput()
            msg.um=float(self.ui.um_sb.value())
            msg.delta1dot=self.ui.b1d_sb.value()
            msg.delta2dot=self.ui.b2d_sb.value()
            self.velocity_publisher.publish(msg)
            
        elif(self.ui.manual_pos_mode_radio.isChecked()):
            msg=PositionCommand()
            msg.d1=self.ui.beta1_sb.value()*math.pi/180
            msg.d2=self.ui.beta2_sb.value()*math.pi/180 
            self.position_publisher.publish(msg)