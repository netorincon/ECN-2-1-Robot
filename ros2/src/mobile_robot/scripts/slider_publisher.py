from PyQt5 import QtCore, QtGui, QtWidgets
from qwt.text import QwtTextLabel
import os, signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from control_input.msg import SliderCommand
import math 


class publisherNode(Node):
    def __init__(self, ui):
        super().__init__('slider_publisher')
        self.ui = ui
        self.slider_publisher = self.create_publisher(SliderCommand, 'slider_cmd', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg=SliderCommand()
        msg.cmd.name=["delta1", "delta2"]
        if(self.ui.manual_speed_mode_radio.isChecked()):   
            msg.mode="velocity"
            msg.um=float(self.ui.um_sb.value())
            msg.cmd.velocity.append(self.ui.b1d_sb.value())
            msg.cmd.velocity.append(self.ui.b2d_sb.value())
            
        elif(self.ui.manual_pos_mode_radio.isChecked()):
            msg.mode="position"
            msg.cmd.position.append(self.ui.beta1_sb.value()*math.pi/180)
            msg.cmd.position.append(self.ui.beta2_sb.value()*math.pi/180)  

        self.slider_publisher.publish(msg)