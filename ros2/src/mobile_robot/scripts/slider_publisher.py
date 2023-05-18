from PyQt5 import QtCore, QtGui, QtWidgets
from qwt.text import QwtTextLabel
import os, signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from widget import Ui_Form
from control_input.msg import ControlInput


class publisherNode(Node):
    def __init__(self, ui):
        super().__init__('minimal_publisher')
        self.ui = ui
        self.publisher_ = self.create_publisher(ControlInput, 'cmd_robot', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = ControlInput()
        msg.um=float(self.ui.umSlider.value()/100)
        msg.beta1dot=float(self.ui.b1d_slider.value()/500)
        msg.beta2dot=float(self.ui.b2d_slider.value()/500)
        if(self.ui.manual_speed_mode_radio.isChecked()):
            self.publisher_.publish(msg)
