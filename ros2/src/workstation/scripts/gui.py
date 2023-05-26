#!/usr/bin/env python3
import os, sys
import rclpy
from PyQt5 import QtWidgets
from threading import Thread 
from widget import Ui_Form
from slider_publisher import publisherNode
from rclpy.executors import MultiThreadedExecutor

def main(args=None):
    rclpy.init(args=args)

    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)

    pub_node = publisherNode(ui)
    executor = MultiThreadedExecutor()
    executor.add_node(pub_node)

    # Start the ROS2 node on a separate thread
    thread = Thread(target=executor.spin)
    thread.start()
    pub_node.get_logger().info("Started slider publisher node . . .")

    # Let the app running on the main thread
    try:
        Form.show()
        sys.exit(app.exec_())
    finally:
        
        pub_node.get_logger().info("Shutting slider publisher node . . .")
        pub_node.destroy_node()
        executor.shutdown()

if __name__ == '__main__':
    main()