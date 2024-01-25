#!/usr/bin/env python

from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QLabel, QWidget
from PyQt5.QtGui import QIcon, QPixmap
from PyQt5.QtCore import QTimer
import sys
import rclpy
from std_msgs.msg import Float32
import numpy as np
import pygame

class PlotHandler(QWidget):
    def __init__(self, vehicle):
        super(PlotHandler, self).__init__()

        self.vehicle = vehicle
        self.setWindowTitle("Teleop plotter")
        self.setGeometry(100, 100, 800, 600)
        self.setWindowIcon(QIcon('./icon02.png'))  

        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)
        layout = QVBoxLayout(self.central_widget)

        self.status_label = QLabel("No data\n\n", self)
        self.status_label.setStyleSheet("font-family: Monospace; font: 30pt; background-color: rgb(0, 0, 0)")
        layout.addWidget(self.status_label)

        self.initialize_plots()

    def initialize_plots(self):
        # Add your plot initialization code here
        pass

    def update_slow(self):
        # Add your slow update code here
        pass

    def update_fast(self):
        # Add your fast update code here
        pass

    def update_labels(self):
        # Add your label update code here
        pass


class TeleopSub:
    def __init__(self):
        self.actual_speed = -1
        self.ref_speed = -1
        self.wheel_actual_rad = -0.01
        self.wheel_gamepa_rad = 0.01
        self.leaf_is_autonomous = "UNDEF"

    def speed_kmph_callback(self, msg_kmph):
        self.actual_speed = msg_kmph.data

    def wheel_deg_callback(self, msg_deg):
        self.wheel_actual_rad = np.deg2rad(msg_deg.data) * 19.68

    def vehicle_status_callback(self, msg):
        self.leaf_is_autonomous = {
            0: "IN CAR DRIVER",
            1: "YOU DRIVE",
        }.get(msg.drivemode, "UNDEF")

    def vehicle_ctrl_callback(self, msg_ctrl):
        self.ref_speed = msg_ctrl.cmd.linear_velocity
        self.wheel_gamepa_rad = msg_ctrl.cmd.steering_angle * 19.68


def main():
    rclpy.init()
    node = rclpy.create_node("steering_visualization")

    veh_sub = TeleopSub()

    ph = PlotHandler(veh_sub)
    
    ph.show()

    timer_slow = QTimer(ph)
    timer_slow.timeout.connect(ph.update_slow)
    timer_slow.start(600)

    timer_fast = QTimer(ph)
    timer_fast.timeout.connect(ph.update_fast)
    timer_fast.start(30)

    timer_labels = QTimer(ph)
    timer_labels.timeout.connect(ph.update_labels)
    timer_labels.start(30)

    node.create_subscription(Float32, "/wheel_angle_deg", veh_sub.wheel_deg_callback, 10)
    node.create_subscription(Float32, "/vehicle_speed_kmph", veh_sub.speed_kmph_callback, 10)
    # node.create_subscription(autow_msgs.VehicleStatus, "/vehicle_status", veh_sub.vehicle_status_callback, 10)
    # node.create_subscription(autow_msgs.ControlCommandStamped, "/ctrl_cmd", veh_sub.vehicle_ctrl_callback, 10)

    rclpy.spin(node)

    rclpy.shutdown()
    sys.exit(ph.app.exec_())


if __name__ == "__main__":
    main()
