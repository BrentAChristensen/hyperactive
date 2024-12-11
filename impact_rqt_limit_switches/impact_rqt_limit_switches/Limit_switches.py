#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rqt_gui_py.plugin import Plugin
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QPushButton, QLabel
from PyQt5.QtCore import QTimer


class ImpactRqtLimitSwitches(Plugin):
    def __init__(self, context):
        super(ImpactRqtLimitSwitches, self).__init__(context)
        self.setObjectName('ImpactRqtLimitSwitches')

        # Initialize rclpy once for the entire plugin
        if not rclpy.ok():
            rclpy.init(args=None)

        # Create the main widget
        self._widget = MyWidget()

        # Add the widget to the user interface
        context.add_widget(self._widget)


class MyWidget(QWidget):
    def __init__(self):
        super(MyWidget, self).__init__()

        print("Monitor Limit Switches constructor called...")

        self.setStyleSheet("background-color: white;")

        # Create a layout
        layout = QVBoxLayout(self)

        # Create a label for text display
        self.label = QLabel('Waiting for Limit Switch response...', self)

        # Add the button and label to the layout
        layout.addWidget(self.label)

        # Set the layout for the widget
        self.setLayout(layout)

        # Use the existing node or create one
        self.node = rclpy.create_node('impact_rqt_limit_switches')

        # ROS 2 Subscription to /joint_states topic
        self.subscription = self.node.create_subscription(
            JointState,
            '/joint_states',
            self.limit_switch_callback,
            10
        )

        # Timer to control update frequency
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_label)
        self.update_timer.start(1000)  # 1000 ms = 1 seconds

        self.current_joint_info = ""

        # Start spinning in a separate thread to avoid blocking the Qt event loop
        from threading import Thread
        spin_thread = Thread(target=rclpy.spin, args=(self.node,))
        spin_thread.start()



    def limit_switch_callback(self, msg):
        # Pair each joint name with its position
        joint_info = zip(msg.name, msg.position)
    
        # Sort the joint info alphabetically by joint name
        sorted_joint_info = sorted(joint_info, key=lambda x: x[0])
    
        # Format the sorted joint information with each joint on a new line
        self.current_joint_info = '\n'.join([f"{name}: {position:.2f}" for name, position in sorted_joint_info])

    def update_label(self):
        # Update the label with the stored joint states information every 5 seconds
        self.label.setText(f'Joint States:\n {self.current_joint_info}')

    def closeEvent(self, event):
        """Handle the Joint State Monitor closing event."""
        self.node.destroy_node()
        rclpy.shutdown()
        super(MyWidget, self).closeEvent(event)