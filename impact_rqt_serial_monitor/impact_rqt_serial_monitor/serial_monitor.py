from PyQt5.QtCore import QTimer, pyqtSignal
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QTextEdit
import rclpy
from rclpy.node import Node
from rqt_gui_py.plugin import Plugin
from std_msgs.msg import String

class ImpactRqtSerialMonitor(Plugin):
    def __init__(self, context):
        super(ImpactRqtSerialMonitor, self).__init__(context)

        #print("Serial Monitor constructor called...")
        self.setObjectName('ImpactRqtSerialMonitor')

        # Initialize rclpy once for the entire plugin
        if not rclpy.ok():
            rclpy.init(args=None)

        # Create the main widget
        self._widget = MyWidget()

        # Add the widget to the user interface
        context.add_widget(self._widget)




class MyWidget(QWidget):
    last_message = ""
    def __init__(self):
        super(MyWidget, self).__init__()
        
        print("Serial Monitor constructor called...")

        self.setStyleSheet("background-color: white;")

        layout = QVBoxLayout(self)

        self.text_edit = QTextEdit(self)
        layout.addWidget(self.text_edit)
        self.text_edit.setPlainText("\n \n")
        self.setLayout(layout)
        # Create a label for text display
        self.label = QLabel('Waiting for Serial Communication...', self)

     
     
      # Use the existing node or create one
        self.node = rclpy.create_node('impact_rqt_serial_monitor')

        # ROS 2 Subscription to /joint_states topic
        self.subscription = self.node.create_subscription(
            String,
            'serial_topic',
            self.serial_callback,
            10
        )

        # Timer to control update frequency
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_label)
        self.update_timer.start(1000)  # 1000 ms = 1 seconds


          # Start spinning in a separate thread to avoid blocking the Qt event loop
        from threading import Thread
        spin_thread = Thread(target=rclpy.spin, args=(self.node,))
        spin_thread.start()



    def serial_callback(self, msg):
        new_text = f"{msg.data}\n"
        if self.last_message != new_text:
            self.text_edit.append(new_text)
            self.last_message = new_text

    def update_label(self):
        # Update the label with the stored joint states information every 5 seconds
        self.label.setText(self.last_message)
    