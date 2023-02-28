# import class gui from gui.py
from gui import GUI
import rclpy
from rclpy.node import Node
from rogilink2_interfaces.msg import Frame

# ros node
class SimpleGUI(Node):
    def __init__(self):
        super().__init__('simple_gui')
        self.gui = GUI()
        self.publisher = self.create_publisher(Frame, 'rogilink_send', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)


    def rogilink_publisher(self, hardid, data):
        msg = Frame()
        msg.hardid = hardid
        msg.data = data
        self.publisher.publish(msg)

    def timer_callback(self):
        self.gui.gui_check()

if __name__ == '__main__':
    rclpy.init()
    simple_gui = SimpleGUI()
    rclpy.spin(simple_gui)
    rclpy.shutdown()