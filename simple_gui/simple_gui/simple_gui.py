# import class gui from gui.py
from gui import GUI
import rclpy
from rclpy.node import Node
from rogilink2_interfaces.msg import Frame

# ros node
class SimpleGUI(Node):
    prev_event = None

    def __init__(self):
        super().__init__('simple_gui')
        self.gui = GUI()
        self.rogilink_publisher = self.create_publisher(Frame, 'rogilink_send', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)


    def rogilink_publisher(self, hardid, data):
        msg = Frame()
        msg.hardid = hardid
        msg.data = data
        self.rogilink_publisher.publish(msg)

    def timer_callback(self):
        self.gui.gui_check()
        if self.gui.event != self.prev_event:
            self.prev_event = self.gui.event
            # ros logger
            self.get_logger().info('Event: %s' % self.gui.event)

    def gui_callback(self):
        if self.gui.event == 'emergency_stop':
            self.rogilink_publisher(0x00,0)
        


if __name__ == '__main__':
    rclpy.init()
    simple_gui = SimpleGUI()
    rclpy.spin(simple_gui)
    rclpy.shutdown()