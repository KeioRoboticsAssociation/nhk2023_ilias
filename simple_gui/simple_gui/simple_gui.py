# import class gui from gui.py
from simple_gui.gui import GUI
import rclpy
from rclpy.node import Node
from rogilink2_interfaces.msg import Frame
from std_msgs.msg import String

# ros node
class SimpleGUI(Node):
    prev_event = None

    def __init__(self):
        super().__init__('simple_gui')
        self.gui = GUI()
        self.rogilink_publisher = self.create_publisher(Frame, 'rogilink_send', 10)
        self.state_subscriber = self.create_subscription(String, 'state', self.state_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)


    def rogilink_publisher(self, hardid, data):
        msg = Frame()
        msg.hardid = hardid
        msg.data = data
        self.rogilink_publisher.publish(msg)

    def timer_callback(self):
        # logger
        self.get_logger().info('Timer callback')
        self.gui.gui_check()
        if self.gui.event != self.prev_event:
            self.prev_event = self.gui.event
            # ros logger
            self.get_logger().info('Event: %s' % self.gui.event)

    def gui_callback(self):
        if self.gui.event == 'emergency_stop':
            self.rogilink_publisher(0x00,0)

    def state_callback(self, msg):
        # info
        self.get_logger().info('State: %s' % msg.data)
        # changing the state image
        if msg.data == 'start':
            self.gui.rr_img_change('start')
        elif msg.data == 'hill_bottom':
            self.gui.rr_img_change('hill_bottom')

def main(args=None):
    rclpy.init(args=args)
    simple_gui = SimpleGUI()
    rclpy.spin(simple_gui)
    simple_gui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()