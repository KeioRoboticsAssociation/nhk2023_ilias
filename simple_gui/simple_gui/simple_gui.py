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
        self.rogimsg_pub_ = self.create_publisher(Frame, 'rogilink2/send', 10)
        self.state_subscriber = self.create_subscription(String, 'state', self.state_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)


    def rogilink_publisher(self, hard_id, data):
        msg = Frame()
        msg.hard_id = hard_id
        msg.data = data
        self.rogimsg_pub_.publish(msg)

    def timer_callback(self):
        # logger
        self.get_logger().info('Timer callback')
        self.gui.gui_check()
        if self.gui.event != self.prev_event:
            self.prev_event = self.gui.event
            # ros logger
            self.get_logger().info('Event: %s' % self.gui.event)
            self.gui_callback()

    def gui_callback(self):
        if self.gui.event == 'emergency_stop':
            self.rogilink_publisher(0x00,[0,0,0,0,0,0,0,0])
            self.get_logger().error('********EMERGENCY STOP*********')
        elif self.gui.event == 'start':
            self.state_callback('start')
        elif self.gui.event == 'restart':
            self.state_callback('restart')


    def state_callback(self, msg):
        # info
        self.get_logger().info('State: %s' % msg.data)
        # changing the state image
        if msg.data == 'start':
            self.gui.rr_img_change('start')
        elif msg.data == 'restart':
            self.gui.rr_img_change('restart')
        elif msg.data == 'hill_bottom':
            self.gui.rr_img_change('hill_bottom')
        elif msg.data == 'hill_top':
            self.gui.rr_img_change('hill_top')
        elif msg.data == 'angkor':
            self.gui.rr_img_change('angkor')
        elif msg.data == 'angkor_center':
            self.gui.rr_img_change('angkor_center')
        elif msg.data == 'type2_attack':
            self.gui.rr_img_change('type2_attack')
        elif msg.data == 'pole_block':
            self.gui.rr_img_change('pole_block')
        elif msg.data == 'last_attack':
            self.gui.rr_img_change('last_attack')
        elif msg.data == 'end':
            self.gui.rr_img_change('end')

def main(args=None):
    rclpy.init(args=args)
    simple_gui = SimpleGUI()
    rclpy.spin(simple_gui)
    simple_gui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()