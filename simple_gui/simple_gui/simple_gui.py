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
        self.state_toggle_pub_ = self.create_publisher(String, 'state_toggle', 10)


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
        self.get_logger().info('GUI callback')
        if self.gui.event == 'emergency_stop':
            self.rogilink_publisher(0x00,[0,0,0,0,0,0,0,0])
            self.get_logger().error('********EMERGENCY STOP*********')
        elif self.gui.event == 'rr_start':
            self.state_callback('start')
            self.get_logger().info('********START*********')
        elif self.gui.event == 'rr_restart':
            self.state_callback('restart')
            self.get_logger().info('********RESTART*********')
        elif self.gui.event == 'rr_idle':
            self.state_callback('idle')
            self.get_logger().info('********IDLE*********')
        elif self.gui.event == 'rr_manual':
            self.state_callback('manual')
            self.get_logger().info('********MANUAL*********')
        elif self.gui.event == 'rr_foward':
            msg = String()
            msg.data = 'foward'
            self.state_toggle_pub_.publish(msg)
            self.get_logger().info('********FOWARD*********')


    def state_callback(self, msg):
        # changing the state image
        if msg == 'start':
            self.gui.rr_img_change('start')
        elif msg == 'restart':
            self.gui.rr_img_change('restart')
        elif msg == 'hill_bottom':
            self.gui.rr_img_change('hill_bottom')
        elif msg == 'hill_top':
            self.gui.rr_img_change('hill_top')
        elif msg == 'angkor':
            self.gui.rr_img_change('angkor')
        elif msg == 'angkor_center':
            self.gui.rr_img_change('angkor_center')
        elif msg == 'type2_attack':
            self.gui.rr_img_change('type2_attack')
        elif msg == 'pole_block':
            self.gui.rr_img_change('pole_block')
        elif msg == 'last_attack':
            self.gui.rr_img_change('last_attack')
        elif msg == 'end':
            self.gui.rr_img_change('end')
        elif msg == 'manual':
            self.gui.rr_img_change('manual')
        elif msg == 'idle':
            self.gui.rr_img_change('idle')

def main(args=None):
    rclpy.init(args=args)
    simple_gui = SimpleGUI()
    rclpy.spin(simple_gui)
    simple_gui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()