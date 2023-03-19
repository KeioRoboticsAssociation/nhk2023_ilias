# import class gui from gui.py
from simple_gui.gui import GUI
import rclpy
from rclpy.node import Node
from rogilink2_interfaces.msg import Frame
from std_msgs.msg import String
from rogilink2_interfaces.msg import Ping
from pure_pursuit_interface.msg import Frame as PurePursuitFrame

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
        self.ping_pub_ = self.create_subscription(Ping, 'rogilink2/ping', self.ping_callback, 10)
        self.rr_path_pub_ = self.create_publisher(PurePursuitFrame, 'pp_cmd', 10)

    def ping_callback(self, msg):
        self.get_logger().info('Ping callback')
        self.gui.ping_gui(msg)

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
            msg = String()
            msg.data = 'START'
            self.state_toggle_pub_.publish(msg)
            self.get_logger().info('********RR_START*********')
        elif self.gui.event == 'rr_restart':
            msg = String()
            msg.data = 'RESTART'
            self.state_toggle_pub_.publish(msg)
            self.get_logger().info('********RR_RESTART*********')
        elif self.gui.event == 'rr_idle':
            msg = String()
            msg.data = 'IDLE'
            self.state_toggle_pub_.publish(msg)
            self.get_logger().info('********RR_IDLE*********')
        elif self.gui.event == 'rr_manual':
            msg = String()
            msg.data = 'MANUAL'
            self.state_toggle_pub_.publish(msg)
            self.get_logger().info('********RR_MANUAL*********')
        elif self.gui.event == 'rr_forward':
            msg = String()
            msg.data = 'FORWARD'
            self.state_toggle_pub_.publish(msg)
            self.get_logger().info('********RR_FORWARD*********')
        elif self.gui.event == 'rr_set_state':
            msg = String()
            msg.data = f'{self.gui.values["rr_state_select"].upper()}'
            self.state_toggle_pub_.publish(msg)
            self.get_logger().info('********RR SET STATE*********')
        elif self.gui.event == 'rr_path_forward':
            msg = PurePursuitFrame()
            msg.forward_flag = True
            msg.path_num = int(self.gui.values['rr_path'])
            msg.is_allowed_to_pub = True
            self.rr_path_pub_.publish(msg)
            self.get_logger().info('********RR PATH FORWARD*********')
        elif self.gui.event == 'rr_path_backward':
            msg = PurePursuitFrame()
            msg.forward_flag = False
            msg.path_num = int(self.gui.values['rr_path'])
            msg.is_allowed_to_pub = True
            self.rr_path_pub_.publish(msg)
            self.get_logger().info('********RR PATH BACKWARD*********')



    def state_callback(self, msg):
        self.get_logger().info('State callback')
        # changing the state image
        if msg.data == 'START':
            self.gui.rr_img_change('start')
        elif msg.data == 'RESTART':
            self.gui.rr_img_change('restart')
        elif msg.data == 'IDLE':
            self.gui.rr_img_change('idle')
        elif msg.data == 'MANUAL':
            self.gui.rr_img_change('manual')
        elif msg.data == 'HILL_BOTTOM':
            self.gui.rr_img_change('hill_bottom')
        elif msg.data == 'HILL_TOP':
            self.gui.rr_img_change('hill_top')
        elif msg.data == 'ANGKOR':
            self.gui.rr_img_change('angkor')
        elif msg.data == 'ANGKOR_CENTER':
            self.gui.rr_img_change('angkor_center')
        elif msg.data == 'TYPE2_ATTACK':
            self.gui.rr_img_change('type2_attack')
        elif msg.data == 'POLE_BLOCK':
            self.gui.rr_img_change('pole_block')
        elif msg.data == 'LAST_ATTACK':
            self.gui.rr_img_change('last_attack')
        elif msg.data == 'END':
            self.gui.rr_img_change('end')

def main(args=None):
    rclpy.init(args=args)
    simple_gui = SimpleGUI()
    rclpy.spin(simple_gui)
    simple_gui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()