import tkinter
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class LocalGui:
    def __init__(self):
        self.root = tkinter.Tk()
        self.root.title("Local GUI")
        self.root.geometry("300x200")
        self.label = tkinter.Label(font=("Ubunt Mono", 160))
        self.label.pack()

    tmr = 0 # カウンタ変数

    def update(self):
        global tmr
        self.tmr = self.tmr + 1
        self.label["text"] = self.tmr
        self.root.after(10, self.update) # 1秒後にcounter関数を呼び出す

class LocalGuiNode(Node):
    def __init__(self):
        super().__init__('local_gui_node')
        self.subscription = self.create_subscription(
            String,
            'mode',
            self.mode_callback,
            10)
        self.local_gui = LocalGui()

    def mode_callback(self, msg):
        self.local_gui.display_mode(msg.data)

def main(args=None):
    local_gui = LocalGui()
    local_gui.update()
    local_gui.root.mainloop()


if __name__ == "__main__":
    main()