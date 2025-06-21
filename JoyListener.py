import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoyListener(Node):
    def __init__(self):
        super().__init__('joy_listener')
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

    def joy_callback(self, msg):
        # pick only axes 0 & 1
        #x = msg.axes[0]
        #y = msg.axes[1]
        L2 = msg.axes[2]            # 1-(-1)
        R2 = msg.axes[5]            # 1-(-1)
        #R3_UP = msg.axes[4]         # 0-1
        #R3_DOWN = msg.axes[4]       # 0-(-1)
        #R3_LEFT = msg.axes[3]       # 0-1
        #R3_RIGHT = msg.axes[3]      # 0-(-1)

        if msg.axes[1] > 0 :
            L3_UP = msg.axes[1]         # 0-1
            L3_DOWN = 0
        else :
            L3_UP = 0
            L3_DOWN = abs(msg.axes[1])       # 0-(-1)

        if msg.axes[0] > 0 :
            L3_LEFT = msg.axes[0]         # 0-1
            L3_RIGHT = 0
        else :
            L3_RIGHT = abs(msg.axes[0])       # 0-(-1)
            L3_LEFT =0
            
        if msg.axes[4] > 0 :
            R3_UP = msg.axes[4]         # 0-1
            R3_DOWN = 0
        else :
            R3_DOWN = abs(msg.axes[4])       # 0-(-1)
            R3_UP = 0

        if msg.axes[3] > 0 :
            R3_LEFT = msg.axes[3]         # 0-1
            R3_RIGHT = 0
        else :
            R3_RIGHT = abs(msg.axes[3])      # 0-(-1)
            R3_LEFT = 0

        # pick only buttons 0 & 1
        #b_button = msg.buttons[1]
        MENU = msg.buttons[9]

        self.get_logger().info(f"Left Trigger: {L2:.2f}, Right Trigger {R2:.2f}")
        self.get_logger().info(f"Left stick — UP: {L3_UP:.2f}, DOWN: {L3_DOWN:.2f}, LEFT: {L3_LEFT:.2f}, RIGHT: {L3_RIGHT:.2f}")
        self.get_logger().info(f"RIGHT stick — UP: {R3_UP:.2f}, DOWN: {R3_DOWN:.2f}, LEFT: {R3_LEFT:.2f}, RIGHT: {R3_RIGHT:.2f}")
        self.get_logger().info(f"MENU: {'Pressed' if MENU else 'Released'}")
        #self.get_logger().info(f"B button: {'Pressed' if b_button else 'Released'}")
        self.declare_parameter('L2', L2)
        self.declare_parameter('L3_LEFT', L3_LEFT)
        self.declare_parameter('L3_RIGHT', L3_RIGHT)
        self.declare_parameter('L3_UP', L3_UP)
        self.declare_parameter('L3_DOWN', L3_DOWN)
        self.declare_parameter('R2', R2)
        self.declare_parameter('R3_LEFT', R3_LEFT)
        self.declare_parameter('R3_RIGHT', R3_RIGHT)
        self.declare_parameter('R3_UP', R3_UP)
        self.declare_parameter('R3_DOWN', R3_DOWN)
        self.declare_parameter('MENU', MENU)


def main(args=None):
    rclpy.init(args=args)
    node = JoyListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
