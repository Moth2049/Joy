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
        # Triggers: full range [-1 to 1]
        L2 = msg.axes[2]
        R2 = msg.axes[5]
        if -1 <= L2 <= 1:
            self.get_logger().info(f"Left Trigger (L2): {L2:.2f}")
        if -1 <= R2 <= 1:
            self.get_logger().info(f"Right Trigger (R2): {R2:.2f}")

        # Left stick — Axis 0 (x), Axis 1 (y)
        x = msg.axes[0]
        y = msg.axes[1]

        if y > 0:  # L3_UP
            self.get_logger().info(f"Left Stick UP: {y:.2f}")
        elif y < 0:  # L3_DOWN
            self.get_logger().info(f"Left Stick DOWN: {y:.2f}")

        if x < 0:  # L3_LEFT
            self.get_logger().info(f"Left Stick LEFT: {x:.2f}")
        elif x > 0:  # L3_RIGHT
            self.get_logger().info(f"Left Stick RIGHT: {x:.2f}")

        # Right stick — Axis 3 (x), Axis 4 (y)
        rx = msg.axes[3]
        ry = msg.axes[4]

        if ry > 0:  # R3_UP
            self.get_logger().info(f"Right Stick UP: {ry:.2f}")
        elif ry < 0:  # R3_DOWN
            self.get_logger().info(f"Right Stick DOWN: {ry:.2f}")

        if rx < 0:  # R3_LEFT
            self.get_logger().info(f"Right Stick LEFT: {rx:.2f}")
        elif rx > 0:  # R3_RIGHT
            self.get_logger().info(f"Right Stick RIGHT: {rx:.2f}")

        # Buttons
        MENU = msg.buttons[10]
        if MENU:
            self.get_logger().info("MENU button pressed")

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
