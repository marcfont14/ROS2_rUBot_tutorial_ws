import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from pynput import keyboard


class MoveTurtle(Node):
    def __init__(self):
        super().__init__('move_turtle')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.pose = Pose()
        self.get_logger().info("✅ MoveTurtle node started! Use arrow keys to move. ESC to exit.")

        # Start listening to keyboard
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def pose_callback(self, msg):
        """Update current pose."""
        self.pose = msg

    def publish_if_inside_bounds(self, cmd):
        """Publish only if turtle inside bounds."""
        # Define limits
        if 0.5 < self.pose.x < 7.0 and 0.5 < self.pose.y < 7.0:
            self.publisher_.publish(cmd)
        else:
            stop = Twist()
            self.publisher_.publish(stop)
            self.get_logger().warn(
                f"🚫 Boundary reached (x={self.pose.x:.2f}, y={self.pose.y:.2f}). Movement blocked."
            )

    def on_press(self, key):
        cmd = Twist()
        try:
            if key == keyboard.Key.up:
                cmd.linear.x = 0.5
                cmd.angular.z = 0.0
            elif key == keyboard.Key.down:
                cmd.linear.x = -0.5
                cmd.angular.z = 0.0
            elif key == keyboard.Key.left:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5
            elif key == keyboard.Key.right:
                cmd.linear.x = 0.0
                cmd.angular.z = -0.5
            elif key == keyboard.Key.esc:
                self.get_logger().info("👋 Exiting...")
                rclpy.shutdown()
                return
            else:
                return
        except AttributeError:
            return

        self.publish_if_inside_bounds(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MoveTurtle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
