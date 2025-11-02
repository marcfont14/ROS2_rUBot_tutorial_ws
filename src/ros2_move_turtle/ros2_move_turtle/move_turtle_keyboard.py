import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from pynput import keyboard

class MoveTurtle(Node):
    def __init__(self):
        super().__init__('move_turtle')
        # Publisher: send velocity commands
        # the turtle will move when we publish a Twist message
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # Subscriber: read turtle pose
        self.subscriber_ = self.create_subscription(Pose, '/turtle1/pose', self.listener_callback, 10)
        self.subscriber_  # prevent unused variable warning
        self.get_logger().info('MoveTurtle node started!')

    def listener_callback(self, msg):
        """Called whenever a new pose message is received."""
        cmd = Twist()

        # Stop the turtle if it's out of bounds
        if msg.x >= 7.0 or msg.y >= 7.0:
            # the Twist linear and angular velocities will go to 0
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info(f"Stopping turtle at x={msg.x:.2f}, y={msg.y:.2f}")
        else:
            # Move forward slowly
            cmd.linear.x = 1.0
            cmd.angular.z = 0.0

        self.publisher_.publish(cmd)

    def on_press(self, key):
        cmd = Twist()

        try:
            # Moviment amb fletxes
            if key == keyboard.Key.up:
                cmd.linear.x = 1.5
                cmd.angular.z = 0.0
                self.get_logger().info("⬆️ Endavant")
            elif key == keyboard.Key.down:
                cmd.linear.x = -1.5
                cmd.angular.z = 0.0
                self.get_logger().info("⬇️ Enrere")
            elif key == keyboard.Key.left:
                cmd.linear.x = 0.0
                cmd.angular.z = 1.5
                self.get_logger().info("⬅️ Gira esquerra")
            elif key == keyboard.Key.right:
                cmd.linear.x = 0.0
                cmd.angular.z = -1.5
                self.get_logger().info("➡️ Gira dreta")
            elif key == keyboard.Key.esc:
                self.get_logger().info("🚫 Sortint del programa...")
                rclpy.shutdown()
                return
            else:
                return  # ignora altres tecles
        except AttributeError:
            return

        # Publica el moviment
        self.publisher_.publish(cmd)

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
