import rclpy
# import the node module
from rclpy.node import Node
# import the msg type for sending velocity commands
from geometry_msgs.msg import Twist
# import the msg type published by turtlesim node which we will receive
from turtlesim.msg import Pose

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

def main(args=None):
    rclpy.init(args=args)
    node = MoveTurtle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
