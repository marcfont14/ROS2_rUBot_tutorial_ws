import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from pynput import keyboard

class MoveTurtle(Node):
    def __init__(self):
        super().__init__('move_turtle_keyboard')
        # Publisher per enviar ordres de moviment
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # Subscriber per llegir la posició actual
        self.subscriber_ = self.create_subscription(Pose, '/turtle1/pose', self.listener_callback, 10)
        self.subscriber_

        # Estat de la tortuga
        self.pose = Pose()
        self.get_logger().info('MoveTurtle amb control de teclat iniciat!')

        # Start keyboard listener in background (non-blocking)
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def listener_callback(self, msg):
        self.pose = msg  # guardem la posició actual

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
