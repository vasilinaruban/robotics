import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CircleMovement(Node):
    def __init__(self):

        super().__init__('circle_movement')

        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.movement)


    def movement(self):
        twister = Twist()
        twister.linear.x = 0.5
        twister.angular.z = 0.5
        self.publisher.publish(twister)


def main(args=None):
    rclpy.init(args=args)
    node = CircleMovement()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


