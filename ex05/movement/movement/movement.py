import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class Movement(Node):
    def __init__(self):
        super().__init__('movement')

        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.movement)

        # Состояния движения
        self.state = 0
        self.state_duration = 2.0  # Длительность каждого состояния в секундах
        self.state_start_time = self.get_clock().now().to_msg().sec

    def movement(self):
        current_time = self.get_clock().now().to_msg().sec
        elapsed_time = current_time - self.state_start_time

        if elapsed_time >= self.state_duration:
            # Переключаем состояние
            self.state = (self.state + 1) % 4
            self.state_start_time = current_time

        twister = Twist()

        if self.state == 0:
            # Движение направо
            twister.linear.x = 0.0
            twister.angular.z = -0.5  # Поворот направо
        elif self.state == 1:
            # Движение вперед
            twister.linear.x = 0.5
            twister.angular.z = 0.0
        elif self.state == 2:
            # Движение налево
            twister.linear.x = 0.0
            twister.angular.z = 0.5  # Поворот налево
        elif self.state == 3:
            # Движение вперед
            twister.linear.x = 0.5
            twister.angular.z = 0.0

        self.publisher.publish(twister)

def main(args=None):
    rclpy.init(args=args)
    node = Movement()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
