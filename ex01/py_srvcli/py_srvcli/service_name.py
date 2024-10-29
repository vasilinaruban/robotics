from tutorial_interfaces.srv import FullNameSumService

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(FullNameSumService, 'full_name_sum_service', self.fullname_callback)

    def fullname_callback(self, request, response):
        response.full_name = request.first_name + " " + request.name + " " + request.last_name
        self.get_logger().info('Incoming request\nfirst_name: %s name: %s last_name: %s' % (request.first_name, request.name, request.last_name))

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()