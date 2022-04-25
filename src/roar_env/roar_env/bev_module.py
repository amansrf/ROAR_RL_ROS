import rclpy
from rclpy.node import Node


class BEVmodule(Node):
    def __init__(self):
        super().__init__("bev_module")
        self.bev_sub_node = self.create_subscription(msg, '/topic', self.bev_callback)

    def bev_callback():
        print('abc')