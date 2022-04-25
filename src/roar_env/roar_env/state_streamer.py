import rclpy
from rclpy.node import Node


class StateStreamer(Node):
    def __init__(self):
        super().__init__("state_streamer")
        self.state_sub_node = self.create_subscription(msg, '/topic', self.state_callback)

    def state_callback():
        print('abc')