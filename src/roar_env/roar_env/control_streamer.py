import rclpy
from rclpy.node import Node
from carla_msgs.msg import CarlaEgoVehicleControl


class ControlStreamer(Node):
    def __init__(self):
        super().__init__("control_streamer")
        self.cntrl_pub_node = self.create_publisher(CarlaEgoVehicleControl,"/carla/ego_vehicle/vehicle_control_cmd_manual",10)
    
    def pub_cntrl(self,throttle = 0, steer = 0, brake = 0):
        control_msg = CarlaEgoVehicleControl

        control_msg.brake = brake
        control_msg.steer = steer
        control_msg.throttle = throttle

        control_msg.header.stamp = self.get_clock().now().to_msg()
        self.cntrl_pub_node.publish(control_msg)