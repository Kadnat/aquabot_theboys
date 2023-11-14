#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, NavSatFix
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float64
from ros_gz_interfaces.msg import ParamVec
from transforms3d.euler import quat2euler
from transforms3d.quaternions import qconjugate
import math
from std_msgs.msg import Float64MultiArray

class MySensors(Node):
    angle_cmd_motors = 0.0
    dist_cmd_motors = 0.0
    def __init__(self):
        super().__init__('sensors')

        #IMU, GPS and Pinger management
        self.current_orientation = None
        self.current_position = None
        self.current_bearing_to_buoy = None
        self.current_range_to_buoy = None

        self.create_subscription(Imu, "/wamv/sensors/imu/imu/data", self.imu_callback, 10)
        self.create_subscription(NavSatFix, "/wamv/sensors/gps/gps/fix", self.gps_callback, 10)
        self.create_subscription(ParamVec, "/wamv/sensors/acoustics/receiver/range_bearing", self.pinger_callback, 10)
        self.pub_cmd_motors = self.create_publisher(Float64MultiArray, '/cmd_motors', 10)

    def imu_callback(self, msg):
        quaternion = (
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        )
        _, _, self.current_orientation = quat2euler(qconjugate(quaternion))
        #self.get_logger().info('IMU callback executed, current orientation: %s' % self.current_orientation)

    def gps_callback(self, msg):
        self.current_position = (msg.latitude, msg.longitude)
        #self.get_logger().info('GPS callback executed, current position: %s' % str(self.current_position))

    def pinger_callback(self, msg):
        print(msg)
        for param in msg.params:
            if param.name == 'bearing':
                self.current_bearing_to_buoy = param.value.double_value
            elif param.name == 'range':
                self.current_range_to_buoy = param.value.double_value

        if self.current_position is not None and self.current_orientation is not None:
            relative_bearing = self.current_bearing_to_buoy - self.current_orientation
            buoy_position = (
                self.current_position[0] + self.current_range_to_buoy * math.cos(relative_bearing),
                self.current_position[1] + self.current_range_to_buoy * math.sin(relative_bearing)
            )
            relative_position = (
                buoy_position[0] - self.current_position[0],
                buoy_position[1] - self.current_position[1]
            )
            #self.get_logger().info('Relative position to buoy: %s' % str(relative_position))
            self.angle_cmd_motors = self.current_bearing_to_buoy
            self.dist_cmd_motors= self.current_range_to_buoy
            msg_cmd_motors = Float64MultiArray()
            msg_cmd_motors.data = [self.angle_cmd_motors, self.dist_cmd_motors]
            self.pub_cmd_motors.publish(msg_cmd_motors)

def main(args=None):
    rclpy.init(args=args)
    node = MySensors()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
