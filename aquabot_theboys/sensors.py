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
        # # Sensor Information API
        # self.create_subscription(Image, '/wamv/sensors/cameras/main_camera_sensor/image_raw', self.image_callback, 10)
        # self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_callback, 10)
        
        # # Alert API
        # self.create_subscription(Pose, '/vrx/patrolandfollow/alert_position', self.alert_position_callback, 10)
        
        # # Task Information API
        # self.create_subscription(Float32, '/clock', self.clock_callback, 10)

    def imu_callback(msg):
        global current_orientation
        quaternion = (
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        )
        _, _, current_orientation = quat2euler(qconjugate(quaternion))

    def gps_callback(self, msg):
        self.current_position = (msg.latitude, msg.longitude)

    def pinger_callback(self, msg):
            self.current_bearing_to_buoy = msg.bearing
            self.current_range_to_buoy = msg.range
            for param in msg.parameters:
                if param.name == 'range':
                    self.current_range_to_buoy = param.value.double_value
                elif param.name == 'bearing':
                    self.current_bearing_to_buoy = param.value.double_value
            # Calculer la position relative ici...
            if self.current_position is not None and self.current_orientation is not None:
                # Ajuster le bearing en fonction de l'orientation actuelle...
                relative_bearing = self.current_bearing_to_buoy - self.current_orientation
                # Calculer la position de la bouée...
                buoy_position = (
                    self.current_position[0] + self.current_range_to_buoy * math.cos(relative_bearing),
                    self.current_position[1] + self.current_range_to_buoy * math.sin(relative_bearing)
                )
                # Calculer la position relative de l'USV...
                relative_position = (
                    buoy_position[0] - self.current_position[0],
                    buoy_position[1] - self.current_position[1]
                )
                self.get_logger().info('Relative position to buoy: %s' % str(relative_position))

                # Publier la position relative sur les topics de contrôle des moteurs
                self.angle_cmd_motors = relative_bearing
                self.dist_cmd_motors= self.current_range_to_buoy


    def image_callback(self, msg):
        # Process image data here
        pass

    def alert_position_callback(self, msg):
        self.get_logger().info('Alert Position: "%s"' % msg.position.x)

    def clock_callback(self, msg):
        self.get_logger().info('Clock: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('sensors_aquabot')
    
    #pos_publisher = node.create_publisher(Float64, '/wamv/thrusters/main/pos', 10)
    #thrust_publisher = node.create_publisher(Float64, '/wamv/thrusters/main/thrust', 10)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

    

if __name__ == '__main__':
    main()
