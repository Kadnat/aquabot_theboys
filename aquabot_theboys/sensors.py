#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64
from ros_gz_interfaces.msg import ParamVec
from sensor_msgs.msg import PointCloud2, LaserScan
from transforms3d.euler import quat2euler
from transforms3d.quaternions import qconjugate
from math import pi, sin, cos, radians, degrees
from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import PoseStamped



class MySensors(Node):
    def __init__(self):
        super().__init__('sensors')
        # Récupération des données de l'IMU
        self.create_subscription(Imu, "/wamv/sensors/imu/imu/data", self.imu_callback, 10)
        # Récupération de la position GPS
        self.create_subscription(NavSatFix, "/wamv/sensors/gps/gps/fix", self.gps_callback, 10)
        # Récupération des données en rapport avec la bouée
        self.create_subscription(ParamVec, "/wamv/sensors/acoustics/receiver/range_bearing", self.pinger_callback, 10)
        # Récupération des données du lidar
        self.create_subscription(LaserScan, "/wamv/sensors/lidars/lidar_wamv_sensor/scan", self.lidar_scan_callback, 10)
        # Récupération de la position de l'ennemi en x,y
        self.create_subscription(Float64MultiArray, "/position/ennemy", self.ennemy_pos_callback, 10)
        # Récupération de l'angle du bateau ennemi approximatif fourni par la caméra
        self.create_subscription(Float64, 'object_position', self.ennemy_angle_callback, 10)

        # Current x,y position of our boat
        self.pub_current_pos = self.create_publisher(Float64MultiArray, '/position/current', 10)
        # Buoy x,y position
        self.pub_pos_buoy = self.create_publisher(Float64MultiArray, '/position/buoy', 10)
        # Our current orientation
        self.pub_current_orientation = self.create_publisher(Float64, '/position/orientation', 10)
        # Ennemy x,y position
        self.pub_pos_ennemy = self.create_publisher(Float64MultiArray, '/position/ennemy', 10)
        # Ennemy GPS position
        self.pub_gps_ennemy = self.create_publisher(PoseStamped, '/vrx/patrolandfollow/alert_position', 10)
        # Object to avoid position
        self.pub_pos_to_avoid = self.create_publisher(Float64MultiArray, '/position/to_avoid', 10)
        self.pub_turn_around = self.create_publisher(Bool, '/position/look_around', 10)
        
        # Our current orientation
        self.current_orientation = 0.0
        # Our x,y position
        self.x_actual = 0.0
        self.y_actual = 0.0
        # Array that contains all distances and angle of one lidar measure 
        self.lidar_distance = []
        self.lidar_angle = []
        # Angle that separate us from the red boat
        self.angle_camera = 0
        self.angle_ennemy = 0.0
        self.distance_ennemy = 0.0



    # Transformation de la position x,y de l'ennemi en position gps 
    def ennemy_pos_callback(self, msg):
        gps_pos = PoseStamped()
        x, y = msg.data
        # GPS origin coordonnee
        lat_ref = 48.04631295
        lon_ref = -4.9763167
        # Earth radius
        R = 6378137

        lat_ref_rad = radians(lat_ref)
        lon_ref_rad = radians(lon_ref)

        lon_rad = (x/(R*cos(lat_ref_rad))) + lon_ref_rad
        lat_rad = (y/R) + lat_ref_rad

        lat = degrees(lat_rad)
        lon = degrees(lon_rad)

        gps_pos.pose.position.x = lat
        gps_pos.pose.position.y = lon
        self.pub_gps_ennemy.publish(gps_pos)

    # Récupération des angles et de leurs distances associés fournies par le lidar
    def lidar_scan_callback(self, msg):
        angle = []
        distance = []
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        for i, range in enumerate(msg.ranges):
            angle.append(angle_min + i * angle_increment)
            distance.append(range)
        self.lidar_angle = angle 
        self.lidar_distance = distance

    # Récupération de l'orientation du bateau
    def imu_callback(self, msg):
        msg_to_send = Float64()
        quaternion = (
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        )
        _, _, self.current_orientation = quat2euler(qconjugate(quaternion))
        msg_to_send.data = self.current_orientation
        self.pub_current_orientation.publish(msg_to_send)

    # Traitement des données de la bouée afin de récupérer une position x,y
    def pinger_callback(self, msg):
        msg_pos_to_reach = Float64MultiArray()
        for param in msg.params:
            if param.name == 'bearing':
                angle_bearing = param.value.double_value
            elif param.name == 'range':
                dist_bearing = param.value.double_value
        difference_angle = angle_bearing - self.current_orientation
        # Normalisation de l'angle
        difference_angle = (difference_angle + pi) % (2 * pi) - pi
        dY = dist_bearing * sin(difference_angle) 
        dX = dist_bearing * cos(difference_angle)  
        x_to_reach = self.x_actual + dX
        y_to_reach = self.y_actual + dY
        self.get_logger().info('Buoy  x : %f,  y : %f ' % (x_to_reach,y_to_reach))
        msg_pos_to_reach.data = [x_to_reach, y_to_reach]
        self.pub_pos_buoy.publish(msg_pos_to_reach)

    # T<ransformation de notre position gps en position x,y
    def gps_callback(self, msg):
        orientation_msg = Float64MultiArray()
        lat_ref = 48.04631295
        lon_ref = -4.9763167
        R = 6378137

        lat_rad = radians(msg.latitude)
        lon_rad = radians(msg.longitude)
        lat_ref_rad = radians(lat_ref)
        lon_ref_rad = radians(lon_ref)

        self.x_actual = R * (lon_rad - lon_ref_rad) * cos(lat_ref_rad)
        self.y_actual = R * (lat_rad - lat_ref_rad)

        orientation_msg.data = [self.x_actual, self.y_actual]
        self.pub_current_pos.publish(orientation_msg)

    def ennemy_angle_callback(self, msg):
        self.angle_camera = msg.data

    # Récupération de la position de l'ennemi
    def ennemy_finded_callback(self, msg):
        msg_pos_to_reach = Float64MultiArray()
        if (msg.data == True) and (abs(self.angle_camera)<0.1) :
            self.angle_ennemy = 0.0
            self.distance_ennemy = 0.0
            for i, (angle, distance) in enumerate(zip(self.lidar_angle, self.lidar_distance)):
                if (self.angle_camera-0.4 <= angle <= self.angle_camera+0.4) and distance>0 and distance < 1000:
                    self.angle_ennemy = angle
                    self.distance_ennemy = distance
            difference_angle = self.angle_ennemy  - self.current_orientation
            # Normalisaion de l'angle
            difference_angle = ((difference_angle+pi) % (2 * pi) -pi)
            dY = self.distance_ennemy * sin(difference_angle)  # changement en y
            dX = self.distance_ennemy * cos(difference_angle)  # changement en x
            x_to_reach = self.x_actual + dX
            y_to_reach = self.y_actual + dY
            if self.distance_ennemy <= 1000 and self.distance_ennemy>0.0:
                msg_pos_to_reach.data = [x_to_reach, y_to_reach]
                self.pub_pos_ennemy.publish(msg_pos_to_reach)
                self.get_logger().info('Ennemy x : %f, Delta y : %f ' % (x_to_reach,y_to_reach))



def main(args=None):
    rclpy.init(args=args)
    node = MySensors()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
