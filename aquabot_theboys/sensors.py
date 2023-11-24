#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64
from ros_gz_interfaces.msg import ParamVec
from sensor_msgs.msg import PointCloud2, LaserScan
from transforms3d.euler import quat2euler
from transforms3d.quaternions import qconjugate
from math import pi, sin, cos, radians, atan2
from std_msgs.msg import Float64MultiArray
from aquabot_theboys_msgs.msg import TwoFloat64Array
import pyproj


class MySensors(Node):
    angle_cmd_motors = 0.0
    dist_cmd_motors = 0.0
    def __init__(self):
        super().__init__('sensors')

        #IMU, GPS and Pinger management
        self.current_orientation = 0.0
        self.x_actual = 0.0
        self.y_actual = 0.0

        self.create_subscription(Imu, "/wamv/sensors/imu/imu/data", self.imu_callback, 10)
        self.create_subscription(NavSatFix, "/wamv/sensors/gps/gps/fix", self.gps_callback, 10)
        self.create_subscription(ParamVec, "/wamv/sensors/acoustics/receiver/range_bearing", self.pinger_callback, 10)
        #self.create_subscription(PointCloud2, "/wamv/sensors/lidars/lidar_wamv_sensor/points", self.lidar_points_callback, 10)
        self.create_subscription(LaserScan, "/wamv/sensors/lidars/lidar_wamv_sensor/scan", self.lidar_scan_callback, 10)
        #self.create_subscription(Float64, "/data/filtered", self.data_filtered_pinger, 10)
        self.pub_current_pos = self.create_publisher(Float64MultiArray, '/position/current', 10)
        self.pub_pos_buoy = self.create_publisher(Float64MultiArray, '/position/buoy', 10)
        #self.pub_filter = self.create_publisher(Float64, '/data/to_filter', 10)
        self.pub_current_orientation = self.create_publisher(Float64, '/position/orientation', 10)
        self.pub_lidar_data = self.create_publisher(TwoFloat64Array, '/lidar/data', 10)

    # def lidar_points_callback(self, msg):
    #     # msg.height, msg.width # taille de nuage de points
    #     # msg.point_step, msg.row_step
    #     # msg.data #tab de uint8 avec dedans des points de taille poitn_step*row_step
    #     # msg.dense #lisible ou pas


    def lidar_scan_callback(self, msg):
        # msg.angle_min, msg.angle_max # start and stop ange of the scan
        # msg.angle_increment, msg.time_increment #angle and time between two measures
        # msg.scan_time
        # msg.range_min, msg.range_max#min et max range value
        # msg.ranges #tab of ranges between min and max
        # msg.intensities
        msg_lidar = TwoFloat64Array()
        angle = []
        distance = []
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        for i, range in enumerate(msg.ranges):
            angle.append(angle_min + i * angle_increment)
            distance.append(range)
            #self.get_logger().info('Angle : %f, Distance : %f' % (angle, distance))
        msg_lidar.array1.data = angle 
        msg_lidar.array2.data= distance
        self.pub_lidar_data.publish(msg_lidar)
        


    


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

    def pinger_callback(self, msg):
        msg_pos_to_reach = Float64MultiArray()
        for param in msg.params:
            if param.name == 'bearing':
                angle_bearing = param.value.double_value
            elif param.name == 'range':
                dist_bearing = param.value.double_value
        # Calcul de l'angle dans le référentiel de  la bouée
        difference_angle = angle_bearing - self.current_orientation
        # Normaliser la différence d'angle
        difference_angle = (difference_angle + pi) % (2 * pi) - pi
        #self.get_logger().info('Angle1 : %s' % str(difference_angle))
        dY = dist_bearing * sin(difference_angle)  # changement en y
        dX = dist_bearing * cos(difference_angle)  # changement en x
        x_to_reach = self.x_actual - dX
        y_to_reach = self.y_actual - dY
        #self.get_logger().info('Delta x : %f, Delta y : %f  ONE' % (dX,dY))
        #self.get_logger().info('Angle2 : %s' % str(atan2(dY,dX)))
        msg_pos_to_reach.data = [x_to_reach, y_to_reach]
        self.pub_pos_buoy.publish(msg_pos_to_reach)

    # def data_filtered_pinger(self, msg):
    #     #########Pour test filter, à mettre quelque part si besoin##############
    #     msg_filter = Float64()
    #     msg_filter.data = self.current_bearing_to_buoy
    #     self.pub_filter.publish(msg_filter)
    #     #######################################
    #     #test=1 # juste pour pas avoir d'erreur
    #     #self.get_logger().info('filter callback executed, data filtered: %s' % str(msg.data))

    def gps_callback(self, msg):
        orientation_msg = Float64MultiArray()
        # # On définit les parametres de projection
        # proj_latlon = pyproj.Proj(proj='latlong', datum='WGS84')
        # proj_xy = pyproj.Proj(proj="utm", zone=44, datum='WGS84')
        # # On convertit les données
        # self.x_actual, self.y_actual = pyproj.transform(proj_latlon, proj_xy, msg.latitude, msg.longitude)

        # coordonnées GPS de l'origine
        lat_ref = 48.04631295
        lon_ref = -4.9763167
        # Rayon de la Terre en mètres
        R = 6378137

        # Conversion des latitudes et longitudes en radians
        lat_rad = radians(msg.latitude)
        lon_rad = radians(msg.longitude)
        lat_ref_rad = radians(lat_ref)
        lon_ref_rad = radians(lon_ref)

        # Calcul des coordonnées x et y
        self.x_actual = R * (lon_rad - lon_ref_rad) * cos(lat_ref_rad)
        self.y_actual = R * (lat_rad - lat_ref_rad)

        orientation_msg.data = [self.x_actual, self.y_actual]
        self.pub_current_pos.publish(orientation_msg)





def main(args=None):
    rclpy.init(args=args)
    node = MySensors()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
