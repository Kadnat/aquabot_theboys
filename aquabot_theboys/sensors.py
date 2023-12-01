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
        # Get the IMU sensor informations
        self.create_subscription(Imu, "/wamv/sensors/imu/imu/data", self.imu_callback, 10)
        # Get our GPS position
        self.create_subscription(NavSatFix, "/wamv/sensors/gps/gps/fix", self.gps_callback, 10)
        # Get the buoy informations
        self.create_subscription(ParamVec, "/wamv/sensors/acoustics/receiver/range_bearing", self.pinger_callback, 10)
        # Get the Lidar sensor informations
        #self.create_subscription(PointCloud2, "/wamv/sensors/lidars/lidar_wamv_sensor/points", self.lidar_points_callback, 10)
        self.create_subscription(LaserScan, "/wamv/sensors/lidars/lidar_wamv_sensor/scan", self.lidar_scan_callback, 10)
        # Get the ennemy position
        self.create_subscription(Float64MultiArray, "/position/ennemy", self.ennemy_pos_callback, 10)
        # To know if the boat ennemy is detected
        self.create_subscription(Bool, 'object_detected', self.ennemy_finded_callback, 10)
        # Boat ennemy angle from our boat
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


    # def lidar_points_callback(self, msg):
    #     # msg.height, msg.width # taille de nuage de points
    #     # msg.point_step, msg.row_step
    #     # msg.data #tab de uint8 avec dedans des points de taille poitn_step*row_step
    #     # msg.dense #lisible ou pas
    # import sensor_msgs.point_cloud2 as pc2
    # for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
    #     x, y, z = point[:3]
    #     distance = math.sqrt(x**2 + y**2 + z**2)
    #     angle = math.atan2(y, x)

    # Transforming ennemy x,y position in gps position
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
        ###POUR LE Z SI VRAIMENT BESOIN, UTILISER LE LIDAR POINTS MAIS LONG A FAIRE JE PENSE
        self.pub_gps_ennemy.publish(gps_pos)

    # Getting a distance and angle array of each lidar measures
    def lidar_scan_callback(self, msg):
        # msg.angle_min, msg.angle_max # start and stop ange of the scan
        # msg.angle_increment, msg.time_increment #angle and time between two measures
        # msg.scan_time
        # msg.range_min, msg.range_max#min et max range value
        # msg.ranges #tab of ranges between min and max
        # msg.intensities
        angle = []
        distance = []
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        #self.get_logger().info('########################Orientation####" : %f' % (self.current_orientation))
        for i, range in enumerate(msg.ranges):
            angle.append(angle_min + i * angle_increment)
            distance.append(range)
            #self.get_logger().info('Angle : %f, Distance : %f' % (angle[i], distance[i]))
        self.lidar_angle = angle 
        self.lidar_distance = distance
        #self.detect_object()

    # Getting our boat orientation from the IMU sensor
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

    # Getting the x,y position of the buoy
    def pinger_callback(self, msg):
        msg_pos_to_reach = Float64MultiArray()
        for param in msg.params:
            if param.name == 'bearing':
                angle_bearing = param.value.double_value
            elif param.name == 'range':
                dist_bearing = param.value.double_value
        # Calculating the angle in the buoy reference
        difference_angle = angle_bearing - self.current_orientation
        # Normalize the angle difference
        difference_angle = (difference_angle + pi) % (2 * pi) - pi
        #self.get_logger().info('Angle1 : %s' % str(difference_angle))
        dY = dist_bearing * sin(difference_angle) 
        dX = dist_bearing * cos(difference_angle)  
        x_to_reach = self.x_actual + dX
        y_to_reach = self.y_actual + dY
        self.get_logger().info('Buoy  x : %f,  y : %f ' % (x_to_reach,y_to_reach))
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

    # Transforming our GPS position in a x,y position
    def gps_callback(self, msg):
        orientation_msg = Float64MultiArray()
        # GPS origin coordonnee
        lat_ref = 48.04631295
        lon_ref = -4.9763167
        # Earth radius
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

    #######A TESTER#############
    # Getting the ennemy x,y position
    def ennemy_finded_callback(self, msg):
        msg_pos_to_reach = Float64MultiArray()
        if (msg.data == True) and (abs(self.angle_camera)<0.1) :
            self.angle_ennemy = 0.0
            self.distance_ennemy = 0.0
            for i, (angle, distance) in enumerate(zip(self.lidar_angle, self.lidar_distance)):
                # Vérifiez si l'angle est entre -0.5 et 0.5 et si la distance est inférieure à 150
                if (self.angle_camera-0.4 <= angle <= self.angle_camera+0.4) and distance>0 and distance < 1000:
                    #self.get_logger().info('angle : %f' % angle)
                    #self.get_logger().info('distance : %f' % distance)
                    self.angle_ennemy = angle
                    self.distance_ennemy = distance
            difference_angle = self.angle_ennemy  - self.current_orientation
            # Normalize the angle difference
            difference_angle = ((difference_angle+pi) % (2 * pi) -pi)
            self.get_logger().info('angle_camera : %s' % str(self.angle_camera))
            self.get_logger().info('angle_equiv : %s' % str(self.angle_ennemy ))
            self.get_logger().info('Angle : %s' % str(difference_angle))
            self.get_logger().info('Distance : %s' % str(self.distance_ennemy))
            #self.get_logger().info('Angle1 : %s' % str(difference_angle))
            dY = self.distance_ennemy * sin(difference_angle)  # changement en y
            dX = self.distance_ennemy * cos(difference_angle)  # changement en x
            x_to_reach = self.x_actual + dX
            y_to_reach = self.y_actual + dY
            #self.get_logger().info('Delta x : %f, Delta y : %f  ONE' % (dX,dY))
            #self.get_logger().info('Angle2 : %s' % str(atan2(dY,dX)))
            if self.distance_ennemy <= 1000 and self.distance_ennemy>0.0:
                msg_pos_to_reach.data = [x_to_reach, y_to_reach]
                self.pub_pos_ennemy.publish(msg_pos_to_reach)
                self.get_logger().info('Ennemy x : %f, Delta y : %f ' % (x_to_reach,y_to_reach))

            else:
                msg_pos_to_reach.data = [self.x_actual, self.y_actual]
                self.pub_pos_ennemy.publish(msg_pos_to_reach)
                
                

    ####CODE NUL####
    # Detect object to avoid
    def detect_object(self):
        msg_pos_to_reach = Float64MultiArray()
        index_angle = [i for i, val in enumerate(self.lidar_angle) if -20 <= val <= 20]
        distance = self.lidar_distance[index_angle[0]]
        difference_angle = self.lidar_angle[index_angle[0]] - self.current_orientation
        # Normaliser la différence d'angle
        difference_angle = (difference_angle) % (2 * pi)
        dY = distance * sin(difference_angle)  # changement en y
        dX = distance * cos(difference_angle)  # changement en x
        x_to_reach = self.x_actual - dX
        y_to_reach = self.y_actual - dY
        msg_pos_to_reach.data = [x_to_reach, y_to_reach]
        self.pub_pos_to_avoid.publish(msg_pos_to_reach)



def main(args=None):
    rclpy.init(args=args)
    node = MySensors()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
