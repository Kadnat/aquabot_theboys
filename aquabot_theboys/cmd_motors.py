#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from math import atan2, pi
from std_msgs.msg import Float64, Float64MultiArray, Bool
from time import sleep


class MyCmdMotors(Node):
    def __init__(self):
        super().__init__('cmd_motors') 
        # Angle des moteurs
        self.pub_pos = self.create_publisher(Float64, '/wamv/thrusters/main/pos', 10)
        # Vitesse des moteurs
        self.pub_thrust = self.create_publisher(Float64, '/wamv/thrusters/main/thrust', 10)
        
        # Récupère la position à atteindre
        self.create_subscription(Float64MultiArray, '/position/to_reach', self.cmd_motors_callback, 10)
        # Récupère l'orientation actuelle depuis l'IMU
        self.create_subscription(Float64, "/position/orientation", self.orientation_callback, 10)
        # Récupère notre position actuelle en x et y
        self.create_subscription(Float64MultiArray, "/position/current", self.position_actual, 10)

        self.create_subscription(Bool, '/position/look_around', self.look_around, 10)

        self.create_subscription(Float64MultiArray, '/position/follow_without_moving',self.follow_without_moving, 10)

        # Pour savoir si nous sommes en train de changer notre orientation
        self.aligning = False  
        self.x_to_reach = 0.0
        self.y_to_reach = 0.0
        self.x_actual = 0.0
        self.y_actual = 0.0
        self.current_orientation = 0.0

    def position_actual(self,msg):
        """
        Pour récupérer notre position actuelle en x et y.
        """
        self.x_actual, self.y_actual = msg.data

    def orientation_callback(self,msg):
        """
        Fonction utilisée pour récupérer la valeur de l'orientation.
        """

        self.current_orientation = msg.data

    def look_around(self,msg):
        """
        Fonction utilisée pour regarder si le bateau ennemi est autour de nous.
        """

        msg_pos = Float64()
        msg_thrust = Float64()
        if msg.data == True:
            msg_pos.data = -pi/2
        else:
            msg_pos.data = pi/2
        msg_thrust.data = 200.0

        self.pub_pos.publish(msg_pos)
        self.pub_thrust.publish(msg_thrust)

    def follow_without_moving(self, msg):
        """
        Fonction permettant de suivre du regard le bateau ennemi lorsqu'il apparait dans le champs de la caméra.
        """
        msg_pos = Float64()
        msg_thrust = Float64()
        self.x_to_reach, self.y_to_reach = msg.data
        delta_x = (self.x_actual - self.x_to_reach) 
        delta_y = (self.y_actual - self.y_to_reach)
        angle_diff = ((atan2(delta_y, delta_x) + self.current_orientation) - pi) % (2*pi) 
        # A VOIR SI ON LAISSE
        if angle_diff > pi:
            angle_diff -= 2 * pi
        elif angle_diff < -pi:
            angle_diff += 2 * pi
        msg_pos.data = -pi/2
        msg_thrust.data = 600.0
        if abs(angle_diff)>0.1: 
            if angle_diff>0.0:
                msg_pos.data = -pi/2
            else:
                msg_pos.data = pi/2
            self.pub_pos.publish(msg_pos)
            msg_thrust.data = 500.0
            self.pub_thrust.publish(msg_thrust)

    def cmd_motors_callback(self, msg):
        """
        Fonction utilisée pour contrôler les moteurs en envoyant une position x et y.
        """
        msg_pos = Float64()
        msg_thrust = Float64()
        self.x_to_reach, self.y_to_reach = msg.data
        delta_x = (self.x_actual - self.x_to_reach) 
        delta_y = (self.y_actual - self.y_to_reach)

        angle_diff = ((atan2(delta_y, delta_x) + self.current_orientation) - pi) % (2*pi) 
        if angle_diff > pi:
            angle_diff -= 2 * pi
        elif angle_diff < -pi:
            angle_diff += 2 * pi

        # Si la position que nous voulons atteindre est trop proche de notre position réelle, nous arrêtons les moteurs.
        if ((self.x_to_reach <= self.x_actual +5) and (self.x_to_reach >= self.x_actual -5)) and ((self.y_to_reach <= self.y_actual +5) and (self.y_to_reach >= self.y_actual -5)):
            msg_pos.data = 0.0
            self.pub_pos.publish(msg_pos)
            msg_thrust.data = 0.0
            self.pub_thrust.publish(msg_thrust)

        else :
            # Si l'angle entre nous et l'objet que nous voulons atteindre est supérieur à 0,1 radians, nous changeons d'orientation.
            if abs(angle_diff)>0.1: 
                if angle_diff>0.0:
                    msg_pos.data = -pi/2
                else:
                    msg_pos.data = pi/2
                self.pub_pos.publish(msg_pos)
                msg_thrust.data = 500.0
                self.pub_thrust.publish(msg_thrust)
                self.aligning = True
            else:
                # Si nous avons déplacé notre bateau avant d'arriver ici, nous l'arrêtons et attendons que la position des moteurs soit égale à 0.
                if self.aligning  == True:
                    self.aligning  = False
                    msg_pos.data = 0.0
                    self.pub_pos.publish(msg_pos)
                    msg_thrust.data = 0.0
                    self.pub_thrust.publish(msg_thrust)
                    sleep(3)
                else :  
                    msg_thrust.data = 10000.0
                    self.pub_thrust.publish(msg_thrust)

    



def main(args=None):
    rclpy.init(args=args)
    node = MyCmdMotors()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
