#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from math import atan2, pi
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from time import sleep


class MyCmdMotors(Node):
    def __init__(self):
        super().__init__('cmd_motors') 
        # position du moteur
        self.pub_pos = self.create_publisher(Float64, '/wamv/thrusters/main/pos', 10)
        # vitesse du propulseur
        self.pub_thrust = self.create_publisher(Float64, '/wamv/thrusters/main/thrust', 10)
        timer_period = 0.1  # Période du timer en secondes
        # Appel la fonction pour la cmd moteurs toutes les 100ms
        self.create_timer(timer_period, self.cmd_motors_callback)
        self.create_subscription(Float64MultiArray, '/position/to_reach', self.position_to_reach_callback, 10)
        self.create_subscription(Float64, "/position/orientation", self.orientation_callback, 10)
        self.create_subscription(Float64, "/position/actual", self.position_actual, 10)
        self.x_to_reach = 0.0
        self.y_to_reach = 0.0
        self.aligning = False  # Ajoutez un indicateur pour savoir si le bateau est en train de s'aligner
        self.x_actual = 0.0
        self.y_actual = 0.0
        self.current_orientation = 0.0

    def position_to_reach_callback(self, msg):
        self.x_to_reach, self.y_to_reach = msg.data

    def position_actual(self,msg):
        self.x_actual, self.y_actual = msg.data

    def orientation_callback(self,msg):
        self.current_orientation = msg.data

    def cmd_motors_callback(self):
        msg_pos = Float64()
        msg_thrust = Float64()
        delta_x = self.x_to_reach-self.x_actual
        delta_y = self.y_to_reach-self.y_actual
        angle_diff = (atan2(delta_y, delta_x)+pi/2) % (pi) + self.current_orientation
        self.get_logger().info('Angle again : %s' % str(angle_diff))
        if abs(angle_diff)>0.3: 
            if angle_diff>0.0:
                msg_pos.data = -pi/2
            else:
                msg_pos.data = pi/2
            self.pub_pos.publish(msg_pos)
            msg_thrust.data = 500.0
            self.pub_thrust.publish(msg_thrust)
            self.aligning = True
        else:
            if self.aligning  == True:
                self.aligning  = False
                msg_pos.data = 0.0
                self.pub_pos.publish(msg_pos)
                msg_thrust.data = 0.0
                self.pub_thrust.publish(msg_thrust)
                ## on dort jusq'à que le moteur revienne en position initiale
                sleep(3)
                self.get_logger().info('good')
            else :  
                msg_thrust.data = 5000.0
                self.pub_thrust.publish(msg_thrust)

    



def main(args=None):
    rclpy.init(args=args)

    node = MyCmdMotors()
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
