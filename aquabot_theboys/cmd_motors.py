#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from sensors import MySensors
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
import os # seulement pour les tests

class MyCmdMotors(Node):
    def __init__(self):
        super().__init__('cmd_motors') 
        # position du moteur
        self.pub_pos = self.create_publisher(Float64, '/wamv/thrusters/main/pos', 10)
        # vitesse du propulseur
        self.pub_thrust = self.create_publisher(Float64, '/wamv/thrusters/main/thrust', 10)
        timer_period = 0.1  # Période du timer en secondes
        # Appel la fonction pour la cmd moteurs toutes les 100ms
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.create_subscription(Float64MultiArray, '/cmd_motors', self.cmd_motors_callback, 10)
        self.pos = 0.0
        self.thrust = 0.0
        self.angle_cmd_motors=0.0
        self.dist_cmd_motors=0.0

    def timer_callback(self):
        msg_pos = Float64()
        msg_thrust = Float64()
        sensors_info = MySensors()
        cmd_angle = self.angle_cmd_motors
        cmd_dist = self.dist_cmd_motors
        self.get_logger().info('Val recup: angle = %f, distance = %f' % (cmd_angle, cmd_dist))
        # Si l'angle est trop grand on ne déplace pas le drone mais on le fait tourner sur lui même
        if abs(cmd_angle) >= 0.1:
            if cmd_angle >= math.pi/4 :
                self.pos = math.pi/4  
                self.thrust = 100.0
            elif cmd_angle <= math.pi/4 :
                self.pos = -math.pi/4 
                self.thrust = 100.0
            else :
                self.pos = cmd_angle
                self.thrust = 50.0
        else :
            self.pos = 0.0
            if cmd_dist < 10.0 :
                self.thrust = 0.0
            else  :
                self.thrust = 100.0
            
        #  On affecte les variables aux messages
        msg_pos.data= self.pos
        msg_thrust.data = self.thrust
        # On publie les messages les messages
        self.pub_pos.publish(msg_pos)
        self.pub_thrust.publish(msg_thrust)
        # Affichage de la commande envoyé
        self.get_logger().info('Envoi: pos = %f, thrust = %f' % (msg_pos.data, msg_thrust.data))

    def cmd_motors_callback(self, msg):
        self.angle_cmd_motors, self.dist_cmd_motors = msg.data
        self.get_logger().info('Received: angle = %f, distance = %f' % (self.angle_cmd_motors, self.dist_cmd_motors))

def main(args=None):
    rclpy.init(args=args)

    node = MyCmdMotors()
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()