#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
import os # seulement pour les tests
from time import sleep

class MyCmdMotors(Node):
    def __init__(self):
        super().__init__('cmd_motors') 
        # position du moteur
        self.pub_pos = self.create_publisher(Float64, '/wamv/thrusters/main/pos', 10)
        # vitesse du propulseur
        self.pub_thrust = self.create_publisher(Float64, '/wamv/thrusters/main/thrust', 10)
        timerthrust_period = 0.1  # Période du timer en secondes
        timerpos_period = 1  # Période du timer en secondes
        # Appel la fonction pour la cmd moteurs toutes les 100ms
        self.timerthrust = self.create_timer(timerthrust_period, self.timerthrust_callback)
        self.timerpos = self.create_timer(timerpos_period, self.timerpos_callback)
        self.create_subscription(Float64MultiArray, '/cmd_motors', self.cmd_motors_callback, 10)
        self.pos = 0.0
        self.thrust = 0.0
        self.angle_cmd_motors=0.0
        self.dist_cmd_motors=0.0
        self.turn_in_progress=False
        self.first_turn=True
        #self.first_in = 0
        self.dist_ancienne = 10000

    def timerpos_callback(self):
        self.turn_in_progress=True
        if(self.first_turn==True):
            self.thrust=0
            self.first_turn=False
        if ((self.angle_cmd_motors >= 0.2) & (self.angle_cmd_motors <= math.pi/2)):
            self.pos = math.pi/8
            self.thrust = 1000
        elif ((self.angle_cmd_motors >= math.pi/2) & (self.angle_cmd_motors <= math.pi)):
            self.pos = math.pi/4
            self.thrust = 1000
        elif ((self.angle_cmd_motors >= -math.pi) & (self.angle_cmd_motors <= -math.pi/2)):
            self.pos = -math.pi/4
            self.thrust = 1000
        elif ((self.angle_cmd_motors >= -math.pi/2) & (self.angle_cmd_motors <= -0.2)):
            self.pos = math.pi/8
            self.thrust = 1000
        elif ((self.angle_cmd_motors >= -0.2) & (self.angle_cmd_motors <= 0)):
            self.pos = 0
            self.thrust = 0
            self.turn_in_progress=False
            self.first_turn=True
        elif ((self.angle_cmd_motors >= 0) & (self.angle_cmd_motors <= 0.2)):
            self.pos = 0
            self.thrust = 0
            self.turn_in_progress=False
            self.first_turn=True


    def timerthrust_callback(self):
        msg_pos = Float64()
        msg_thrust = Float64()
        cmd_dist = self.dist_cmd_motors
        
        if(self.turn_in_progress == False):
            if(cmd_dist>=100.0):
                self.pos=0.0
                self.thrust=12000
            else:
                if(cmd_dist>=50):
                    self.pos=0.0
                    self.thrust=5000
                else:
                    if(cmd_dist>=20):
                        self.pos=0.0
                        self.thrust=1000
                    else:
                        if(cmd_dist>10):
                            self.pos=0.0
                            self.thrust=100
                        else:
                            if(cmd_dist<5):
                                self.pos=0.0
                                self.thrust=0.0
            
        else:
            self.get_logger().info('Wait the USV turn')

        msg_pos.data= self.pos
        msg_thrust.data = self.thrust
        # On publie les messages les messages
        self.pub_pos.publish(msg_pos)
        self.pub_thrust.publish(msg_thrust)
        # Affichage de la commande envoyé
        self.get_logger().info('Envoi: pos = %f, thrust = %f' % (msg_pos.data, msg_thrust.data))                

                


            

    def cmd_motors_callback(self, msg):
        #self.angle_cmd_motors, self.dist_cmd_motors = msg.data
        self.get_logger().info('Received: angle = %f, distance = %f' % (self.angle_cmd_motors, self.dist_cmd_motors))


def main(args=None):
    rclpy.init(args=args)

    node = MyCmdMotors()
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()