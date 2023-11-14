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
        timer_period = 0.1  # Période du timer en secondes
        # Appel la fonction pour la cmd moteurs toutes les 100ms
        self.timerthrust = self.create_timer(timer_period, self.timer_callback)
        self.create_subscription(Float64MultiArray, '/cmd_motors', self.cmd_motors_callback, 10)
        self.pos = 0.0
        self.thrust = 0.0
        self.angle_cmd_motors=0.0
        self.dist_cmd_motors=0.0
        self.turn_in_progress=False
        self.first_turn=True
        self.first_in = 0
        self.stop_turn=False
        self.is_accelerating=False

    def position_corrector(self):

        msg_pos = Float64()
        msg_thrust = Float64()

        while (((self.angle_cmd_motors >= 0) & (self.angle_cmd_motors <= 0.2))|((self.angle_cmd_motors >= -0.2) & (self.angle_cmd_motors <= 0))):
            # if ((self.angle_cmd_motors >= 0.2) & (self.angle_cmd_motors <= math.pi/2)):
            #     self.pos = -math.pi/8
            #     self.thrust = 500.0
            # elif ((self.angle_cmd_motors >= math.pi/2) & (self.angle_cmd_motors <= math.pi)):
            #     self.pos = -math.pi/4
            #     self.thrust = 500.0
            # elif ((self.angle_cmd_motors >= -math.pi) & (self.angle_cmd_motors <= -math.pi/2)):
            #     self.pos = math.pi/4
            #     self.thrust = 500.0
            # elif ((self.angle_cmd_motors >= -math.pi/2) & (self.angle_cmd_motors <= -0.2)):
                self.pos = math.pi/8
                self.thrust = 250.0

                msg_pos.data= self.pos
                msg_thrust.data = self.thrust
                # On publie les messages les messages
                self.pub_pos.publish(msg_pos)
                self.pub_thrust.publish(msg_thrust)



    def motor_controller(self):
        # cmd_dist = self.dist_cmd_motors
        # if(self.is_accelerating==False):
        #     if(cmd_dist>=100.0):
        #         self.position_corrector()
        #         self.acceleration(5000.0)
        #     else:
        #         if(cmd_dist>=50):
        #             #self.decceleration()
        #             self.position_corrector()
        #             self.acceleration(5000.0)
        #         else:
        #             if(cmd_dist>=20):
        #                 #self.decceleration()
        #                 self.position_corrector()
        #                 self.acceleration(1000.0)
        #             else:
        #                 if(cmd_dist>10):
        #                     #self.decceleration()
        #                     self.position_corrector()
        #                     self.acceleration(100.0)
        #                 else:
        #                     if(cmd_dist<5):
        #                         self.pos=0.0
        #                         self.thrust=0.0

        while (self.dist_cmd_motors<5):
            
            if(((self.angle_cmd_motors >= 0) & (self.angle_cmd_motors <= 0.2))|((self.angle_cmd_motors >= -0.2) & (self.angle_cmd_motors <= 0))):
                self.decceleration()
                self.position_corrector()
            else:
                self.acceleration(5000.0)

             

    def timer_callback(self):
        msg_pos = Float64()
        msg_thrust = Float64()

        self.motor_controller()

        msg_pos.data= self.pos
        msg_thrust.data = self.thrust
        # On publie les messages les messages
        self.pub_pos.publish(msg_pos)
        self.pub_thrust.publish(msg_thrust)
        # Affichage de la commande envoyé
        self.get_logger().info('Envoi: pos = %f, thrust = %f' % (msg_pos.data, msg_thrust.data))  

                
    def acceleration(self, final_value):
        # self.pos=0.0
        # self.thrust=0.0
        # msg_pos = Float64()
        # msg_thrust = Float64()
        # for i in range(3):
        #     self.is_accelerating=True
        #     self.thrust = self.thrust + final_value/3
        #     msg_pos.data= self.pos
        #     msg_thrust.data = self.thrust
        #     self.pub_pos.publish(msg_pos)
        #     self.pub_thrust.publish(msg_thrust)
        #     self.get_logger().info('Accel: pos = %f, thrust = %f' % (msg_pos.data, msg_thrust.data))    
        #     sleep(2)
        # self.is_accelerating=False

        self.pos=0.0
        msg_pos = Float64()
        msg_thrust = Float64()
        self.is_accelerating=True
        self.thrust = final_value
        msg_pos.data= self.pos
        msg_thrust.data = self.thrust
        self.pub_pos.publish(msg_pos)
        self.pub_thrust.publish(msg_thrust)
        self.get_logger().info('Accel: pos = %f, thrust = %f' % (msg_pos.data, msg_thrust.data))    
        #sleep(10)
        self.is_accelerating=False

    def decceleration(self):    
        msg_pos = Float64()
        msg_thrust = Float64()
        self.thrust = 0.0
        msg_pos.data= self.pos
        msg_thrust.data = self.thrust
        self.pub_pos.publish(msg_pos)
        self.pub_thrust.publish(msg_thrust)  
        sleep(5)
            

    def cmd_motors_callback(self, msg):
        self.angle_cmd_motors, self.dist_cmd_motors = msg.data
        #self.get_logger().info('Received: angle = %f, distance = %f' % (self.angle_cmd_motors, self.dist_cmd_motors))


def main(args=None):
    rclpy.init(args=args)

    node = MyCmdMotors()
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()