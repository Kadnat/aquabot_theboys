#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from math import atan2, pi
from std_msgs.msg import Float64, Float64MultiArray
from time import sleep


class MyCmdMotors(Node):
    def __init__(self):
        super().__init__('cmd_motors') 
        # position du moteur
        self.pub_pos = self.create_publisher(Float64, '/wamv/thrusters/main/pos', 10)
        # vitesse du propulseur
        self.pub_thrust = self.create_publisher(Float64, '/wamv/thrusters/main/thrust', 10)
        self.create_subscription(Float64MultiArray, '/position/to_reach', self.cmd_motors_callback, 10)
        self.create_subscription(Float64, "/position/orientation", self.orientation_callback, 10)
        self.create_subscription(Float64MultiArray, "/position/current", self.position_actual, 10)
        self.x_to_reach = 0.0
        self.y_to_reach = 0.0
        self.aligning = False  # Ajoutez un indicateur pour savoir si le bateau est en train de s'aligner
        self.x_actual = 0.0
        self.y_actual = 0.0
        self.current_orientation = 0.0

    def position_actual(self,msg):
        self.x_actual, self.y_actual = msg.data

    def orientation_callback(self,msg):
        self.current_orientation = msg.data

    def cmd_motors_callback(self, msg):
        msg_pos = Float64()
        msg_thrust = Float64()
        self.x_to_reach, self.y_to_reach = msg.data
        delta_x = (self.x_actual - self.x_to_reach) 
        delta_y = (self.y_actual - self.y_to_reach)
        #self.get_logger().info('Delta x : %f, Delta y : %f   TWO' % (delta_x, delta_y))
        #self.get_logger().info('Angle : %s' % str(atan2(delta_y, delta_x)))
        angle_diff = ((atan2(delta_y, delta_x) + self.current_orientation)) 
        if angle_diff > pi:
            angle_diff -= 2 * pi
        elif angle_diff < -pi:
            angle_diff += 2 * pi
        self.get_logger().info('Angle : %s' % str(angle_diff))
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
