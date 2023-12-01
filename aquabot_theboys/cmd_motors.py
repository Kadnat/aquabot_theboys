#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from math import atan2, pi
from std_msgs.msg import Float64, Float64MultiArray, Bool
from time import sleep


class MyCmdMotors(Node):
    def __init__(self):
        super().__init__('cmd_motors') 
        # Bot motor angle
        self.pub_pos = self.create_publisher(Float64, '/wamv/thrusters/main/pos', 10)
        # thrust speed
        self.pub_thrust = self.create_publisher(Float64, '/wamv/thrusters/main/thrust', 10)
        
        # Get position to reach
        self.create_subscription(Float64MultiArray, '/position/to_reach', self.cmd_motors_callback, 10)
        # Get actual orientation from the IMU
        self.create_subscription(Float64, "/position/orientation", self.orientation_callback, 10)
        # Get our actual position x,y
        self.create_subscription(Float64MultiArray, "/position/current", self.position_actual, 10)

        self.create_subscription(Bool, '/position/look_around', self.look_around, 10)

        self.create_subscription(Float64MultiArray, '/position/follow_without_moving',self.follow_without_moving, 10)

        # To know if we are changing its orientation
        self.aligning = False  
        # x,y variable
        self.x_to_reach = 0.0
        self.y_to_reach = 0.0
        self.x_actual = 0.0
        self.y_actual = 0.0
        # Our current orientation
        self.current_orientation = 0.0

    # To get our actual x,y position
    def position_actual(self,msg):
        self.x_actual, self.y_actual = msg.data

    # To get our actual orientation
    def orientation_callback(self,msg):
        self.current_orientation = msg.data

    def look_around(self,msg):
        msg_pos = Float64()
        msg_thrust = Float64()
        #if (pre_orientation%(2*pi)) <= ((self.current_orientation+0.1)%(2*pi)) and (pre_orientation%(2*pi)) >= ((self.current_orientation-0.1)%(2*pi)):
        if msg.data == True:
            msg_pos.data = -pi/2
        else:
            msg_pos.data = pi/2
        msg_thrust.data = 200.0
        # else:
        #     msg_pos.data = 0.0
        #     msg_thrust.data = 0.0
        self.pub_pos.publish(msg_pos)
        self.pub_thrust.publish(msg_thrust)

    def follow_without_moving(self, msg):
        msg_pos = Float64()
        msg_thrust = Float64()
        self.x_to_reach, self.y_to_reach = msg.data
        delta_x = (self.x_actual - self.x_to_reach) 
        delta_y = (self.y_actual - self.y_to_reach)
        #self.get_logger().info('Delta x : %f, Delta y : %f   TWO' % (delta_x, delta_y))
        #self.get_logger().info('Angle : %s' % str(atan2(delta_y, delta_x)))
        # Calculating the angle we have to add at our orientation to point the object we want to reach
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

    # To control motors with a x,y command
    def cmd_motors_callback(self, msg):
        msg_pos = Float64()
        msg_thrust = Float64()
        #self.get_logger().info('Current_pos : (%f;%f), pos to reach : (%f;%f)' % (self.x_actual, self.y_actual, self.x_to_reach, self.y_to_reach))
        self.x_to_reach, self.y_to_reach = msg.data
        delta_x = (self.x_actual - self.x_to_reach) 
        delta_y = (self.y_actual - self.y_to_reach)
        #self.get_logger().info('Delta x : %f, Delta y : %f   TWO' % (delta_x, delta_y))
        #self.get_logger().info('Angle : %s' % str(atan2(delta_y, delta_x)))
        # Calculating the angle we have to add at our orientation to point the object we want to reach
        angle_diff = ((atan2(delta_y, delta_x) + self.current_orientation) - pi) % (2*pi) 
        # A VOIR SI ON LAISSE
        if angle_diff > pi:
            angle_diff -= 2 * pi
        elif angle_diff < -pi:
            angle_diff += 2 * pi
        #self.get_logger().info('Angle : %s' % str(angle_diff))
        # If the position we want to reach is too near to our actual position, we stop the motors
        if ((self.x_to_reach <= self.x_actual +5) and (self.x_to_reach >= self.x_actual -5)) and ((self.y_to_reach <= self.y_actual +5) and (self.y_to_reach >= self.y_actual -5)):
            msg_pos.data = 0.0
            self.pub_pos.publish(msg_pos)
            msg_thrust.data = 0.0
            self.pub_thrust.publish(msg_thrust)

        else :
            # If the angle between us and the object we wand to reach is over 0.1 radians, we change our orientation
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
                # If we were moving our boat before getting here, we stop them and wait until the position of the motors is not 0
                if self.aligning  == True:
                    self.aligning  = False
                    msg_pos.data = 0.0
                    self.pub_pos.publish(msg_pos)
                    msg_thrust.data = 0.0
                    self.pub_thrust.publish(msg_thrust)
                    sleep(3)
                    #self.get_logger().info('good')
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
