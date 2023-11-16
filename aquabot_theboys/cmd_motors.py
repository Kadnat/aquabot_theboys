#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from math import sqrt, atan2, pi, sin, cos, radians
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image, Imu, NavSatFix
from time import sleep
import pyproj

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
        self.create_subscription(NavSatFix, "/wamv/sensors/gps/gps/fix", self.gps_callback, 10)
        self.create_subscription(Float64, "/current_pos", self.imu_callback, 10)
        self.pos = 0.0
        self.thrust = 0.0
        ###vairable x et y a la place des deux suivantes###
        self.angle_cmd_motors=0.0
        self.dist_cmd_motors=0.0
        self.x_to_reach = 0.0
        self.y_to_reach = 0.0
        #########
        self.prev_pos = 0.0
        self.prev_thrust = 0.0
        self.aligning = False  # Ajoutez un indicateur pour savoir si le bateau est en train de s'aligner
        self.angle_history = [0.0] * 10  # Ajoutez un historique des angles
        self.x_actual = 0.0
        self.y_actual = 0.0
        self.current_orientation = 0.0
        #### à voir pour ajouter l'orientation actuel

    def imu_callback(self,msg):
        self.current_orientation = msg.data

    def gps_callback(self, msg):
        # On définit les parametres de projection
        proj_latlon = pyproj.Proj(proj='latlong', datum='WGS84')
        proj_xy = pyproj.Proj(proj="utm", zone=44, datum='WGS84')
        # On convertit les données
        self.x_actual, self.y_actual = pyproj.transform(proj_latlon, proj_xy, msg.latitude, msg.longitude)

        #self.get_logger().info('GPS callback executed, current position: %s' % str(self.current_position))

    def timer_callback(self):
        msg_pos = Float64()
        msg_thrust = Float64()
        delta_x = self.x_to_reach-self.x_actual
        delta_y = self.y_to_reach-self.y_actual
        # self.get_logger().info('x actual : %s' % str(self.x_actual))
        # self.get_logger().info('y actual : %s' % str(self.y_actual))
        # self.get_logger().info('x to reach : %s' % str(self.x_to_reach))
        # self.get_logger().info('y to reach : %s' % str(self.y_to_reach))
        ########################################
        angle_diff = atan2(delta_y, delta_x) - self.current_orientation
        while angle_diff > pi:
            angle_diff -= 2 * pi
        while angle_diff < -pi:
            angle_diff += 2 * pi
        msg_pos.data = pi/4
        #if abs(angle_diff)>abs(angle_diff-pi):
        angle_diff=angle_diff-pi
        if angle_diff<0:
            msg_pos.data = -pi/4
        self.thrust = 1000.0 
        rotation_time = abs(angle_diff) / ((pi / 4) * (12000/abs(self.thrust)))
        self.get_logger().info('Angle diff : %s' % str(angle_diff))
        if abs(angle_diff)>0.5:
            self.pub_pos.publish(msg_pos)
            msg_thrust.data = self.thrust
            self.pub_thrust.publish(msg_thrust)
            self.get_logger().info('On va tourner pendant : %s' % str(rotation_time))
            ## on dort jusq'à que le moteur soit en position
            sleep(rotation_time)
            msg_pos.data = 0.0
            self.pub_pos.publish(msg_pos)
            ## on dort jusq'à que le moteur revienne en position initiale
            sleep(rotation_time)
        self.thrust = 5000.0 # peut etre un peu rapide, a voir en test
        # A voir si on update pas sans cesse la position à rejoindre (timer inutile si on update)
        #distance = sqrt((delta_x)**2 + (delta_y)**2)
        #time_to_travel = distance/self.thrust
        msg_thrust.data = self.thrust
        self.pub_thrust.publish(msg_thrust)
        #sleep(time_to_travel) # a voir si on laisse ducoup


        

    # def timer_callback(self):
    #     msg_pos = Float64()
    #     msg_thrust = Float64()

    #     # Si l'angle est proche de zéro, avancez tout droit
    #     if abs(self.angle_cmd_motors) < 0.2:
    #         self.pos = 0.0
    #         target_thrust = 3000.0  # Vitesse maximale
    #         self.aligning = False  # Le bateau est maintenant aligné
    #     # Sinon, tournez sur place pour vous aligner avec la bouée
    #     else:
    #         target_pos = math.copysign(math.pi / 4, -self.angle_cmd_motors)  # Tournez à gauche ou à droite en fonction de l'angle
    #         target_thrust = 200.0
    #         self.aligning = True  # Le bateau est en train de s'aligner

    #     # Si le bateau est en train de s'aligner, réduisez progressivement la position du moteur
    #     if self.aligning:
    #         ramp_rate = 0.1  # Taux de changement maximal par appel de fonction
    #         self.pos = self.prev_pos + ramp_rate * (target_pos - self.prev_pos)
    #         self.thrust = 210.0
    #     else:
    #         # Vérifiez si l'angle a peu changé au cours des 10 derniers appels de fonction
    #         if max(self.angle_history) - min(self.angle_history) < 0.01:
    #             # Appliquez la logique de rampe pour la poussée
    #             ramp_rate = 0.1  # Taux de changement maximal par appel de fonction
    #             self.thrust = self.prev_thrust + ramp_rate * (target_thrust - self.prev_thrust)

    #     msg_pos.data= self.pos
    #     msg_thrust.data = self.thrust
    #     # On publie les messages les messages
    #     self.pub_pos.publish(msg_pos)
    #     self.pub_thrust.publish(msg_thrust)
    #     # Affichage de la commande envoyé
    #     self.get_logger().info('Envoi: pos = %f, thrust = %f' % (msg_pos.data, msg_thrust.data))  

    #     # Mémorisez la position et la poussée pour la prochaine fois
    #     self.prev_pos = self.pos
    #     self.prev_thrust = self.thrust

    #     # Mettez à jour l'historique des angles
    #     self.angle_history.pop(0)
    #     self.angle_history.append(self.angle_cmd_motors)

    def cmd_motors_callback(self, msg):
        angle, distance = msg.data
        dY = distance * cos(radians(angle))  # changement en y
        dX = distance * sin(radians(angle))  # changement en x
        self.x_to_reach = self.x_actual + dX
        self.y_to_reach = self.y_actual + dY
        
        #self.angle_cmd_motors, self.dist_cmd_motors = msg.data
        #self.get_logger().info('Received: angle = %f, distance = %f' % (self.angle_cmd_motors, self.dist_cmd_motors))


def main(args=None):
    rclpy.init(args=args)

    node = MyCmdMotors()
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
