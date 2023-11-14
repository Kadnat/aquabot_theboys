#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

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
        self.prev_pos = 0.0
        self.prev_thrust = 0.0
        self.aligning = False  # Ajoutez un indicateur pour savoir si le bateau est en train de s'aligner
        self.angle_history = [0.0] * 10  # Ajoutez un historique des angles

    def timer_callback(self):
        msg_pos = Float64()
        msg_thrust = Float64()

        # Si l'angle est proche de zéro, avancez tout droit
        if abs(self.angle_cmd_motors) < 0.2:
            self.pos = 0.0
            target_thrust = 3000.0  # Vitesse maximale
            self.aligning = False  # Le bateau est maintenant aligné
        # Sinon, tournez sur place pour vous aligner avec la bouée
        else:
            target_pos = math.copysign(math.pi / 4, -self.angle_cmd_motors)  # Tournez à gauche ou à droite en fonction de l'angle
            target_thrust = 200.0
            self.aligning = True  # Le bateau est en train de s'aligner

        # Si le bateau est en train de s'aligner, réduisez progressivement la position du moteur
        if self.aligning:
            ramp_rate = 0.1  # Taux de changement maximal par appel de fonction
            self.pos = self.prev_pos + ramp_rate * (target_pos - self.prev_pos)
            self.thrust = 210.0
        else:
            # Vérifiez si l'angle a peu changé au cours des 10 derniers appels de fonction
            if max(self.angle_history) - min(self.angle_history) < 0.01:
                # Appliquez la logique de rampe pour la poussée
                ramp_rate = 0.1  # Taux de changement maximal par appel de fonction
                self.thrust = self.prev_thrust + ramp_rate * (target_thrust - self.prev_thrust)

        msg_pos.data= self.pos
        msg_thrust.data = self.thrust
        # On publie les messages les messages
        self.pub_pos.publish(msg_pos)
        self.pub_thrust.publish(msg_thrust)
        # Affichage de la commande envoyé
        self.get_logger().info('Envoi: pos = %f, thrust = %f' % (msg_pos.data, msg_thrust.data))  

        # Mémorisez la position et la poussée pour la prochaine fois
        self.prev_pos = self.pos
        self.prev_thrust = self.thrust

        # Mettez à jour l'historique des angles
        self.angle_history.pop(0)
        self.angle_history.append(self.angle_cmd_motors)

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
