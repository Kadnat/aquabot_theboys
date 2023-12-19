import rclpy
from rclpy.node import Node
from enum import Enum
from std_msgs.msg import Float64MultiArray, Bool, Float64, UInt32
from geometry_msgs.msg import PoseStamped
from math import cos, radians
from time import sleep

####################CODE A COMMENTER AVEC JULES
#Problème de path si mis dans un autre fichier 
class MyPatrolAlgo(): 
    def __init__(self, x, y):
        # Buoy position
        self.x_buoy = x    
        self.y_buoy = y
        # Listes nécéssaires
        self.M_s = ['spawn',0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0, 30.0, 31.0, 32.0, 33.0, 34.0, 35.0, 36.0, 37.0, 38.0, 39.0, 40.0, 41.0, 42.0, 43.0, 44.0, 45.0, 46.0, 47.0, 48.0, 49.0, 50.0, 51.0, 52.0, 53.0, 54.0, 55.0, 56.0, 57.0, 58.0, 59.0, 60.0, 61.0, 62.0, 63.0, 64.0, 65.0, 66.0, 67.0, 68.0, 69.0, 70.0, 71.0]       
                        # Liste servant à numéroter les points de la spirales
        self.X_s = [-self.x_buoy,0.0, -82.5, -0.0, 115.5, 39.7705, -114.7995, -125.4785, 52.0076, 181.5, 96.25, -101.75, -214.5, -112.75, 118.25, 247.5, 160.1923, -59.27, -248.4743, -256.9692, -65.5642, 189.5854, 313.5, 227.5116, 0.0, -239.1789, -346.5, -250.8461, -0.0, 262.5134, 379.5, 296.3315, 68.4463, -200.75, -384.1777, -391.0687, -211.75, 74.8134, 335.6551, 445.5, 365.7566, 141.7461, -143.7856, -381.7751, -478.5, -392.4541, -151.9437, 153.9832, 408.4727, 511.5, 435.3487, 217.4698, -75.3557, -350.6779, -519.5654, -525.3224, -362.4654, -79.6252, 234.9172, 480.7764, 577.5, 504.8928, 294.25, 0.0, -299.75, -523.9454, -610.5, -533.4716, -310.75, -0.0, 316.25, 552.5242, 643.5]        
                        # Liste répertoriant l'abscisse d'un point M_s[k] de la spirale
        self.Y_s = [-self.y_buoy,66.0, 0.0, -99.0, -0.0, 122.401, 83.4067, -91.1655, -160.0628, -0.0, 166.7099, 176.2362, -0.0, -195.2887, -204.815, -0.0, 200.8748, 259.679, 119.6589, -123.7498, -287.2555, -237.7326, -0.0, 227.5116, 330.0, 239.1789, -0.0, -250.8461, -363.0, -262.5134, -0.0, 248.6517, 388.1784, 347.7092, 139.8292, -142.3374, -366.7618, -424.288, -281.6481, -0.0, 265.7377, 436.2496, 442.5266, 277.3759, -0.0, -285.1346, -467.6345, -473.9115, -296.7728, -0.0, 279.7816, 476.1923, 524.1105, 404.7039, 152.5582, -154.2486, -418.3074, -553.8051, -514.3969, -308.9762, -0.0, 291.5, 509.656, 594.0, 519.1822, 302.5, 0.0, -308.0, -538.2348, -627.0, -547.7611, -319.0, -0.0]        
                        # Liste répertoriant l'abscisse d'un point M_s[k] de la spirale
        self.N_s = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]       
                        # Liste donnant l'état d'un point M_s[k] de la spirale : Si N_s[k]=0 point dans la map, Si N_s[k]=1 point hors zone
        ### Préparation à la définition des points du chemin prévisionnel ###
        # Orientation de la spirale autour de la boué
        self.add_value(self.X_s, self.x_buoy)
        self.add_value(self.Y_s, self.y_buoy)
        # Initilisation des variables du chemin prévisionnel
        self.M_c = ['spawn',0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                        # Liste servant à numéroter les points du chemin prévisionnel
        self.X_c = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                        # Liste répertoriant l'abscisses d'un point M_c[k] du chemin prévisionnel
        self.Y_c = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                        # Liste répertoriant l'ordonnées d'un point M_c[k] du chemin prévisionnel
        self.N_c = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]       # Liste donnant l'état du point M_c[k] du chemin prévisionnel :
                        # Si N_c[k] = 0 point non réajusté, Si N_c[k] = 1 point réajusté
        self.k = 1           # Constante d'incrémentation fixé à 1 pour initialisation

    # Pour orienter la spirale avec comme centre la boué
    def add_value(self, liste, A):
        i = 0
        while i < len(liste):
            liste[i]=liste[i]+A
            i+=1
        return liste
    
    # Calcul d'une ordonnée à l'origine
    def calcul_m(self, x_1,x_2,y_1,y_2):
        return (y_2-y_1)/(x_2-x_1)

    # Transforming ennemy x,y position in gps position
    def patrol_algo(self):
        ### définition des points du chemin prévisionnel ###
        while self.k < len(self.X_s):
            # M_s[k] est dans la map et M_s[k-1] est en dehors donc on peut réajuster M[k-1] sur une bordure
            if self.N_s[self.k-1] == 1 and self.X_s[self.k] < 280 and self.X_s[self.k] > -280 and self.Y_s[self.k] < 280 and self.Y_s[self.k] > -280:

                # Si M[self.k-1] est dans la zone 2 : on le réajuste dans la zone 1
                if self.X_s[self.k-1] > 280 and self.Y_s[self.k-1] > -280 and self.Y_s[self.k-1] < 280:
                    self.X_c[self.k-1]=280
                    m = self.calcul_m(self.X_s[self.k],self.X_s[self.k-1],self.Y_s[self.k],self.Y_s[self.k-1])
                    p = self.Y_s[self.k-1]-m*self.X_s[self.k-1]
                    self.Y_c[self.k-1]=m*280+p
                    self.N_c[self.k-1]=1
                    self.N_s[self.k-1]=1

                    # M[self.k] est dans la zone 1
                    self.X_c[self.k]=self.X_s[self.k]
                    self.Y_c[self.k]=self.Y_s[self.k]
                    self.N_c[self.k]=0
                    self.N_s[self.k]=0
                    self.k+=1
                
                # Si M[self.k] est dans la zone 3
                elif self.X_s[self.k-1] < -280 and self.Y_s[self.k-1] > -280 and self.Y_s[self.k-1] < 280:
                    self.X_c[self.k-1]=-280
                    m = self.calcul_m(self.X_s[self.k],self.X_s[self.k-1],self.Y_s[self.k],self.Y_s[self.k-1])
                    p = self.Y_s[self.k-1]-m*self.X_s[self.k-1]
                    self.Y_c[self.k-1]=-m*280+p
                    self.N_c[self.k-1]=1
                    self.N_s[self.k-1]=1

                    # M[self.k] est dedans
                    self.X_c[self.k]=self.X_s[self.k]
                    self.Y_c[self.k]=self.Y_s[self.k]
                    self.N_c[self.k]=0
                    self.N_s[self.k]=0
                    self.k+=1
                
                # Si M[self.k] est dans la zone 4
                elif self.Y_s[self.k-1] > 280 and self.X_s[self.k-1] > -280 and self.X_s[self.k-1] < 280:
                    self.Y_c[self.k-1]=280
                    m = self.calcul_m(self.X_s[self.k],self.X_s[self.k-1],self.Y_s[self.k],self.Y_s[self.k-1])
                    p = self.Y_s[self.k-1]-m*self.X_s[self.k-1]
                    self.X_c[self.k-1]=(280-p)/m
                    self.N_c[self.k-1]=1
                    self.N_s[self.k-1]=1

                    # M[self.k] est dedans
                    self.X_c[self.k]=self.X_s[self.k]
                    self.Y_c[self.k]=self.Y_s[self.k]
                    self.N_c[self.k]=0
                    self.N_s[self.k]=0
                    self.k+=1

                # Si M[self.k] est dans la zone 5
                elif self.Y_s[self.k-1] < -280 and self.X_s[self.k-1] > -280 and self.X_s[self.k-1] < 280:
                    self.Y_c[self.k-1]=-280
                    m = self.calcul_m(self.X_s[self.k],self.X_s[self.k-1],self.Y_s[self.k],self.Y_s[self.k-1])
                    p = self.Y_s[self.k-1]-m*self.X_s[self.k-1]
                    self.X_c[self.k-1]=(-280-p)/m
                    self.N_c[self.k-1]=1
                    self.N_s[self.k-1]=1

                    # M[self.k] est dedans
                    self.X_c[self.k]=self.X_s[self.k]
                    self.Y_c[self.k]=self.Y_s[self.k]
                    self.N_c[self.k]=0
                    self.N_s[self.k]=0
                    self.k+=1
                
                else :
                    del(self.X_c[self.k-1],self.Y_c[self.k-1],self.N_c[self.k-1],self.X_s[self.k-1],self.Y_s[self.k-1],self.N_s[self.k-1])
                    
                    # M[self.k] est dedans
                    self.X_c[self.k-1]=self.X_s[self.k-1]
                    self.Y_c[self.k-1]=self.Y_s[self.k-1]
                    self.N_c[self.k-1]=0
                    self.N_s[self.k-1]=0

            # Le point est dans la map
            elif self.X_s[self.k] < 280 and self.X_s[self.k] > -280 and self.Y_s[self.k] < 280 and self.Y_s[self.k] > -280:
                self.X_c[self.k]=self.X_s[self.k]
                self.Y_c[self.k]=self.Y_s[self.k]
                self.N_c[self.k]=0
                self.N_s[self.k]=0
                self.k+=1

            # Point en dehors de la map
            else :
                if self.N_c[self.k-1] == 1:
                    del(self.X_c[self.k],self.Y_c[self.k],self.N_c[self.k],self.X_s[self.k],self.Y_s[self.k],self.N_s[self.k])

                # Si M[self.k] est dans la zone 2
                elif self.X_s[self.k] > 280 and self.Y_s[self.k] > -280 and self.Y_s[self.k] < 280:
                    self.X_c.append(280)
                    m = self.calcul_m(self.X_s[self.k-1],self.X_s[self.k],self.Y_s[self.k-1],self.Y_s[self.k])
                    p = self.Y_s[self.k]-m*self.X_s[self.k]
                    self.Y_c.append(m*280+p)
                    self.N_c[self.k]=1
                    self.N_s[self.k]=1
                    self.k+=1
                
                # Si M[self.k] est dans la zone 3
                elif self.X_s[self.k] < -280 and self.Y_s[self.k] > -280 and self.Y_s[self.k] < 280:
                    self.X_c[self.k]=-280
                    m = self.calcul_m(self.X_s[self.k-1],self.X_s[self.k],self.Y_s[self.k-1],self.Y_s[self.k])
                    p = self.Y_s[self.k]-m*self.X_s[self.k]
                    self.Y_c[self.k]=-m*280+p
                    self.N_c[self.k]=1
                    self.N_s[self.k]=1
                    self.k+=1
                
                # Si M[self.k] est dans la zone 4
                elif self.Y_s[self.k] > 280 and self.X_s[self.k] > -280 and self.X_s[self.k] < 280:
                    self.Y_c[self.k]=280
                    m = self.calcul_m(self.X_s[self.k-1],self.X_s[self.k],self.Y_s[self.k-1],self.Y_s[self.k])
                    p = self.Y_s[self.k]-m*self.X_s[self.k]
                    self.X_c[self.k]=(280-p)/m
                    self.N_c[self.k]=1
                    self.N_s[self.k]=1
                    self.k+=1

                # Si M[self.k] est dans la zone 5
                elif self.Y_s[self.k] < -280 and self.X_s[self.k] > -280 and self.X_s[self.k] < 280:
                    self.Y_c[self.k]=-280
                    m = self.calcul_m(self.X_s[self.k-1],self.X_s[self.k],self.Y_s[self.k-1],self.Y_s[self.k])
                    p = self.Y_s[self.k]-m*self.X_s[self.k]
                    self.X_c[self.k]=(-280-p)/m
                    self.N_c[self.k]=1
                    self.N_s[self.k]=1
                    self.k+=1
                
                else :
                    del(self.X_c[self.k],self.Y_c[self.k],self.N_c[self.k],self.X_s[self.k],self.Y_s[self.k],self.N_s[self.k])
        return self.X_c, self.Y_c



# State machine enum
class State(Enum):
    REACH_ZONE = 0
    PATROL = 1
    FOLLOW = 2
    EXIT_BUOY = 3
    GO_TO_POS_ENNEMY = 4


class MyHub(Node):
    def __init__(self):
        super().__init__('hub') 
        # Position à rejoindre
        self.pub_pos_to_reach = self.create_publisher(Float64MultiArray, '/position/to_reach', 10)
        # Demande au moteurs de faire tourner le bateau sur lui même
        self.pub_look_around = self.create_publisher(Bool, '/position/look_around', 10)

        # Appel de la machine à états toutes les 100 ms      
        timer_period_1 = 0.1  
        self.create_timer(timer_period_1, self.hub_callback)
        # Récupération de la position x,y de la bouée
        self.create_subscription(Float64MultiArray, '/position/buoy', self.position_buoy_callback, 10)
        # Récupération de la position x,y de notre bateau
        self.create_subscription(Float64MultiArray, '/position/current', self.current_pos_callback, 10)
        # Récupération de la position x,y de l'ennemi
        self.create_subscription(Float64MultiArray, '/position/ennemy', self.ennemy_position_callback, 10)
        # Afin de savoir si l'ennemi a été détecté par la caméra
        self.create_subscription(Bool, 'object_detected', self.ennemy_finded_callback, 10)
        # Récupération de l'orientation de notre bateau
        self.create_subscription(Float64, "/position/orientation", self.orientation_callback, 10)
        # Récupération de l'état de la simulation  => 1: Rejoindre zone de patrouille; 2: Zone de patrouille rejoint; 3: Bateau ennemi détecté
        self.create_subscription(UInt32, '/vrx/patrolandfollow/current_phase', self.current_phase_callback, 10)
        # Récupération de l'angle qui nous sépare entre notre bateau et l'ennemi (estimé par la caméra)
        self.create_subscription(Float64, 'object_position', self.ennemy_angle_callback, 10)
        # Récupération des positions gps de l'ennemi
        self.create_subscription(PoseStamped, '/wamv/ais_sensor/ennemy_position', self.ennemy_position_given_callback, 10)
        
        # Variables de position
        self.x_buoy = 0.0
        self.y_buoy = 0.0
        self.x_actual = 0.0
        self.y_actual = 0.0
        self.x_obstacle = 0.0
        self.y_obstacle = 0.0
        self.x_patrol = []
        self.y_patrol = []
        self.x_patrol_point = 0.0
        self.y_patrol_point = 0.0
        self.x_ennemy = 0.0
        self.y_ennemy = 0.0
        # Ennemi détecté
        self.ennemy_detected = False
        # Compteur pour laisser le temps au premiers messages ROS d'apparaitre
        self.ctr_in_state = 0
        # Variable d'états
        self.state_hub = State.REACH_ZONE
        self.previous_state = State.REACH_ZONE
        # Orientation de notre bateau
        self.orientation = 0.0
        # Angle qui nous sépare du bateau ennemi (estimé par la caméra)
        self.angle_camera = 0.0
        # Phase de la simulation
        self.phase = 0
        # Pour savoir si on vient de s'aligner
        self.aligning = False
        # Pour savoir si la position de l'ennemi nous a été donné
        self.ennemy_position_given = False
        self.get_logger().info('Reaching Zone')

    # Récupération de l'orientation de notre bateau
    def orientation_callback(self, msg):
        self.orientation = msg.data

    # Récupération de la position x,y de notre bateau
    def current_pos_callback(self, msg):
        self.x_actual, self.y_actual = msg.data

    # Récupération de la position x,y de la bouée
    def position_buoy_callback(self, msg):
        self.x_buoy, self.y_buoy = msg.data   
        #self.get_logger().info('Position boué : (%f;%f)' % (self.x_buoy, self.y_buoy)) 

    # Récupération de la position x,y de l'ennemi
    def ennemy_position_callback(self,msg):
        self.x_ennemy, self.y_ennemy = msg.data

    # Afin de savoir si l'ennemi a été détecté par la caméra
    def ennemy_finded_callback(self,msg):
        self.ennemy_detected = msg.data

    # Récupération de l'angle qui nous sépare entre notre bateau et l'ennemi (estimé par la caméra)
    def ennemy_angle_callback(self, msg):
        self.angle_camera = msg.data
    
    # Récupération de l'état de la simulation  => 1: Rejoindre zone de patrouille; 2: Zone de patrouille rejoint; 3: Bateau ennemi détecté
    def current_phase_callback(self,msg):
        self.phase = msg.data

    # Pour savoir si x2,y2 est dans un rayon delta de x1,y1 
    def are_near(self, x1,y1,x2,y2, delta):
        if((x1<=x2+delta) and (x1>=x2-delta)) and ((y1<=y2+delta) and (y1>=y2-delta)):
            return True
        else:
            return False

    # Pour récupéré la position de l'ennemi donné au bout de 180 secondes
    def ennemy_position_given_callback(self, msg):
        msg_to_send = PoseStamped()
        msg_to_send._pose._position.x = msg.pose.position.x
        msg_to_send.pose.position.y =msg.pose.position.y
        self.pub_gps_ennemy.publish(msg_to_send)
        # Coordonnées GPS de l'origine
        lat_ref = 48.04631295
        lon_ref = -4.9763167
        # Rayon de la terre
        R = 6378137

        lat_rad = radians(msg.latitude)
        lon_rad = radians(msg.longitude)
        lat_ref_rad = radians(lat_ref)
        lon_ref_rad = radians(lon_ref)

        self.x_ennemy = R * (lon_rad - lon_ref_rad) * cos(lat_ref_rad)
        self.y_ennemy = R * (lat_rad - lat_ref_rad)
        self.state_hub = State.GO_TO_POS_ENNEMY

    # Machine à états
    def hub_callback(self):
        msg_pos_to_reach = Float64MultiArray()
        # Rejoindre la bouée
        if self.state_hub == State.REACH_ZONE:
            if self.ctr_in_state < 100:
                self.ctr_in_state += 1
            # Si nous avons atteint la phase 2
            if self.phase == 2:
                patrol = MyPatrolAlgo(self.x_buoy, self.y_buoy)
                self.x_patrol, self.y_patrol = patrol.patrol_algo()
                self.state_hub = State.EXIT_BUOY
                # On demande a arreter les moteurs
                msg_pos_to_reach.data = [self.x_actual, self.y_actual] 
                self.pub_pos_to_reach.publish(msg_pos_to_reach)
                self.get_logger().info('Go patrol')
                self.x_patrol_point = self.x_actual
                self.y_patrol_point = self.y_actual
                self.ctr_in_state = 0
            else :
                # Si nous ne sommes pas arrivé à la phase deux, on rejoint la bouée
                msg_pos_to_reach.data = [self.x_buoy, self.y_buoy] 
                self.pub_pos_to_reach.publish(msg_pos_to_reach)
        # Cet état nous permet de nous décaler de la bouée pour pouvoir nous déplacer rapidement
        elif self.state_hub == State.EXIT_BUOY:
            if self.are_near(self.x_actual, self.y_actual, self.x_buoy+30.0, self.y_buoy+30.0, 5.0)==True:
                msg_pos_to_reach.data = [self.x_actual, self.y_actual] 
                self.pub_pos_to_reach.publish(msg_pos_to_reach)
                self.state_hub = State.PATROL
            else:
                msg_pos_to_reach.data = [self.x_buoy+35.0, self.y_buoy+35.0] 
                self.pub_pos_to_reach.publish(msg_pos_to_reach)
        # Cet état nous fait tourner sur nous même jusqu'au moment où on détecte l'ennemi
        elif self.state_hub == State.PATROL:
            if (self.ennemy_detected == True) and (self.ctr_in_state < 5):
                self.ctr_in_state += 1
            else:
                self.ctr_in_state = 0
            if (self.ennemy_detected == True) and (self.ctr_in_state>=5):
                self.state_hub = State.FOLLOW
                self.get_logger().info('Follow the ennemy')
                self.ctr_in_state = 0
            else:
                msg_turn_around = Bool()
                msg_turn_around.data = True
                self.pub_look_around.publish(msg_turn_around)
        # On rejoint le bateau
        elif self.state_hub == State.FOLLOW:
            if self.ennemy_detected == True and abs(self.angle_camera)<0.3:
                # Si nous sommes trop proche ou que nous venons de nous aligner, on remet le moteur en position initiale
                if (self.are_near(self.x_actual, self.y_actual, self.x_ennemy, self.y_ennemy, 20.0)==True) and (self.aligning  == True):
                    msg_pos_to_reach.data = [self.x_actual, self.y_actual] 
                    self.pub_pos_to_reach.publish(msg_pos_to_reach)
                    self.aligning  = False
                    sleep(3)
                    self.get_logger().info('In Front')
                # Sinon on rejoint la position de l'ennemi
                else:
                    msg_pos_to_reach.data = [self.x_ennemy, self.y_ennemy] 
                    self.pub_pos_to_reach.publish(msg_pos_to_reach)
                    self.get_logger().info('Follow it')
            # Si l'ennemi est détecté mais qu'il n'est pas en face, on tourne jusqu'à avoir le bateau en face
            elif (self.ennemy_detected == True):
                msg_bool = Bool()
                if self.angle_camera >0:
                    msg_bool.data = False
                else:
                    msg_bool.data = True
                self.pub_look_around.publish(msg_bool) 
                self.get_logger().info('Detected but not in front')
                self.aligning  = True      
            # Si le bateau n'est plus détecté, on tourne sur nous même
            else:
                msg_turn_around = Bool()
                msg_turn_around.data = False
                self.get_logger().info('Not detected')
                self.pub_look_around.publish(msg_turn_around)
                self.aligning  = True
        # On rejoint la position de l'ennemi fournie
        elif self.state_hub == State.GO_TO_POS_ENNEMY:
            self.get_logger().info('In Go TO POS ENNEMY')
            if (self.are_near(self.x_actual, self.y_actual, self.x_ennemy, self.y_ennemy, 20.0)==True):
                    msg_pos_to_reach.data = [self.x_actual, self.y_actual] 
                    self.pub_pos_to_reach.publish(msg_pos_to_reach)
                    sleep(3)
            else:
                msg_pos_to_reach.data = [self.x_ennemy, self.y_ennemy] 
                self.pub_pos_to_reach.publish(msg_pos_to_reach)
            print('State unknow')


def main(args=None):
    rclpy.init(args=args)
    node = MyHub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()

