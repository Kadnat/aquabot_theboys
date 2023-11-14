from std_msgs.msg import Float64
import rclpy
from rclpy.node import Node
from scipy.signal import kaiserord, firwin, lfilter

MAX_NUMBER_OF_ELEMENTS = 100

# Fréquence d'échantillonnage
Fe = 10  # Hz

# Fréquence de Nyquist
F_nyq = Fe / 2.  # Hz

# Largeur de transition entre passe et coupe, en fonction de la freq de Nyquist
width = 5.0/F_nyq

# Niveau d'atténuation des fréquences coupées
ripple_db = 60.0

# Fréquence de coupure
Fc = 0.16  # < F_nyq en Hz

class FiltersNode(Node):

    def __init__(self):
        super().__init__('filters')
        self.pub_data_filtered = self.create_publisher(Float64, '/data/filtered', 10)
        self.sub_data_to_filter = self.create_subscription(
            Float64, '/data/to_filter', self.callback_data_to_filter, 10)
        self.sub_data_to_filter  # prevent unused variable warning
        self.data_array = []
        self.data_filtered = []
        self.data_counter = 0


    def callback_data_to_filter(self, msg):
        self.get_logger().info(f'ANT1: {msg.data}')
        if self.data_counter > MAX_NUMBER_OF_ELEMENTS:  
            self.data_array.pop(0) # si >100, on enleve le premier element

        else:                         # sinon on augmente le compteur
            self.data_counter += 1
        
        self.data_array.append(msg.data)

        # Déterminer la fenêtre de Kaiser
        N, beta = kaiserord(ripple_db, width)

        if self.data_counter > N:   # Si le nombre de valeurs est supérieur à l'ordre du filtre
            # Déterminer le vecteur de coefficients du filtre
            coefs = firwin(N, Fc/F_nyq, window=('kaiser', beta))

            # Appliquer le filtre
            self.data_filtered = lfilter(coefs, 1, self.data_array)

            filtered = Float64()
            filtered.data = self.data_filtered[-1]
            
            self.pub_data_filtered.publish(filtered)
        else:
            self.pub_data_filtered.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)

    filters_node = FiltersNode()
    rclpy.spin(filters_node)
    filters_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()