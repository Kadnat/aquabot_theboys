import rclpy
from rclpy.node import Node
from enum import Enum
from std_msgs.msg import Float64MultiArray

class State(Enum):
    REACH_ZONE = 0
    PATROL = 1
    FOLLOW = 2
    AVOID = 3


class MyHub(Node):
    def __init__(self):
        super().__init__('hub') 
        # position to reach
        self.pub_pos_to_reach = self.create_publisher(Float64MultiArray, '/position/to_reach', 10)
        timer_period = 0.1  # Période du timer en secondes
        # Appel la fonction pour la cmd moteurs toutes les 100ms
        self.create_timer(timer_period, self.hub_callback)
        self.create_subscription(Float64MultiArray, '/position/buoy', self.position_buoy_callback, 10)
        self.x_buoy = 0.0
        self.y_buoy = 0.0
        self.state_hub = State.REACH_ZONE

    def position_buoy_callback(self, msg):
        self.x_buoy, self.y_buoy = msg.data

    def hub_callback(self):
        msg_pos_to_reach = Float64MultiArray()
        if self.state_hub == State.REACH_ZONE:
            msg_pos_to_reach.data = [self.x_buoy, self.y_buoy] 
            self.pub_pos_to_reach.publish(msg_pos_to_reach)
        elif self.state_hub == State.PATROL:
            x=0
        elif self.state_hub == State.FOLLOW:
            x=0
        elif self.state_hub == State.AVOID:
            x=0
        else:
            print('State unknow')


def main(args=None):
    rclpy.init(args=args)
    node = MyHub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()
