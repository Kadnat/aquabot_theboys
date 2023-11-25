import rclpy
from rclpy.node import Node
from enum import Enum
from std_msgs.msg import Float64MultiArray

# State machine enum
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

        # Calling State Machine every 100ms        
        timer_period = 0.1  
        self.create_timer(timer_period, self.hub_callback)
        # Getting Buoy position x,y
        self.create_subscription(Float64MultiArray, '/position/buoy', self.position_buoy_callback, 10)
        # Getting our Boat position x,y
        self.create_subscription(Float64MultiArray, '/position/current', self.current_pos_callback, 10)
        
        # x,y variable
        self.x_buoy = 0.0
        self.y_buoy = 0.0
        self.x_actual = 0.0
        self.y_actual = 0.0
        # Counter for getting the firsts values
        self.ctr_in_state = 0
        # State mahine variable
        self.state_hub = State.REACH_ZONE

    # Getting x,y position of our boat
    def current_pos_callback(self, msg):
        self.x_actual, self.y_actual = msg.data

    # Getting x,y position of the buoy
    def position_buoy_callback(self, msg):
        self.x_buoy, self.y_buoy = msg.data   
        #self.get_logger().info('Position boué : (%f;%f)' % (self.x_buoy, self.y_buoy)) 

    # To know if we are around 30 meters from the other object
    def are_near(self, x1,y1,x2,y2):
        if((x1<=x2+30) and (x1>=x2-30)) and ((y1<=y2+30) and (y1>=y2-30)):
            return True
        else:
            return False

    # State machine
    def hub_callback(self):
        # The x,y position to send to the motors
        msg_pos_to_reach = Float64MultiArray()
        # Reaching the buoy 
        if self.state_hub == State.REACH_ZONE:
            # To not count to much
            if self.ctr_in_state < 100:
                self.ctr_in_state += 1
            # If we are near to the buoy
            if(self.are_near(self.x_actual, self.y_actual, self.x_buoy, self.y_buoy)==True) and (self.ctr_in_state >= 100):
                self.state_hub = State.PATROL
                # We're sending a position that is next to us to stop the motors
                msg_pos_to_reach.data = [self.x_actual, self.y_actual] 
                self.pub_pos_to_reach.publish(msg_pos_to_reach)
            else :
                # If we are not near to the buoy, we send the buoy position
                msg_pos_to_reach.data = [self.x_buoy, self.y_buoy] 
                self.pub_pos_to_reach.publish(msg_pos_to_reach)
        # Patrol in the buoy zone
        elif self.state_hub == State.PATROL:
            self.get_logger().info('In the Zone')
        # Follow the red boat
        elif self.state_hub == State.FOLLOW:
            x=0
        # Avoid objects
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
