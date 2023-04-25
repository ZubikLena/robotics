import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('py_pub_spiral_node')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 1)
        timer_period = 0.5  # seconds
        self.i = 0.0
        self.timer_ = self.create_timer(timer_period, self.publish_message)
        #self.get_logger().info(sys.argv[1]) #->s
        self.key_mapping = {
            sys.argv[1]: [0, 1],  #w
            sys.argv[2]: [0, -1], #x
            sys.argv[3]: [-1, 0], #a
            sys.argv[4]: [1, 0],  #d
            sys.argv[5]: [0, 0]   #s
        }
        print(self.key_mapping)

    def correct_key(self, user_key):
        for key in self.key_mapping:
            if(user_key==key):
                return True
        return False


    def publish_message(self):
        message = Twist()
        user_input = input()
        if (len(user_input) == 0 or not self.correct_key(user_input)):
            return #unknown key
        
        vels = self.key_mapping[user_input[0]]
        message.angular.z = float(vels[0])
        message.linear.x = float(vels[1])
        self.publisher_.publish(message)
        



def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
