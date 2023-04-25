import sys
import rclpy
import getch
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('py_pub_spiral_node')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 1)
        timer_period = 0.5  # seconds
        self.timer_ = self.create_timer(timer_period, self.publish_message)
        
        self.declare_parameter('my_parameter', 'gdpl')
        
    def correct_key(self, user_key):
        for key in self.key_mapping:
            if(user_key==key):
                return True
        return False

    def publish_message(self):
        message = Twist()

        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value

        self.key_mapping = {
            my_param[0]: [0, 1],  #g
            my_param[1]: [0, -1], #d
            my_param[2]: [-1, 0], #p
            my_param[3]: [1, 0],  #l
        }
        print(self.key_mapping)
        """
        my_new_param = rclpy.parameter.Parameter(
            'my_parameter',
            rclpy.Parameter.Type.STRING,
            'gdpl'
        )
        all_new_parameters = [my_new_param]
        self.set_parameters(all_new_parameters)
        """

        user_input = getch.getch()
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
