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
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer_ = self.create_timer(timer_period, self.publish_message)
        
        #deklaracja defaultowych parametrów
        self.declare_parameter('my_parameter_up', 'g')
        self.declare_parameter('my_parameter_down', 'd')
        self.declare_parameter('my_parameter_right', 'p')
        self.declare_parameter('my_parameter_left', 'l')
        self.declare_parameter('my_parameter_stop', 's')

        #pomocnicze parametry, aby wyświetlać instrukcje tylko gdy oryginalne ulegną zmianie
        self.last_my_param_up = 'g'
        self.last_my_param_down = 'd'
        self.last_my_param_right = 'p'
        self.last_my_param_left = 'l'
        self.last_my_param_stop = 's'

        #wyświetlanie instrukcji dla używtkownika
        print ("Motion: " + self.last_my_param_up + "- UP; " + self.last_my_param_down + "- DOWN; " + self.last_my_param_right + "- RIGHT; " + self.last_my_param_left + "- LEFT; " + self.last_my_param_stop + "- STOP")

    #funkcja sprawdzająca poprawność wprowadzonego sterowania z dostępnymi klawiszami   
    def correct_key(self, user_key):
        for key in self.key_mapping:
            if(user_key==key):
                return True
        return False

    def publish_message(self):
        message = Twist()

        #ponieranie parametrów, wartości aktulanych ustalonych przycisków do sterowania żółwiem
        my_param_up = self.get_parameter('my_parameter_up').get_parameter_value().string_value
        my_param_down = self.get_parameter('my_parameter_down').get_parameter_value().string_value
        my_param_right = self.get_parameter('my_parameter_right').get_parameter_value().string_value
        my_param_left = self.get_parameter('my_parameter_left').get_parameter_value().string_value
        my_param_stop = self.get_parameter('my_parameter_stop').get_parameter_value().string_value

        #jeżeli parametry uległy zmianie, zostaje wyświetlona instrukcja dla użytkownika
        if (self.last_my_param_up != my_param_up or self.last_my_param_down!= my_param_down or self.last_my_param_right != my_param_right or self.last_my_param_left != my_param_left or self.last_my_param_stop != my_param_stop):
            print ("Motion: " + my_param_up + "- UP; " + my_param_down + "- DOWN; " + my_param_right + "- RIGHT; " + my_param_left + "- LEFT; " + my_param_stop + "- STOP")

        #remapping parametrów na odpowiednie prędkości liniowe i kątowe
        self.key_mapping = {
            my_param_up: [0, 1],  #g
            my_param_down: [0, -1], #d
            my_param_right: [-1, 0], #p
            my_param_left: [1, 0],  #l
            my_param_stop: [0,0]    #s
        }
        
        #sterowanie wprowadzane przez użytkownika
        user_input = getch.getch()
        if (len(user_input) == 0 or not self.correct_key(user_input)):
            return #unknown key
        
        #ustawianie odpwoiednich wartości prędkości zgodnych z remappingiem
        vels = self.key_mapping[user_input[0]]
        message.angular.z = float(vels[0])
        message.linear.x = float(vels[1])
        
        #wysyłanie wiadomości o sterowaniu do turtlesim
        self.publisher_.publish(message)

        #aktualizacja parametrów odpowiadających za wyświelanie się instrukcji przy zmianie sterowań
        self.last_my_param_up = my_param_up
        self.last_my_param_down = my_param_down
        self.last_my_param_right = my_param_right
        self.last_my_param_left = my_param_left
        self.last_my_param_stop = my_param_stop

        

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
