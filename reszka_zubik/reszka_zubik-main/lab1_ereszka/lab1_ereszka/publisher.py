import rclpy
from rclpy.node import Node
import curses
import sys
import os
from time import sleep

from geometry_msgs.msg import Twist


class MinimalPublisher(Node):
    '''
    Class representing publisher node

    Methods:
    -------
    timer_callback(): publisher's callback method

    '''


    def __init__(self):
        '''
        Class constructor

        Parameters:
        -------
        publisher: publisher, Twist
            publishes turtle velocity
        timer: timer
            callback funciton's timer
        '''
        super().__init__('publisher')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # declaring default parameters
        self.declare_parameter('left', 'a')
        self.declare_parameter('right', 'l')
        self.declare_parameter('up', 't')
        self.declare_parameter('down', 'v')


    def timer_callback(self):
        '''
        Publisher's callback method, reads keyboard keys
        Controls turtle's velocity based on pressed keys

        Parameters:
        ----------
        left: char
            left key param
        right: char
            right key param
        up: char
            up key param
        down: char
            down key param
        stdscr: curses app
        msg: Twist
            published turtle velocity
        key: string
            pressed key string    
        '''

        # getting key input parameters
        self.left = self.get_parameter('left').get_parameter_value().string_value
        self.right = self.get_parameter('right').get_parameter_value().string_value
        self.up = self.get_parameter('up').get_parameter_value().string_value
        self.down = self.get_parameter('down').get_parameter_value().string_value

        sleep(0.2) # to stop curses from blocking param getter

        stdscr = curses.initscr() # initialising curses
        curses.noecho()
        curses.cbreak()

        msg = Twist()

        key = stdscr.getkey()
        
        # checking pressed key name
        if(self.left == key):
            msg.angular.z = 1.0
            self.get_logger().info('Going left')
        elif(self.right == key):
            msg.angular.z = -1.0
            self.get_logger().info('Going right')
        elif(self.up == key):
            msg.linear.x = 1.0
            self.get_logger().info('Going up')
        elif(self.down == key):
            msg.linear.x = -1.0
            self.get_logger().info('Going down')
        else:
            print("Waiting for pressed key")

        self.publisher_.publish(msg)


        # ending curses
        curses.nocbreak()
        stdscr.keypad(0)
        curses.echo()
        curses.endwin()


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
