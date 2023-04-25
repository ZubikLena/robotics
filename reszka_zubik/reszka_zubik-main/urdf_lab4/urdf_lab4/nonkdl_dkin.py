#! /usr/bin/env python
from math import sin, cos, atan2, sqrt
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import numpy
import json
import os


def read_dh(jsonfile):
    ''' 
    method reading json file
    returns dict
    '''
    with open(jsonfile, 'r') as file:
        data = json.load(file)    
    return data

class NonKdlDkin(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('nonkdl_dkin')
       
        # load DH parameters
        data = read_dh(os.path.dirname(os.path.realpath(__file__)) + '/dh.json')

        self.pose_stamped = PoseStamped()
        self.arm1connect = 0.0
        self.arm2connect = 0.0
        self.wristconnect = 0.0

        self.base = (data['base']['length'] - (data['wrist']['size']['z']/2) - (data['gripper']['size']['z']/2))
        self.d1 = data['arm1']['d']
        self.a2 = data['arm1']['size']['z']
        self.a3 = data['arm2']['size']['z']
        self.d3 = data['wrist']['d']
        self.alfa3 = data['wrist']['alfa']


        qos_profile = QoSProfile(depth=10)
        self.pose_pub = self.create_publisher(PoseStamped, 'pose_stamped_nonkdl', qos_profile)
        self.joint_state = self.create_subscription(JointState, 'joint_states', self.listener_callback, 10)

    

        #self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        # rate
        self.rate = self.create_rate(10)

    def listener_callback(self, msg):
        # get joint state
        [self.arm1connect, self.arm2connect, self.wristconnect] = msg.position

        # publish pose of chwytak
        now = self.get_clock().now()
        self.calculate_pose_stamp()
        self.pose_stamped.header.stamp = now.to_msg()
        self.pose_stamped.header.frame_id = 'base'
        self.pose_pub.publish(self.pose_stamped)
        #self.rate.sleep()

    def calculate_pose_stamp(self):
        
        T01 = numpy.array([[cos(self.arm1connect), -sin(self.arm1connect), 0.0, 0.0],
                           [sin(self.arm1connect), cos(self.arm1connect), 0.0, 0.0],
                           [0.0, 0.0, 1.0, self.d1],
                           [0.0, 0.0, 0.0, 1.0]])
        #self.a2 =1
        T12 = numpy.array([[cos(self.arm2connect), -sin(self.arm2connect), 0.0, self.a2],
                           [sin(self.arm2connect), cos(self.arm2connect), 0.0, 0.0],
                           [0.0, 0.0, 1.0, 0.0],
                           [0.0, 0.0, 0.0, 1.0]])

        T23 = numpy.array([[1.0, 0.0, 0.0, self.a3],
                           [0.0, cos(self.alfa3), -sin(self.alfa3), 0.0],
                           [0.0, sin(self.alfa3), cos(self.alfa3), cos(self.alfa3) * (self.d3 + self.wristconnect)],
                           [0.0, 0.0, 0.0, 1.0]])

        # T23 = numpy.array([[1.0, 0.0, 0.0, self.a3],
        #                    [0.0, -1.0, 0.0, 0.0],
        #                    [0.0, 0.0, -1.0, cos(self.alfa3) * (self.d3 + self.wristconnect)],
        #                    [0.0, 0.0, 0.0, 1.0]])

        # T23 = numpy.array([[1.0, 0.0, 0.0, self.a3],
        #                    [0.0, cos(self.alfa3), -sin(self.alfa3), 0.0],
        #                    [0.0, sin(self.alfa3), cos(self.alfa3), cos(self.alfa3) * (self.d3 + self.poz3)],
        #                    [0.0, 0.0, 0.0, 1.0]])

        T03 = numpy.matmul(T01, numpy.matmul(T12, T23))
        P3 = numpy.array([[0.0], [0.0], [0.0], [1.0]])  # poczatek ukladu wspolrzednych
        P0 = numpy.matmul(T03, P3)

        #print(P0[0], P0[1], P0[2], euler_to_quaternion(*rot_to_euler(T03)))
        self.pose_stamped.pose.position.x = float(P0[0])
        self.pose_stamped.pose.position.y = float(P0[1])
        #taking the height of base into consideration
        self.pose_stamped.pose.position.z = float(P0[2])+self.base
        self.pose_stamped.pose.orientation = euler_to_quaternion(*rot_to_euler(T03))


def rot_to_euler(t):
    yaw = atan2(t[1][0], t[0][0])
    pitch = atan2(-1 * t[2][0], sqrt(t[2][1] ** 2 + t[2][2] ** 2))
    roll = atan2(t[2][1], t[2][2])
    return roll, pitch, yaw


def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
    qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2)
    qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2)
    qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main():
    node = NonKdlDkin()
    rclpy.spin(node)


if __name__ == '__main__':
    main()#! /usr/bin/env python
from math import sin, cos, atan2, sqrt
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import numpy
import json
import os


def read_dh(jsonfile):
    ''' 
    method reading json file
    returns dict
    '''
    with open(jsonfile, 'r') as file:
        data = json.load(file)    
    return data

class NonKdlDkin(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('nonkdl_dkin')
       
        # load DH parameters
        data = read_dh(os.path.dirname(os.path.realpath(__file__)) + '/dh.json')

        self.pose_stamped = PoseStamped()
        self.arm1connect = 0.0
        self.arm2connect = 0.0
        self.wristconnect = 0.0

        self.base = (data['base']['length'] - (data['wrist']['size']['z']/2) - (data['gripper']['size']['z']/2))
        self.d1 = data['arm1']['d']
        self.a2 = data['arm1']['size']['z']
        self.a3 = data['arm2']['size']['z']
        self.d3 = data['wrist']['d']
        self.alfa3 = data['wrist']['alfa']


        qos_profile = QoSProfile(depth=10)
        self.pose_pub = self.create_publisher(PoseStamped, 'pose_stamped_nonkdl', qos_profile)
        self.joint_state = self.create_subscription(JointState, 'joint_states', self.listener_callback, 10)

    

        #self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        # rate
        self.rate = self.create_rate(10)

    def listener_callback(self, msg):
        # get joint state
        [self.arm1connect, self.arm2connect, self.wristconnect] = msg.position

        # publish pose of chwytak
        now = self.get_clock().now()
        self.calculate_pose_stamp()
        self.pose_stamped.header.stamp = now.to_msg()
        self.pose_stamped.header.frame_id = 'base'
        self.pose_pub.publish(self.pose_stamped)
        #self.rate.sleep()

    def calculate_pose_stamp(self):
        
        T01 = numpy.array([[cos(self.arm1connect), -sin(self.arm1connect), 0.0, 0.0],
                           [sin(self.arm1connect), cos(self.arm1connect), 0.0, 0.0],
                           [0.0, 0.0, 1.0, self.d1],
                           [0.0, 0.0, 0.0, 1.0]])
        #self.a2 =1
        T12 = numpy.array([[cos(self.arm2connect), -sin(self.arm2connect), 0.0, self.a2],
                           [sin(self.arm2connect), cos(self.arm2connect), 0.0, 0.0],
                           [0.0, 0.0, 1.0, 0.0],
                           [0.0, 0.0, 0.0, 1.0]])

        T23 = numpy.array([[1.0, 0.0, 0.0, self.a3],
                           [0.0, cos(self.alfa3), -sin(self.alfa3), 0.0],
                           [0.0, sin(self.alfa3), cos(self.alfa3), cos(self.alfa3) * (self.d3 + self.wristconnect)],
                           [0.0, 0.0, 0.0, 1.0]])

        T03 = numpy.matmul(T01, numpy.matmul(T12, T23))
        P3 = numpy.array([[0.0], [0.0], [0.0], [1.0]])  # poczatek ukladu wspolrzednych
        P0 = numpy.matmul(T03, P3)

        self.pose_stamped.pose.position.x = float(P0[0])
        self.pose_stamped.pose.position.y = float(P0[1])

        #taking the height of base into consideration
        self.pose_stamped.pose.position.z = float(P0[2])+self.base
        self.pose_stamped.pose.orientation = euler_to_quaternion(*rot_to_euler(T03))


def rot_to_euler(t):
    yaw = atan2(t[1][0], t[0][0])
    pitch = atan2(-1 * t[2][0], sqrt(t[2][1] ** 2 + t[2][2] ** 2))
    roll = atan2(t[2][1], t[2][2])
    return roll, pitch, yaw


def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
    qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2)
    qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2)
    qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main():
    node = NonKdlDkin()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
