#! /usr/bin/env python
from math import sin, cos, atan2, sqrt
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
# https://ubuntu.pkgs.org/20.04/ubuntu-universe-arm64/python3-pykdl_1.4.0-7build2_arm64.deb.html
from PyKDL import *
import yaml
import os


class KdlDkin(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('kdl_dkin')
       
        self.pose_stamped = PoseStamped()
        self.arm1connect = 0.0
        self.arm2connect = 0.0
        self.wristconnect = 0.0


        qos_profile = QoSProfile(depth=10)
        self.pose_pub = self.create_publisher(PoseStamped, 'pose_stamped_kdl', qos_profile)
        self.joint_state = self.create_subscription(JointState, 'joint_states', self.listener_callback, 10)

        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        self.rate = self.create_rate(10)

    def listener_callback(self, msg):
        # get joint state
        [self.arm1connect, self.arm2connect, self.wristconnect] = msg.position

        print(self.pose_stamped.pose.position)
        print(self.pose_stamped.pose.orientation)

        now = self.get_clock().now()

        self.get_pose_stamped()

        self.pose_stamped.header.stamp = now.to_msg()
        self.pose_stamped.header.frame_id = 'base'
        self.pose_pub.publish(self.pose_stamped)
        # self.rate.sleep()

    def get_pose_stamped(self):
        chain = Chain()

        rpy = self.read_yaml(os.path.dirname(os.path.realpath(__file__)) + '/dh.yaml')

        r = rpy['base']['rpy']['r']
        p = rpy['base']['rpy']['p']
        yaw = rpy['base']['rpy']['y']
        x = 2*rpy['base']['xyz']['x']
        y = 2*rpy['base']['xyz']['y']
        z = 2*rpy['base']['xyz']['z']

        base_frame = Frame(Rotation.RPY(r,p,yaw), Vector(x,y,z))

        r = rpy['arm1']['rpy']['r']
        p = rpy['arm1']['rpy']['p']
        yaw = rpy['arm1']['rpy']['y']
        x = 2*rpy['arm1']['xyz']['x']
        y = 2*rpy['arm1']['xyz']['y']
        z = 2*rpy['arm1']['xyz']['z'] # + rpy['base']['length']

        arm1_frame = Frame(Rotation.RPY(r,p,yaw), Vector(x,y,z))
        
        r = rpy['arm2']['rpy']['r']
        p = rpy['arm2']['rpy']['p']
        yaw = rpy['arm2']['rpy']['y']
        x = 2*rpy['arm2']['xyz']['x']
        y = 2*rpy['arm2']['xyz']['y']
        z = 2*rpy['arm2']['xyz']['z']

        arm2_frame = Frame(Rotation.RPY(r,p,yaw), Vector(x,y,z))

        r = rpy['wrist']['rpy']['r']
        p = rpy['wrist']['rpy']['p']
        yaw = rpy['wrist']['rpy']['y']
        x = rpy['wrist']['xyz']['x']
        y = rpy['wrist']['xyz']['y']
        z = rpy['wrist']['xyz']['z']

        wrist_frame = Frame(Rotation.RPY(r,p,yaw), Vector(x,y,z))

        # r = rpy['gripper']['rpy']['r']
        # p = rpy['gripper']['rpy']['p']
        # yaw = rpy['gripper']['rpy']['y']
        # x = rpy['gripper']['xyz']['x']
        # y = rpy['gripper']['xyz']['y']
        # z = rpy['gripper']['xyz']['z']

        # gripper_frame = Frame(Rotation.RPY(r,p,yaw), Vector(x,y,z))

        chain.addSegment(Segment(Joint(), base_frame))
        chain.addSegment(Segment(Joint(Joint.RotZ), arm1_frame))
        chain.addSegment(Segment(Joint(Joint.RotZ), arm2_frame))
        chain.addSegment(Segment(Joint(Joint.TransZ), wrist_frame))
        # chain.addSegment(Segment(Joint(), gripper_frame))

        result = ChainFkSolverPos_recursive(chain)
        gripper = Frame()

        joint_array = JntArray(3)
        joint_array[0] = self.arm1connect
        joint_array[1] = self.arm2connect
        joint_array[2] = -self.wristconnect

        result.JntToCart(joint_array, gripper)

        quaternion = gripper.M.GetQuaternion()
        position = gripper.p

        self.pose_stamped.pose.position.x = float(position[0])
        self.pose_stamped.pose.position.y = float(position[1])
        self.pose_stamped.pose.position.z = float(position[2])

        self.pose_stamped.pose.orientation.x = float(quaternion[0])
        self.pose_stamped.pose.orientation.y = float(quaternion[1])
        self.pose_stamped.pose.orientation.z = float(quaternion[2])
        self.pose_stamped.pose.orientation.w = float(quaternion[3])



    ''' dividing kdl solving into functions causes issues'''
    def set_pose_stamped(self):
        position, quaternion = self.solve_gripper_params()

        self.pose_stamped.pose.position.x = float(position[0])
        self.pose_stamped.pose.position.y = float(position[1])
        self.pose_stamped.pose.position.z = float(position[2])

        self.pose_stamped.pose.orientation.x = float(quaternion[0])
        self.pose_stamped.pose.orientation.y = float(quaternion[1])
        self.pose_stamped.pose.orientation.z = float(quaternion[2])
        self.pose_stamped.pose.orientation.w = float(quaternion[3])

    def solve_gripper_params(self):
        # solving forward kinematics
        result = self.solve_kdl(self.get_chain())

        gripper = Frame()
        result.JntToCart(self.get_joint_array(), gripper)
        quaternion = gripper.M.GetQuaternion()
        position = gripper.p

        return position, quaternion


    def solve_kdl(self, chain):
        return ChainFkSolverPos_recursive(chain)

    def get_joint_array(self):
        joint_array = JntArray(3)

        joint_array[0] = self.arm1connect
        joint_array[1] = self.arm2connect
        joint_array[2] = -self.wristconnect

        return joint_array

    def get_chain(self):
        chain = Chain()
        arm1 = Segment(Joint(Joint.RotZ), self.get_frame('arm1'))
        arm2 = Segment(Joint(Joint.RotZ), self.get_frame('arm2'))
        wrist = Segment(Joint(Joint.TransZ), self.get_frame('wrist'))

        chain.addSegment(arm1)
        chain.addSegment(arm2)
        chain.addSegment(wrist)
    
        return chain

    def get_frame(self, name):
        rpy = self.read_yaml(os.path.dirname(os.path.realpath(__file__)) + '/dh.yaml')

        r = rpy[name]['rpy']['r']
        p = rpy[name]['rpy']['p']
        y = rpy[name]['rpy']['y']
        x = rpy[name]['xyz']['x']
        y = rpy[name]['xyz']['y']
        z = rpy[name]['xyz']['z']

        return Frame(Rotation.RPY(r,p,y), Vector(x,y,z))

    def read_yaml(self, filename):
        with open(filename, 'r') as file:
            return yaml.load(file)


def main():
    node = KdlDkin()
    rclpy.spin(node)


if __name__ == '__main__':
    main()