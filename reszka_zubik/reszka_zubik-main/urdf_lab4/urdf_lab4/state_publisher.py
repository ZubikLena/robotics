from math import sin, cos, pi
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
class StatePublisher(Node):
  '''
    Class representing state_publisher node

    Methods:
    -------
    __init__: constructor method, JointState callback function
    euler_to_quaternion: euler to quaternion converter
  '''
  def __init__(self):
      '''
      Class constructor
      JointState callback function

      Parameters:
      -------
      joint_pub: publisher, JointState
        publishes joint states
      wristconnect: float
        wrist joint position
      arm1connect: float
        arm1 joint position
      arm2connect: float
        arm2 joint positions
      delta: float
        value added to change state
      angle1: float
        angle added to arm1
      angle2: float
        angle added to arm2
      '''
      rclpy.init()
      super().__init__('state_publisher')

      qos_profile = QoSProfile(depth=10)
      self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
      self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
      self.nodeName = self.get_name()
      self.get_logger().info("{0} started".format(self.nodeName))

      loop_rate = self.create_rate(30)

      # robot state
      wristconnect = 0.1
      arm1connect = 0.
      arm2connect = 0.


      # message declarations
      odom_trans = TransformStamped()
      odom_trans.header.frame_id = 'odom'
      odom_trans.child_frame_id = 'base'

      joint_state = JointState()

      try:
          while rclpy.ok():
              rclpy.spin_once(self)

              # update joint_state
              now = self.get_clock().now()
              joint_state.header.stamp = now.to_msg()
              joint_state.name = ['arm1connect', 'arm2connect', 'wristconnect']
              joint_state.position = [arm1connect, arm2connect, wristconnect]


              # update transform
              odom_trans.header.stamp = now.to_msg()
              odom_trans.transform.translation.x = 0.
              odom_trans.transform.translation.y = 0.0
              odom_trans.transform.translation.z = 0.0
              odom_trans.transform.rotation = \
                  euler_to_quaternion(0, 0, 0)

              # send the joint state and transform
              self.joint_pub.publish(joint_state)
              self.broadcaster.sendTransform(odom_trans)


              loop_rate.sleep()

      except KeyboardInterrupt:
          pass

def euler_to_quaternion(roll, pitch, yaw):
  qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
  qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
  qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
  qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
  return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
  node = StatePublisher()

if __name__ == '__main__':
  main()
