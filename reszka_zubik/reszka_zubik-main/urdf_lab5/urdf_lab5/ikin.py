from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

class Ikin(Node):

  def __init__(self):

      rclpy.init()
      super().__init__('ikin')

      qos_profile = QoSProfile(depth=10)
      self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
      self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
      self.nodeName = self.get_name()
      self.get_logger().info("{0} started".format(self.nodeName))

      loop_rate = self.create_rate(30)

      joint_state = JointState()

def main():
  node = Ikin()

if __name__ == '__main__':
  main()
