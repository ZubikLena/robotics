import rclpy
from rclpy.node import Node
from interpolation.srv import OintControl
from geometry_msgs.msg import PoseStamped
import transforms3d
import time
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from math import pi


class OintControlService(Node):

    def __init__(self):
        super().__init__('oint_service')

        qos_profile = QoSProfile(depth=10)

        self.service = self.create_service(OintControl, 'oint_control', self.oint_control_callback)
        self.pose_pub = self.create_publisher(PoseStamped, 'pose_oint_control', qos_profile)
        self.pose_rpy_pub = self.create_publisher(PoseStamped, 'rpy_oint_control', qos_profile)
        self.time = 0
        # self.f_x = open("x_data", 'a')
        # self.f_y = open("y_data", 'a')
        # self.f_z = open("z_data", 'a')

        # self.f_r = open("r_data", 'a')
        # self.f_p = open("p_data", 'a')
        # self.f_ya = open("ya_data", 'a')


    def oint_control_callback(self, msg, response):

        self.f_x = open("x_data", 'a')
        self.f_y = open("y_data", 'a')
        self.f_z = open("z_data", 'a')

        self.f_r = open("r_data", 'a')
        self.f_p = open("p_data", 'a')
        self.f_ya = open("ya_data", 'a')

        sample_time = msg.sample_time

        while(self.time < msg.time):

            pose = PoseStamped()
            now = self.get_clock().now()
            pose.header.stamp = now.to_msg()
            pose.header.frame_id = "map"

            rpy = PoseStamped()
            now = self.get_clock().now()
            rpy.header.stamp = now.to_msg()
            rpy.header.frame_id = "map"

            
            pose.pose.position.x = self.interpolate(msg.time, msg.arm1_pos)
            pose.pose.position.y = self.interpolate(msg.time, msg.arm2_pos)
            pose.pose.position.z = self.interpolate(msg.time, msg.wrist_pos)

            self.f_x.write(str(pose.pose.position.x) + "\n")
            self.f_y.write(str(pose.pose.position.y) + "\n")
            self.f_z.write(str(pose.pose.position.z) + "\n")


            roll = self.interpolate(msg.time, msg.roll)
            pitch = self.interpolate(msg.time, msg.pitch)
            yaw = self.interpolate(msg.time, msg.yaw)

            self.f_r.write(str(roll) + "\n")
            self.f_p.write(str(pitch) + "\n")
            self.f_ya.write(str(yaw) + "\n")


            rpy.pose.position.x = pose.pose.position.x
            rpy.pose.position.y = pose.pose.position.y
            rpy.pose.position.z = pose.pose.position.z

            rpy.pose.orientation = Quaternion(w= 0.0, x=roll, y=pitch, z=yaw)

            quat = transforms3d.euler.euler2quat(roll*pi/180, pitch*pi/180, yaw*pi/180, axes='sxyz')
            pose.pose.orientation = Quaternion(w=quat[0], x=quat[1], y=quat[2], z=quat[3])

            self.pose_rpy_pub.publish(rpy)
            self.pose_pub.publish(pose)

            self.time += sample_time

            time.sleep(sample_time)


        response.output = 'Interpolation completed'

        self.f_p.close()
        self.f_r.close()
        self.f_ya.close()
        self.f_x.close()
        self.f_y.close()
        self.f_z.close()
        return response
        

    def interpolate(self,time, end_point): # TODO zmienic zeby dzialalo dla roznych pkt pocz.
        # (x0,t0) = (0,0)

        x1 = end_point
        t1 = time

        return (x1/t1)*self.time


def main(args=None):

    rclpy.init(args=args)

    oint_control_service = OintControlService()

    rclpy.spin(oint_control_service)

    rclpy.shutdown()

if __name__ == '__main__':

    main()