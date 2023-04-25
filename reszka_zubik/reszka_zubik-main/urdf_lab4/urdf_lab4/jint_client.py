import sys

from interpolation.srv import JintControl
import rclpy
from rclpy.node import Node


class JintClient(Node):

    def __init__(self):
        super().__init__('jint_client')
        self.cli = self.create_client(JintControl, 'jint_control')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = JintControl.Request()

    def send_request(self):
        try:
            if float(sys.argv[2]) > 2.5 or float(sys.argv[2]) < -2.5:
                raise ValueError('Incorrect arm2 value for interpolation')
            if float(sys.argv[3]) > 0.2 or float(sys.argv[3]) < -0.5:
                # self.get_logger().info('Incorrect arm2 value for interpolation')
                raise ValueError('Incorrect wrist value for interpolation')
            if(float(sys.argv[4]) <= 0):
                raise ValueError("Incorrect time value for interpolation")
            if(float(sys.argv[5]) > float(sys.argv[4]) or float(sys.argv[5]) < 0):
                raise ValueError("Incorrect sample time value for interpolation")
            if(str(sys.argv[6]) !='linear' and sys.argv[6] != 'polynomial'):
                # self.get_logger().info('Incorrect interpolation type')
                raise ValueError('Incorrect interpolation type')
        except IndexError("Incorrect number of arguments"):
            pass
        except ValueError():
            pass


        self.req.arm1_pos = float(sys.argv[1])
        self.req.arm2_pos = float(sys.argv[2])
        self.req.wrist_pos = float(sys.argv[3])
        self.req.time = float(sys.argv[4])
        self.req.sample_time = float(sys.argv[5])
        self.req.type = str(sys.argv[6])

        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    jint_client = JintClient()
    jint_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(jint_client)
        if jint_client.future.done():
            try:
                response = jint_client.future.result()
            except Exception as e:
                jint_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                jint_client.get_logger().info(
                    'Finished interpolation: arm1:%d, arm2:%d, wrist:%d' %
                    (jint_client.req.arm1_pos, jint_client.req.arm2_pos, jint_client.req.wrist_pos)
                )
            break

    jint_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()