import sys

from interpolation.srv import OintControl
import rclpy
from rclpy.node import Node


class OintClient(Node):

    def __init__(self):
        super().__init__('oint_client')
        self.cli = self.create_client(OintControl, 'oint_control')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = OintControl.Request()

    def send_request(self):
        try:
            if(float(sys.argv[7])<=0):
                raise ValueError("Invalid time value")

            if(float(sys.argv[8]) > float(sys.argv[7]) or float(sys.argv[8]) < 0):
                raise ValueError("Incorrect sample time")

            if(str(sys.argv[9]) !='linear' and str(sys.argv[9]) !='polynomial'):
                raise ValueError("Invalid type")

            self.req.arm1_pos = float(sys.argv[1])
            self.req.arm2_pos = float(sys.argv[2])
            self.req.wrist_pos = float(sys.argv[3])

            self.req.roll = float(sys.argv[4])
            self.req.pitch = float(sys.argv[5])
            self.req.yaw = float(sys.argv[6])

            self.req.time = float(sys.argv[7])
            self.req.sample_time = float(sys.argv[8])
            self.req.type = str(sys.argv[9])

        except IndexError("Incorrect number of arguments"):
            pass

        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    oint_client = OintClient()
    oint_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(oint_client)
        if oint_client.future.done():
            try:
                response = oint_client.future.result()
            except Exception as e:
                oint_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                oint_client.get_logger().info(
                    'Finished interpolation: arm1:%d, arm2:%d, wrist:%d roll: %d pitch: %d yaw: %d' %
                    (float(oint_client.req.arm1_pos), float(oint_client.req.arm2_pos), float(oint_client.req.wrist_pos),
                    oint_client.req.roll, oint_client.req.pitch, oint_client.req.yaw)
                )
            break

    oint_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()