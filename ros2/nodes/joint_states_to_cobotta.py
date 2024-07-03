import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import math

from cobotta_utils import *

class CobottaController(Node):

    def __init__(self, client, hRobot):
        super().__init__('cobotta_controller')

        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
    
        self.client = client
        self.hRobot = hRobot

        # if active then jointState is send to Cobotta
        self.active = False

        self.subscription = self.create_subscription(
            Bool,
            'activeStatusCobotta',
            self.active_status_listener_callback,
            10)

    def active_status_listener_callback(self, msg):
        self.active = not msg.data
        
    def listener_callback(self, msg):
        if self.active:
            self.move_robot(msg.position)

    def move_robot(self, positions):       
        # make conversion from radiant to degree
        positions_degree_format = [math.degrees(pos) for pos in positions]
        
        try:
            move_to_angle(self.client, self.hRobot, positions_degree_format)
        except Exception as e:
            self.get_logger().error("ERROR OCCURRED WHILE TRYING TO MOVE COBOTTA FROM GAZEBO: %s" % str(e))


def main(args=None):
    rclpy.init(args=args)

    # connect to Cobotta
    client, _, hRobot = connect("192.168.0.1", 5007, 5000)

    physical_robot_controller = CobottaController(client, hRobot)

    rclpy.spin(physical_robot_controller)

    physical_robot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
