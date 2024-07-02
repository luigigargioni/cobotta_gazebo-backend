import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import math

from cobotta_utils import *

class PhysicalRobotController(Node):
    # Receive new joints position from the sliders on the page that publish on "joint_statesWeb"
    def __init__(self, client, hRobot):
        super().__init__('physical_robot_controller')
        self.subscription = self.create_subscription(
            JointState,
            'joint_statesWeb',
            self.listener_callback,
            10)

        # used to send calibration command to Cobotta
        self.subscription = self.create_subscription(
            Bool,
            'calibrate_command',
            self.calibrate_callback,
            2)
    
        self.client = client
        self.hRobot = hRobot

    def calibrate_callback(self, msg):
        if msg.data:
            move_to_calibration_position(self.client, self.hRobot)

    def listener_callback(self, msg):
        self.get_logger().info('Physical robot controller Received joint states: {}'.format(msg.position))

        # Once a new position is received, we move the robot to this new position        
        self.move_robot(msg.position)

    def move_robot(self, positions):   
        try:
            move_to_angle(self.client, self.hRobot, positions)
        except Exception as e:
            self.get_logger().error("ERROR OCCURRED WHILE TRYING TO MOVE COBOTTA: %s" % str(e))

def main(args=None):
    rclpy.init(args=args)

    # connect to Cobotta
    client, _, hRobot = connect("192.168.0.1", 5007, 5000)
    # printf("physical robot controller connected to Cobotta.")
    # # move robot to calibration position
    # move_to_calibration_position(client, hRobot)

    physical_robot_controller = PhysicalRobotController(client, hRobot)

    rclpy.spin(physical_robot_controller)

    physical_robot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
