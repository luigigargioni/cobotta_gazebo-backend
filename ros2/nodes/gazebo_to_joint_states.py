import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class JointStatePublisher(Node):

    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_names = ['joint1R', 'joint2R', 'joint3R', 'joint4R', 'joint5R', 'joint6R']
        self.positions = [0.0] * len(self.joint_names)
        self.subscribers = []

        # there is a ROS2 bridge from model/.../0/cmd_pos to joint1R
        for i, joint_name in enumerate(self.joint_names):
            self.subscribers.append(self.create_subscription(
                Float64,
                joint_name,
                lambda msg, i=i: self.update_position(msg, i),
                10))

        #self.timer = self.create_timer(0.1, self.publish_joint_states)

    def update_position(self, msg, index):
        old_pos = self.positions[index]
        self.positions[index] = msg.data
        if old_pos != msg.data:
            self.publish_joint_states() # al posto di usare il timer

    def publish_joint_states(self):
        msg = JointState()
        msg.name = self.joint_names
        msg.position = self.positions
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing joint states: {}'.format(self.positions))


def main(args=None):
    rclpy.init(args=args)

    joint_state_publisher = JointStatePublisher()

    rclpy.spin(joint_state_publisher)

    joint_state_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
