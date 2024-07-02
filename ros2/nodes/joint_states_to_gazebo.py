import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
    
import math

class JointStateSubscriber(Node):

    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'joint_statesW',
            self.listener_callback,
            10)
        
        self.joint_publishers = {
            'joint1': self.create_publisher(Float64, 'joint1W', 10), # there is a ROS2 bridge from joint1W to model/.../0/cmd_pos
            'joint2': self.create_publisher(Float64, 'joint2W', 10), 
            'joint3': self.create_publisher(Float64, 'joint3W', 10),
            'joint4': self.create_publisher(Float64, 'joint4W', 10),
            'joint5': self.create_publisher(Float64, 'joint5W', 10),
            'joint6': self.create_publisher(Float64, 'joint6W', 10)
        }

    def listener_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name in self.joint_publishers:
                float_msg = Float64()
                #conversion from degree to radiant
                float_msg.data = round(math.radians(position), 2)
                # print(name, " -> ", float_msg.data)

                self.joint_publishers[name].publish(float_msg)
                # self.get_logger().info('Published {}: "{}"'.format(name, float_msg.data))

def main(args=None):
    rclpy.init(args=args)
    node = JointStateSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
