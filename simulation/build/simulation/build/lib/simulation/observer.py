import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('block_publisehr')
        self.publisher_ = self.create_publisher(PoseStamped, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.client = RemoteAPIClient('localhost',23000)
        self.sim = self.client.getObject('sim')
        self.block_handles=[]
    def connect(self):
        self.sim.startSimulation()
        self.get_logger().info('Observer connected to simulation')
    def get_handles(self):
        self.block_handles.append(self.sim.getObject('/A58_1_1'))
        self.get_logger().info('object aquired')

    def timer_callback(self):
        object_handle = self.block_handles[0]
        [x,y,z,qx,qy,qz,qw]=self.sim.getObjectPose(object_handle)
        b_pose = PoseStamped()
        b_pose.pose.position.x=x
        b_pose.pose.position.y=y
        b_pose.pose.position.z=z
        b_pose.pose.orientation.x =qx
        b_pose.pose.orientation.y = qy
        b_pose.pose.orientation.z = qz
        b_pose.pose.orientation.w = qw
        self.publisher_.publish(b_pose)
        self.get_logger().info('Publishing block pose')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    minimal_publisher.connect()
    minimal_publisher.get_handles()
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()