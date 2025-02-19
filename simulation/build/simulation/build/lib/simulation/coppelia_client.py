import rclpy
import rclpy.logging
from rclpy.node import Node

from std_msgs.msg import String

#coppeliasim imports

from coppeliasim_zmqremoteapi_client import RemoteAPIClient



class RoboticEnvironment(Node):
    def __init__(self):

        #initalise ros stuff

        #coppelia configs
        self.client = RemoteAPIClient('localhost',23000)
        self.sim = self.client.getObject('sim')
        self.simIK = self.client.require('simIK')
        self.joint_handles=[]
        self.ee_handle =None
        self.ik_env =None
        self.ik_group=None

    def connect(self):
        self.sim.startSimulation()
        self.get_logger().info('Connected to simulation')

    def stop_simulation(self):
        self.sim.stopSimulation()
        self.get_logger().info('simulation stopped')

    def initialize_handles(self):
        joints = ['/UR10/joint'+str(i) for i in range(1,7)]
        self.joint_handles = [self.sim.getObject(joint) for joint in joints]
        print('Joint handles retrieved')
        self.ee_handle = self.sim.getObject('/UR10/tip')
        self.get_logger.info('End effector handle retrieved')

    def setup_ik(self):
        self.ik_env = self.simIK.createEnvironment()
        self.ik_group = self.simIK.createGroup(self.ik_env)
        self.simIK.setGroupCalculation(self.ik_env,self.ik_group,self.simIK.method_damped_least_squares,0.3,99)

def main():
    env = RoboticEnvironment()
    env.connect()
    env.initialize_handles()
    #keep the sim alive
    # rclpy.spin(env)

    #stop simulation and destroy node
    env.stop_simulation()
    # env.destroy_node()
    # rclpy.shutdown()

 
if __name__ == '__main__':
    main()



