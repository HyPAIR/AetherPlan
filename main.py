from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import random
import subprocess

class RoboticEnvironment:
    def __init__(self):
        self.client = RemoteAPIClient('localhost',23000)
        self.sim = self.client.getObject('sim')
        self.simIK = self.client.require('simIK')
        self.joint_handles=[]
        self.ee_handle =None
        self.ik_env =None
        self.ik_group=None

    def connect(self):
        self.sim.startSimulation()
        print('Connected to simulation')

    def stop_simulation(self):
        self.sim.stopSimulation()
        print('simulation stopped')

    def initialize_handles(self):
        joints = ['/UR10/joint'+str(i) for i in range(1,7)]
        self.joint_handles = [self.sim.getObject(joint) for joint in joints]
        print('Joint handles retrieved')
        self.ee_handle = self.sim.getObject('/UR10/tip')
        print('End effector handle retrieved')

    def setup_ik(self):
        self.ik_env = self.simIK.createEnvironment()
        self.ik_group = self.simIK.createGroup(self.ik_env)
        self.simIK.setGroupCalculation(self.ik_env,self.ik_group,self.simIK.method_damped_least_squares,0.3,99)
def main():
    env = RoboticEnvironment()
    env.connect()
    env.initialize_handles()

if __name__=='__main__':
    main()