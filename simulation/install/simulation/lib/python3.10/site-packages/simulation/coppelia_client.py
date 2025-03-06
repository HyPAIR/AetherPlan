# import rclpy
# import rclpy.logging
# from rclpy.node import Node

# from std_msgs.msg import String

#coppeliasim imports
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from math import pi

class RoboticEnvironment():
    def __init__(self):

        #initalise ros stuff

        #coppelia configs
        self.client = RemoteAPIClient('localhost',23000)
        self.sim = self.client.getObject('sim')
        self.simIK = self.client.require('simIK')
        self.joint_handles=[]
        self.base_handle=None
        self.ee_handle =None
        self.target_dummy =None
        self.ik_env =None
        self.ik_group=None

    def connect(self):
        self.sim.startSimulation()
        print('Connected to simulation')

    def stop_simulation(self):
        self.sim.stopSimulation()
        print('simulation stopped')

    def initialize_handles(self):
        self.base_handle=self.sim.getObject('/UR10')
        print('Base handle retrieved')
        joints = ['/UR10/joint'+str(i) for i in range(1,7)]
        self.joint_handles = [self.sim.getObject(joint) for joint in joints]
        print('Joint handles retrieved')
        self.ee_handle = self.sim.getObject('/UR10/tip')
        print('End effector handle retrieved')
        self.target_dummy = self.sim.getObject('/Dummy')
        print(f"Position:{self.sim.getObjectPosition(self.target_dummy)}, Orientation: {self.sim.getObjectOrientation(self.target_dummy)}")
    def create_dummy(self,position,orientation):
        dummy_handle = self.sim.createDummy(0.01)
        self.sim.setObjectPosition(dummy_handle,-1,position)
        self.sim.setObjectOrientation(dummy_handle,-1,orientation)
        print("Target dummy created")
        return dummy_handle

    def setup_ik(self):
        self.ik_env = self.simIK.createEnvironment()
        self.ik_group = self.simIK.createGroup(self.ik_env)
        self.simIK.setGroupCalculation(self.ik_env,self.ik_group,self.simIK.method_damped_least_squares, 0.3, 99)
        
        constraints =0
        if hasattr(self.simIK,'constraint_pose'):
            constraints = self.simIK.constraint_pose
            print(f'using constraints {constraints}')
        try:
            self.simIK.addElementFromScene(self.ik_env,self.ik_group,self.base_handle,self.ee_handle,self.target_dummy,constraints)
        except Exception as e:
            print(f"Error setting up IK ")
    # def setup_ik(self):
    #     self.ik_env = self.simIK.createEnvironment()
    #     self.ik_group = self.simIK.createGroup(self.ik_env)
    #     self.simIK.setGroupCalculation(self.ik_env, self.ik_group, self.simIK.method_damped_least_squares, 0.3, 99)

    #     constraints = 0
    #     if hasattr(self.simIK, 'constraint_pose'):
    #         constraints = self.simIK.constraint_pose
    #         print(f'Using constraints: {constraints}')

    #     try:
    #         # Extract relevant IK data
    #         ikElement, simToIkObjectMap, ikToSimObjectMap = self.simIK.addElementFromScene(
    #             self.ik_env, self.ik_group, self.base_handle, self.ee_handle, self.target_dummy, constraints
    #         )

    #         print(f"simToIkObjectMap: {simToIkObjectMap}")
    #         print(f"ikToSimObjectMap: {ikToSimObjectMap}")

    #         # Extract joint handles from the mapping
    #         extracted_joints = sorted(simToIkObjectMap.keys())  # Sorted to maintain order
    #         print(f"Extracted joint handles from IK: {extracted_joints}")

    #         # Compare with manually defined joints
    #         print(f"Manually defined joint handles: {self.joint_handles}")

    #         # If there’s a mismatch, update self.joint_handles
    #         if set(self.joint_handles) != set(extracted_joints):
    #             print("Mismatch detected! Using extracted joints instead.")
    #             self.joint_handles = extracted_joints

    #     except Exception as e:
    #         print(f"Error setting up IK: {e}")


    def get_distance(self, handle1, handle2):
        position1 = self.sim.getObjectPosition(handle1, -1)
        position2 = self.sim.getObjectPosition(handle2, -1)
        # Calculate the Euclidean distance in 3D space (x, y, z)
        return sum((p1 - p2) ** 2 for p1, p2 in zip(position1, position2)) ** 0.5

    def move_to_configuration(self,target_pose):
        #create a dummy at the pose
        # self.target_dummy =self.create_dummy([1.850,0.325,0.606],[-pi/2,0,-pi/2])
        self.setup_ik()
        # configs = [self.sim.getJointPosition(joint) for joint in self.joint_handles]
        # print(configs)
        tries=0
        result,reason,precision = self.simIK.handleGroup(self.ik_env,self.ik_group,{'syncWorlds':True})
        # ikconfigs= self.simIK.findConfig(self.ik_env,self.ik_group,self.joint_handles)
        # print(ikconfigs)
        while True:
            reso =self.get_distance(self.ee_handle,self.target_dummy)
            
            if reso<0.01 or tries>1000:
                configs = [self.sim.getJointPosition(joint) for joint in self.joint_handles]
                print(configs)
                break
            tries+=1
        if result:
            print("IK calculation succeeded")
            return True
        else:
            print("failed to handle IK group")
            return False
   
        
    def get_target_configuration(self, target_pose):
        """
        Gets a joint configuration that would achieve the target_pose without moving the robot.
        
        :param target_pose: A tuple or list with 6 values (x, y, z, alpha, beta, gamma)
                            representing the desired position and orientation (in radians).
        :return: A list of joint values (configuration) if found, otherwise None.
        """
        # Set the target dummy's pose to the desired target_pose.
        # Assuming target_pose = [x, y, z, alpha, beta, gamma]
        # self.sim.setObjectPosition(self.target_dummy, -1, target_pose[0:3])
        # self.sim.setObjectOrientation(self.target_dummy, -1, target_pose[3:6])
        
        # (Re)initialize IK environment/group if needed
        self.setup_ik()  # Ensure self.ik_env and self.ik_group are correctly set up.
        
        # Set up search parameters for findConfigs.
        params = {
            'maxDist': 0.1,         # Only consider configurations that produce a tip close to target.
            'maxTime': 2.0,         # Maximum time (in seconds) to search.
            'pMetric': [1, 1, 1, 0.5],  # Pose metric: weights for [dx, dy, dz, dAngle]. Adjust as needed.
            'findMultiple': False   # Only the best (first) solution is required.
            # You can add more parameters such as 'cMetric', 'findAlt', or a callback 'cb' if needed.
        }
        #Give a config as seed
        current_config = [0]*len(self.joint_handles)#[self.sim.getJointPosition(joint) for joint in self.joint_handles]
        # Call simIK.findConfigs to search for a configuration that matches the target dummy pose.
        configs = self.simIK.findConfigs(self.ik_env, self.ik_group, self.joint_handles, params, current_config)
        
        if configs and len(configs) > 0:
            chosen_config = configs[0]  # The first configuration is the best one found.
            print("Found a valid IK configuration:")
            for i, joint_value in enumerate(chosen_config):
                print(f"Joint {i+1}: {joint_value}")
            return chosen_config
        else:
            print("No valid IK configuration found for the target pose.")
            return None
    def get_target_configuration_deprecated(self, target_pose):
        """
        Gets a joint configuration for the given target_pose using the deprecated simIK.findConfig.
        
        :param target_pose: A tuple or list with 6 values [x, y, z, alpha, beta, gamma]
                            representing the desired target pose.
        :return: A list of joint values (configuration) if found, otherwise None.
        """
        # Set the target dummy's pose to the desired target_pose.
        # Note: target_pose = [x, y, z, alpha, beta, gamma]
        # self.sim.setObjectPosition(self.target_dummy, -1, target_pose[0:3])
        # self.sim.setObjectOrientation(self.target_dummy, -1, target_pose[3:6])
        
        # (Re)initialize IK if needed. Ensure self.ik_env and self.ik_group are valid.
        self.setup_ik()  # This function should set self.ik_env and self.ik_group correctly.
        
        # Define parameters for the IK search:
        thresholdDist = 0.1  # Only consider configurations that produce a tip within 0.1 units of the target.
        maxTime = 0.5        # Maximum search time in seconds.
        metric = [1, 1, 1, 0.1]  # Pose metric: weights for [dx, dy, dz, dAngle]. Adjust if necessary.
        
        try:
            # Call the deprecated simIK.findConfig to search for an IK solution.
            joint_config = self.simIK.findConfig(
                self.ik_env,
                self.ik_group,
                self.joint_handles,
                thresholdDist,
                maxTime,
                metric,
                None,   # No validation callback
                None    # No auxiliary data
            )
        except Exception as e:
            print("Error during simIK.findConfig:", e)
            ik_joints = self.simIK.getConfigForTipPose(self.ik_env, self.ik_group,self.joint_handles)
            
            print("Joint handles:", self.joint_handles)
            return None
        
        if joint_config:
            print("Found IK configuration (deprecated):", joint_config)
            return joint_config
        else:
            print("No valid IK configuration found using simIK.findConfig.")
            return None
 
    def move_to_config(self,target_config):
        #moves manipulator to target c space config in radians
        n_joints = len(self.joint_handles)
        maxVel = [2.094395102]*n_joints #rad/s
        maxAccel = [0.698131701]*n_joints #rad/s^2
        maxJerk = [1.396263402]*n_joints #rad/s^3

        state =self.sim.moveToConfig({

            'joints':self.joint_handles,
            'maxVel':maxVel,
            'maxAccel':maxAccel,
            'maxJerk':maxJerk,
            'targetPos': target_config
        }
            )
        return state


def main():
    env = RoboticEnvironment()
    env.connect()
    env.initialize_handles()
    env.move_to_configuration(9)
    # env.set_joint_to_config([])
    # env.move_to_config([0.23141078789183975, 1.2240021699772328, 1.9758523090714766, -1.541240219625911, 1.591575825355268, -0.7027195550025449])
    # env.move_to_config([0.3989910845719479, 2.1435238238363294, -0.15221753013363898, -2.1080604947994734, 2.6365143775571744, 0.9988351580545902])#([0.38761265523134303, 1.4816286443570872, 1.3111003213041332, -2.5730493290047773, 2.7319379968363147, -0.2711492036361079])
    # env.move_to_config( [0.9003445122443277, 0.046505136555976545, 1.8862136715987274, -1.7909194909514174, 0.6752951007542772, -0.582252705721346])
    while True:
        pass
    
    #keep the sim alive
    # rclpy.spin(env)

    #stop simulation and destroy node
    env.stop_simulation()
    # env.destroy_node()
    # rclpy.shutdown()

 
if __name__ == '__main__':
    main()



