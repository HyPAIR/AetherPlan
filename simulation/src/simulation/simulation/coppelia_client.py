# import rclpy
# import rclpy.logging
# from rclpy.node import Node

# from std_msgs.msg import String

#coppeliasim imports

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
    def create_dummy(self,position,orientation):
        dummy_handle = self.sim.createDummy(0.01)
        self.sim.setObjectPosition(dummy_handle,-1,position)
        self.sim.setObjectOrientation(dummy_handle,-1,orientation)
        print("Target dummy created")
        return dummy_handle

    # def setup_ik(self):
    #     self.ik_env = self.simIK.createEnvironment()
    #     self.ik_group = self.simIK.createGroup(self.ik_env)
    #     self.simIK.setGroupCalculation(self.ik_env,self.ik_group,self.simIK.method_damped_least_squares, 0.3, 99)
        
    #     constraints =0
    #     if hasattr(self.simIK,'constraint_pose'):
    #         constraints = self.simIK.constraint_pose
    #         print(f'using constraints {constraints}')
    #     try:
    #         self.simIK.addElementFromScene(self.ik_env,self.ik_group,self.base_handle,self.ee_handle,self.target_dummy,constraints)
    #     except Exception as e:
    #         print(f"Error setting up IK ")
    def setup_ik(self):
        self.ik_env = self.simIK.createEnvironment()
        self.ik_group = self.simIK.createGroup(self.ik_env)
        self.simIK.setGroupCalculation(self.ik_env, self.ik_group, self.simIK.method_damped_least_squares, 0.3, 99)

        constraints = 0
        if hasattr(self.simIK, 'constraint_pose'):
            constraints = self.simIK.constraint_pose
            print(f'Using constraints: {constraints}')

        try:
            # Capture the extracted joint handles
            extracted_joints = self.simIK.addElementFromScene(
                self.ik_env, self.ik_group, self.base_handle, self.ee_handle, self.target_dummy, constraints
            )
            print("Extracted IK joint handles:", extracted_joints)

            # Compare with manually defined joints
            print("Manually defined joint handles:", self.joint_handles)

            if set(self.joint_handles) != set(extracted_joints):
                print("Mismatch detected! Using extracted joints instead.")
                self.joint_handles = extracted_joints

        except Exception as e:
            print(f"Error setting up IK: {e}")


    def get_distance(self, handle1, handle2):
        position1 = self.sim.getObjectPosition(handle1, -1)
        position2 = self.sim.getObjectPosition(handle2, -1)
        # Calculate the Euclidean distance in 3D space (x, y, z)
        return sum((p1 - p2) ** 2 for p1, p2 in zip(position1, position2)) ** 0.5

    def move_to_configuration(self,target_pose):
        #create a dummy at the pose
        # self.target_dummy =self.create_dummy([1.850,0.325,0.606],[-pi/2,0,-pi/2])
        self.setup_ik()
        configs = [self.sim.getJointPosition(joint) for joint in self.joint_handles]
        print(configs)
        tries=0
        result,reason,precision = self.simIK.handleGroup(self.ik_env,self.ik_group,{'syncWorlds':True})
        # ikconfigs= self.simIK.findConfig(self.ik_env,self.ik_group,self.joint_handles)
        # print(ikconfigs)
        while True:
            reso =self.get_distance(self.ee_handle,self.target_dummy)
            
            if reso<0.01 or tries>1000:
                break
            tries+=1
        if result:
            print("IK calculation succeeded")
            return True
        else:
            print("failed to handle IK group")
            return False
   
        return configs
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
        current_config = [self.sim.getJointPosition(joint) for joint in self.joint_handles]
        # Call simIK.findConfigs to search for a configuration that matches the target dummy pose.
        configs = self.simIK.findConfigs(self.ik_env, self.ik_group, self.joint_handles, params, [0,0,0,0,0,0,0])
        
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
            ik_joints = self.simIK.getConfigForTipPose(self.ik_env, self.ik_group)
            print("IK-extracted joint handles:", ik_joints)
            print("Manually defined joint handles:", self.joint_handles)
            return None
        
        if joint_config:
            print("Found IK configuration (deprecated):", joint_config)
            return joint_config
        else:
            print("No valid IK configuration found using simIK.findConfig.")
            return None



def main():
    env = RoboticEnvironment()
    env.connect()
    env.initialize_handles()
    # env.move_to_configuration(9)
    # env.get_target_configuration(9)
    env.get_target_configuration_deprecated(9)
    #keep the sim alive
    # rclpy.spin(env)

    #stop simulation and destroy node
    env.stop_simulation()
    # env.destroy_node()
    # rclpy.shutdown()

 
if __name__ == '__main__':
    main()



