from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import math

class RoboticsEnvironment():
    def __init__(self):
        self.client = RemoteAPIClient('localhost',23000)
        self.sim = self.client.getObject('sim')
        self.simIK = self.client.require('simIK')
        self.simOMPL =self.client.require('simOMPL')
        
    def connect(self):
        '''
        Connects to coppeliasim and sets sim parameters
        '''
        self.sim.startSimulation()
        print('connected to simulation')
        self.sim.setStepping(True)
        print('simulation is explicitly stepped')
    
    def stop_simulation(self):
        '''
        stops simulation
        '''
        self.sim.stopSimulation()

    def initialize_params(self):
        '''
        Initialise robot parameters and solver configs
        '''

        #get joint handles
        joint_names = ['/UR10/joint'+str(i) for i in range(1,7)]
        self.joints = [self.sim.getObject(joint) for joint in joint_names]
        print('Joint handles retrieved')
        #TODO: get gripper sensor handle

        #get tip handle
        self.robotTip = self.sim.getObject('/UR10/tip')
        #get target handle
        self.robotTarget = self.sim.getObject('/UR10/target')
        #get base handle
        self.robotBase=self.sim.getObject('/UR10')
        #create a robot collectoin
        self.robotCollection = self.sim.createCollection()
        self.sim.addItemToCollection(self.robotCollection,self.sim.handle_tree,self.robotBase,0)
        self.pathPlanningMaxtime = 4.0
        self.pathPlanningSimplificationTime=4.0
        self.pathPlanningAlgo = self.simOMPL.Algorithm.RRTstar

        #TODO:define dropping point poses

        #IK Motions
        self.ikMaxVel=[0.4,0.4,0.4,1.8]
        self.ikMaxAccel=[0.8,0.8,0.8,0.9]
        self.ikMaxJerk=[0.6,0.6,0.6,0.8]

        #FK Motions
        fkVel=180
        fkAccel=40
        fkJerk=80
        
        self.fkMaxVel = [fkVel*math.pi/180]*6
        self.fkMaxAccel = [fkAccel*math.pi/180]*6
        self.fkMaxJerk =[fkJerk*math.pi/180]*6
    
    def getConfig(self):
        '''
        gets current robot configuration 

        Input: None
        Output: c space variable configuration q of dimention 1xn_joints
        '''
        return [self.sim.getJointPosition(joint) for joint in self.joints]
    
    def collides(self,target_configs):
        '''
        checks if any configuration self collides
        input: List of configurations
        output:bool collision 
        '''
        retVal = False
        bufferedConfig = self.getConfig()
        for target in target_configs:
            self.setConfig(target)
            res = self.sim.checkCollision(self.robotCollection,self.sim.handle_all)[0]
            if res >0:
                retVal = True
                break
            else:
                res= self.sim.checkCollision(self.robotCollection,self.robotCollection)[0]
                if res >0:
                    retVal = True
                    break
        self.setConfig(bufferedConfig)
        return retVal
    def setConfig(self,config):
        '''
        sets joint position to given configuration
        '''
        n_joints = len(self.joints)
        for i in range(n_joints):
            self.sim.setJointPosition(self.joints[i],config[i])
    
    def findConfigs(self,pose):
        '''
        returns configurations for the manipultor for given pose
        '''
        ikEnv = self.simIK.createEnvironment()
        ikGroup = self.simIK.createGroup(ikEnv)
        ikEl,simToIk,ikToSim = self.simIK.addElementFromScene(ikEnv,ikGroup,self.robotBase,self.robotTip,self.robotTarget,self.simIK.constraint_pose)
        ikJoints=[simToIk[joint] for joint in self.joints]
        self.sim.setObjectPose(self.robotTarget,pose)
        self.simIK.syncFromSim(ikEnv,[ikGroup])
        p={
            'maxDist':0.28,
            'maxTime':1,
            'cMetric':[8,8,8,0.8,0.6,0.3],
            'findMultiple': True
        }
        retVal = self.simIK.findConfigs(ikEnv,ikGroup,ikJoints,p)
        self.simIK.eraseEnvironment(ikEnv)
        return retVal
    
    def selectOneValidConfig(self,configs,approachIKTr,withdrawIkTr):
        '''
        Picks a valid configuration out of available ik configs

        input: list of configs, approach,withsraw IK transforms
        returns: a valid configuration, a visualisation object
        '''
        bufferedConfig = self.getConfig()
        i=0
        for target in configs:
            if not self.collides([target]):
                print("found no collision")    
                self.setConfig(target)

                if approachIKTr:
                    pose = self.sim.getObjectPose(self.robotTip)
                    targetPose = self.sim.multiplyPoses(pose,approachIKTr)
                    self.sim.setObjectPose(self.robotTarget,targetPose)
                    ikEnv = self.simIK.createEnvironment()
                    ikGroup = self.simIK.createGroup(ikEnv)
                    ikEl,simToIk,ikToSim = self.simIK.addElementFromScene(ikEnv,ikGroup,self.robotBase,self.robotTip,self.robotTarget,self.simIK.constraint_pose)
                    ikJoints=[]
                    for joint in self.joints:
                        ikJoints.append(simToIk[joint])
                    
                    path = self.simIK.generatePath(ikEnv,ikGroup,ikJoints,simToIk[self.robotTip],4)
                    self.simIK.eraseEnvironment(ikEnv)
                    if path:
                        #convert path into a list of configs to check for collisions
                        print(path)
            else:
                print(f"collision in config {i}")
            i+=1

    def ActionPick(self,pickPose,approachIKTr,withdrawIktr):
        #fing possible configurations
        configs = self.findConfigs(pickPose)
        #if more than one configuration is present, pick a valid one 
        n_configs=len(configs)
        if n_configs>0:
            #TODO:write a funciton to select valid configurations
            print(f'Found {n_configs} potential configurations')
            pickConfig,passiveVizShape = self.selectOneValidConfig(configs,approachIKTr,withdrawIktr)
            self.sim.step()
            print('selected configuration')
        else:
            print('Failed to find a valid configuration for the desired pick')
  

def main():
    env = RoboticsEnvironment()
    env.connect()
    env.initialize_params()
    initConfig = env.getConfig()
    
    #get a pick pose
    pickItem = env.sim.getObject('/pickPose')
    pickPose = env.sim.getObjectPose(pickItem)
    #TODO: open the gripper and wait

    #pick the item
    #the appoach and withdrawal transforms have distance as pose transform
    outcome = env.ActionPick(pickPose,[0, 0, -0.105, 0, 0, 0, 1],[0, 0,0.105, 0, 0, 0, 1])

    # q = input('Quit ?')
    env.stop_simulation()

if __name__ == '__main__':
    main()