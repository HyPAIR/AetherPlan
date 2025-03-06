
from examples.pybullet.utils.pybullet_tools.utils import WorldSaver,connect,dump_body,get_pose,set_pose,set_default_camera,draw_global_system\
    ,HideOutput,load_model,show_axes,create_box,GREEN,set_point,Point,get_point
import numpy as np
import pybullet as p
import time
def transform_pose_from_coppelia_to_pybullet(pose_coppelia):
    x, y, z, qx, qy, qz, qw=pose_coppelia
    # Transform position
    x_new = -y
    y_new = x
    z_new = z  # Z remains unchanged

    # Quaternion for -π/2 rotation about Z-axis
    q_correction = p.getQuaternionFromEuler([0, 0, np.pi/2])

    # Apply quaternion multiplication: q_new = q_corr * q_original
    q_new = p.multiplyTransforms([0, 0, 0], q_correction, [0, 0, 0], [qx, qy, qz, qw])[1]

    return [x_new, y_new, z_new, q_new[0], q_new[1], q_new[2], q_new[3]]

#defining p

#This is where digital twin is setup
def load_world():
    set_default_camera()
    show_axes()
    draw_global_system()
    with HideOutput():
        #Bring in the robot
        robot_base_pose =[0,0,0.75, 0.0, 0, -0.707106781186621, 0.7071067811864743]
        robot_base_pose = transform_pose_from_coppelia_to_pybullet(robot_base_pose)
        robot = load_model('/home/user/AetherPlan/misc/robot/ur10e.urdf',pose=(robot_base_pose[:3],robot_base_pose[3:]),fixed_base=True)
        #Load the floor
        floor_pose = [0.0, 0.0, 0, 0.0, 0.0, 0.0, 1.0]
        floor_pose = transform_pose_from_coppelia_to_pybullet(floor_pose)
        floor = load_model('/home/user/AetherPlan/misc/floor.urdf',pose=(floor_pose[:3],floor_pose[3:]))
        #Load the robot stand
        robot_platform_pose =[0,0,0.5*0.75,0,0,0,1]
        robot_platform = load_model('/home/user/AetherPlan/misc/robot_platform.urdf',pose =(robot_platform_pose[:3],robot_platform_pose[3:]))
        #Load table1
        table_pos = [1.25,0,0.25,0,0,0,1]
        table =load_model('/home/user/AetherPlan/misc/table1.urdf',pose=(table_pos[:3],table_pos[3:]))
        #Load items on to table 1
        pillar_poses=[
            [1.25,-0.1,0.55,0,0,0,1],
            [1.25,0,0.55,0,0,0,1],
            [1.25,+0.1,0.55,0,0,0,1]
        ]
        pillars =[load_model('/home/user/AetherPlan/misc/pillar.urdf',pose=(pillar_pose[:3],pillar_pose[3:])) for pillar_pose in pillar_poses]
        #Load the assembly table
        assembly_table_pose = [0,1.25,0.5*0.5,0,0,0,1]
        # assembly_table_pose = transform_pose_from_coppelia_to_pybullet(assembly_table_pose)
        assembly_table_pose = load_model('/home/user/AetherPlan/misc/assembly_table.urdf',pose=(assembly_table_pose[:3],assembly_table_pose[3:]))
        #create target regions on the assembly table
        x_t=[-0.2,0,0.2]
        y_t=[1.25,1.25,1.25]
        z_t=[0.5,0.5,0.5]
        targets=[]
        for i in range(3):
            targets.append(create_box(0.05,0.05,0.0001,color=GREEN))
            set_point(targets[i],Point(x=x_t[i],y=y_t[i],z=z_t[i])) 

        body_names={
            pillars[0]:'pillarA',
            pillars[1]:'pillarB',
            pillars[2]:'pillarC'
        }
        movable = pillars
   
        
    return robot,body_names,movable
    

def main():
    connect(use_gui=True)
    robot,names,movable = load_world()
    tgt_pose =get_point(movable[0])
    print(tgt_pose)

    poses =[(0.2,1.25,1),(0,1.25,1),(-0.2,1.25,1)]
    for pose in poses:
        ik=p.calculateInverseKinematics(robot,5,pose,(0,0,0,1))
        # Apply IK solution to the robot
        for i, joint_value in enumerate(ik):
            p.resetJointState(robot, i, joint_value)
        time.sleep(1)
    saver = WorldSaver()
    # problem = pddlstream_from_problem()
    while True:
        pass

if __name__=='__main__':
    main()