
from examples.pybullet.utils.pybullet_tools.utils import WorldSaver,connect,dump_body,get_pose,set_pose,set_default_camera,draw_global_system,HideOutput,load_model,show_axes
import numpy as np
import pybullet as p
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



def load_world():
    set_default_camera()
    show_axes()
    draw_global_system()
    with HideOutput():
        robot_base_pose =[1.2469990249203744, 0.013001600910179347, 0.7852500204890966, 0.0, 0.0, -0.707106781186621, 0.7071067811864743]
        robot = load_model('/home/user/AetherPlan/misc/ur10_e.urdf',pose=(robot_base_pose[:3],robot_base_pose[3:]),fixed_base=True)
        floor_pose = [0.0, 0.0, 0, 0.0, 0.0, 0.0, 1.0]
        floor = load_model('/home/user/AetherPlan/misc/floor.urdf',pose=(floor_pose[:3],floor_pose[3:]))
        # pillar = load_model()
        table_pose =[0.05000000000000001, -0.4750000000000001, 0.25, 0.0, 0.0, 0.0, 1.0]
        table = load_model('/home/user/AetherPlan/misc/table1.urdf',pose=(table_pose[:3],table_pose[3:]))
        body_names=None
        movable = None
        
    return robot,body_names,movable
    

def main():
    connect(use_gui=True)
    robot,names,movable = load_world()
    saver = WorldSaver()

    while True:
        pass

if __name__=='__main__':
    main()