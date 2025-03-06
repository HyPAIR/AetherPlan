import pybullet as p
import pybullet_data
from math import pi
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

ur10_id = p.loadURDF("misc/robot/ur10e.urdf", useFixedBase=True)

end_effector_index = 6 # The last joint in UR10
# Target position (XYZ)
target_pos = [1.39522,0.57788, 1.42524]  # Change these as needed

# Target orientation (Euler angles α, β, γ)
# target_euler = [-90*pi/180, 0, -90*pi/180]  # Roll, pitch, yaw in radians
target_euler =[-180*pi/180,90*pi/180,0*pi/180,]

# Convert Euler angles to quaternion
# target_orientation = p.getQuaternionFromEuler(target_euler)
target_orientation=(0.0, 0.0, -0.7071067811865475, 0.7071067811865476)
print(target_orientation)
# Compute inverse kinematics (IK) with position + orientation
joint_angles = p.calculateInverseKinematics(
    ur10_id, end_effector_index, target_pos, target_orientation
)
# # Solve IK
# joint_angles = p.calculateInverseKinematics(ur10_id, end_effector_index, target_pos)





# Apply the computed joint angles
for i in range(len(joint_angles)):
    p.setJointMotorControl2(ur10_id, i, p.POSITION_CONTROL, joint_angles[i])

print("IK Solution:", list(joint_angles[:6]))


# Disconnect PyBullet
# p.disconnect()
