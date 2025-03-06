import pybullet as p
import pybullet_data
import numpy as np

# Connect PyBullet in silent mode
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load the robot (KUKA iiwa as an example)
robot_id = p.loadURDF("misc/ur10_e.urdf", useFixedBase=True)

# Define target position and orientation
target_pos = [1.39522,0.57788, 1.42524] # XYZ in meters
target_ori = (0.0, 0.0, -0.7071067811865475, 0.7071067811865476)#p.getQuaternionFromEuler([0, 0, 0])  # Convert Euler to quaternion

end_effector_index = 6  # Change based on your robot

# Define error threshold
pos_threshold = 0.001  # 1 mm position accuracy
ori_threshold = 0.8   # Small orientation error threshold

max_attempts = 50  # Number of attempts to find a good solution
found_solution = False

for attempt in range(max_attempts):
    # Run IK solver
    ik_solution = p.calculateInverseKinematics(robot_id, end_effector_index, target_pos, target_ori)

    # Apply IK solution to the robot
    for i, joint_value in enumerate(ik_solution):
        p.resetJointState(robot_id, i, joint_value)

    # Get actual end-effector position after applying IK
    actual_pos, actual_ori = p.getLinkState(robot_id, end_effector_index)[4:6]

    # Compute errors
    pos_error = np.linalg.norm(np.array(target_pos) - np.array(actual_pos))
    ori_error = np.linalg.norm(np.array(target_ori) - np.array(actual_ori))

    print(f"Attempt {attempt+1}: Position Error = {pos_error:.6f}, Orientation Error = {ori_error:.6f}")

    # Check if within threshold
    if pos_error < pos_threshold and ori_error < ori_threshold:
        print("Found IK solution within error threshold!")
        found_solution = True
        print("IK Solution:", list(ik_solution[:6]))
        break

if not found_solution:
    print("Could not find an IK solution within the error threshold.")
while True:
    pass
# Disconnect
p.disconnect()
