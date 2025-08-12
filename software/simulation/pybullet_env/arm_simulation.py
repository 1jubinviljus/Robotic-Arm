import numpy as np
import pybullet as p
import pybullet_data
import time

# ------------------------
# Initialize PyBullet and simulation
# ------------------------
p.connect(p.GUI)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)

# Load ground plane
p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])

# Load robot arm URDF with fixed base
targid = p.loadURDF(
    "software/simulation/urdf/simple_arm.urdf",
    [0, 0, 0],
    [0, 0, 0, 1],
    useFixedBase=True
)

# ------------------------
# Visual configuration
# ------------------------

# Disable specular highlights (remove glare)
p.changeVisualShape(targid, -1, specularColor=[0, 0, 0])
for link_index in range(p.getNumJoints(targid)):
    p.changeVisualShape(targid, link_index, specularColor=[0, 0, 0])

# Set initial camera view (distance, yaw, pitch, target position)
p.resetDebugVisualizerCamera(
    cameraDistance=0.5,
    cameraYaw=45,
    cameraPitch=-30,
    cameraTargetPosition=[0, 0, 0.1]
)

# Hide PyBullet's side GUI panel
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

# ------------------------
# Find the tool_link joint index to use as end effector
# ------------------------
end_effector_link_index = None
for i in range(p.getNumJoints(targid)):
    info = p.getJointInfo(targid, i)
    link_name = info[12].decode()
    # print(f"Joint {i}: child link = {link_name}")  # uncomment to debug
    if link_name == "tool_link":
        end_effector_link_index = i
        break

if end_effector_link_index is None:
    print("Warning: 'tool_link' not found in URDF joints. Using last joint index as end effector.")
    end_effector_link_index = p.getNumJoints(targid) - 1

print(f"Using end effector link index: {end_effector_link_index}")

# ------------------------
# Target point setup
# ------------------------

# Start with the target point at the origin
target_x = 0.0  # meters
target_y = 0.0  # Keep in x-z plane, so y is always 0 for now
target_z = 0.0  # meters

# Create a small blue sphere to mark the target position
point_visual = p.createVisualShape(
    shapeType=p.GEOM_SPHERE,
    radius=0.02,
    rgbaColor=[0, 0, 1, 1]
)

point_body = p.createMultiBody(
    baseMass=0,  # Static object (no physics)
    baseVisualShapeIndex=point_visual,
    basePosition=[target_x, target_y, target_z]
)

# ------------------------
# Function to move robot arm to given target position
# ------------------------
def move_to_target(x, y, z):
    target_pos = [x, y, z]
    joint_angles = p.calculateInverseKinematics(
        bodyUniqueId=targid,
        endEffectorLinkIndex=end_effector_link_index,
        targetPosition=target_pos
    )
    joint_indices = [0, 1, 2]  # your 3 joints
    for i, joint_index in enumerate(joint_indices):
        p.setJointMotorControl2(
            bodyUniqueId=targid,
            jointIndex=joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=joint_angles[i],
            force=500
        )

# Function to update target position
def moveTarget(x, y, z):
    x += 0.001  # Move target along x-axis
    y += 0.1    # Keep y=0 for x,z plane movement
    z += 0.002  # Move target along z-axis
    return x, y, z

# Function to get end effector position
def get_end_effector_position(body_id, link_index):
    state = p.getLinkState(body_id, link_index)
    pos = state[4]  # world position of link frame (link COM)
    return np.array(pos)

# ------------------------
# Main simulation loop
# ------------------------
# Threshold to decide if arm reached the target (meters)
REACH_THRESHOLD = 0.01
while True:
    # Calculate IK and move joints
    move_to_target(target_x, target_y, target_z)
    p.stepSimulation()

    # Check current end effector position
    ee_pos = get_end_effector_position(targid, end_effector_link_index)
    target_pos = np.array([target_x, target_y, target_z])

    dist = np.linalg.norm(ee_pos - target_pos)

    # Only update target once arm is close enough
    if dist < REACH_THRESHOLD:
        target_x, target_y, target_z = moveTarget(target_x, target_y, target_z)
        p.resetBasePositionAndOrientation(point_body, [target_x, target_y, target_z], [0, 0, 0, 1])

    time.sleep(1. / 240.)  # simulation step time (240Hz = 240)
