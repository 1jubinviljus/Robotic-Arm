import numpy as np
import pybullet as p
import pybullet_data
import time

class ArmSimulation:
    def __init__(self):
        # Initialize PyBullet and simulation
        p.connect(p.GUI)
        p.resetSimulation()
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0)

        # Load ground plane
        p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])

        # Load robot arm URDF with fixed base
        self.targid = p.loadURDF(
            "simulation/urdf/simple_arm.urdf",
            [0, 0, 0],
            [0, 0, 0, 1],
            useFixedBase=True
        )

        # Visual configuration
        p.changeVisualShape(self.targid, -1, specularColor=[0, 0, 0])
        for link_index in range(p.getNumJoints(self.targid)):
            p.changeVisualShape(self.targid, link_index, specularColor=[0, 0, 0])

        p.resetDebugVisualizerCamera(
            cameraDistance=0.5,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0.1]
        )
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

        # Find the tool_link joint index to use as end effector
        self.end_effector_link_index = None
        for i in range(p.getNumJoints(self.targid)):
            info = p.getJointInfo(self.targid, i)
            link_name = info[12].decode()
            if link_name == "tool_link":
                self.end_effector_link_index = i
                break
        if self.end_effector_link_index is None:
            print("Warning: 'tool_link' not found in URDF joints. Using last joint index as end effector.")
            self.end_effector_link_index = p.getNumJoints(self.targid) - 1
        print(f"Using end effector link index: {self.end_effector_link_index}")

        # Target point setup
        self.target_x = 0.1
        self.target_y = 0.0
        self.target_z = 0.1
        point_visual = p.createVisualShape(
            shapeType=p.GEOM_SPHERE,
            radius=0.02,
            rgbaColor=[0, 0, 1, 1]
        )
        self.point_body = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=point_visual,
            basePosition=[self.target_x, self.target_y, self.target_z]
        )
        self.REACH_THRESHOLD = 0.01

    def move_to_target(self, x, y, z):
        self.target_x = x
        self.target_y = y
        self.target_z = z

    def get_end_effector_position(self):
        state = p.getLinkState(self.targid, self.end_effector_link_index)
        pos = state[4]
        return np.array(pos)

    def step_simulation(self):
        # Run inverse kinematics towards the current target position
        joint_angles = p.calculateInverseKinematics(
            bodyUniqueId=self.targid,
            endEffectorLinkIndex=self.end_effector_link_index,
            targetPosition=[self.target_x, self.target_y, self.target_z]
        )
        joint_indices = [0, 1, 2]
        for i, joint_index in enumerate(joint_indices):
            p.setJointMotorControl2(
                bodyUniqueId=self.targid,
                jointIndex=joint_index,
                controlMode=p.POSITION_CONTROL,
                targetPosition=joint_angles[i],
                force=500
            )

        p.stepSimulation()

        # Update the visual target sphere to current target position
        p.resetBasePositionAndOrientation(self.point_body, [self.target_x, self.target_y, self.target_z], [0, 0, 0, 1])

        # Optional: check distance and do something if close
        ee_pos = self.get_end_effector_position()
        target_pos = np.array([self.target_x, self.target_y, self.target_z])
        dist = np.linalg.norm(ee_pos - target_pos)
        if dist < self.REACH_THRESHOLD:
            pass  # You could add feedback here

        time.sleep(1. / 240.)

    def disconnect(self):
        p.disconnect()
