import numpy as np
import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)

# Load assets
p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])
targid = p.loadURDF(
    "software/simulation/urdf/simple_arm.urdf",
    [0, 0, 0],
    [0, 0, 0, 1],
    useFixedBase=True
)

obj_of_focus = targid

# Disable specular highlights (glare)
p.changeVisualShape(targid, -1, specularColor=[0, 0, 0])  # For the base link
for link_index in range(p.getNumJoints(targid)):
    p.changeVisualShape(targid, link_index, specularColor=[0, 0, 0])

# Set initial camera view, but allow mouse rotation
p.resetDebugVisualizerCamera(
    cameraDistance=0.5,
    cameraYaw=45,
    cameraPitch=-30,
    cameraTargetPosition=[0, 0, 0.1]
)

# Hide PyBullet's side GUI
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)


# Run simulation
while True:  # infinite loop so you can rotate freely
    p.stepSimulation()
    time.sleep(1. / 240.)
