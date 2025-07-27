import numpy as np
import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)

# load assets
p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])
targid = p.loadURDF("software/simulation/urdf/robotic_arm.urdf", [0, 0, 0], [0, 0, 0, 1], useFixedBase=True)
obj_of_focus = targid

# Instead of time.sleep(20), run a loop:
for _ in range(240*20):  # roughly 20 seconds at 240 Hz
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()
