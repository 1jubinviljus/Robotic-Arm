# Robotic Arm Project

An advanced, fully custom robotic arm—designed, 3D-printed, and assembled from scratch. This project features seamless integration between hardware and software: movements are planned in a Python simulation (PyBullet, URDF) and executed in real time by the physical arm, driven by an Arduino with a CNC shield.

**Key Features & Technologies:**
- End-to-end mechatronic design: SolidWorks (CAD), 3D printing, and assembly
- Realistic simulation and visualization: Python , PyBullet, URDF
- Direct hardware control: Arduino + CNC shield, C++ firmware, serial communication from Python

**Repository Structure:**
- `hardware/`: CAD and STL files
- `software/arduino_firmware/`: Arduino firmware
- `software/simulation/urdf/`: Robot model (URDF)
- `software/simulation/pybullet_env/`: Python simulation

**Note:**
URDF model is used for both simulation and visualization. Pin assignments in firmware match hardware wiring for reliable operation.

---
For more details, see source files.