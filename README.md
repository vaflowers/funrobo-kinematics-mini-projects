# Fundamentals of Robotics: Mini-Project 1
## Hiwonder & Kinova: Kinematics and RRMC

This repository contains the kinematic implementations for the 5-DOF Hiwonder arm and the 6-DOF Kinova arm, alongside a Resolved-Rate Motion Control (RRMC) system for the Hiwonder. 

All implementations are derived using Denavit-Hartenberg (DH) conventions. The RRMC system utilizes Jacobian-based velocity control to translate real-time gamepad inputs into motion.

> [!IMPORTANT]  
> These files are designed to function within a specific ecosystem. They require the FunRobo visualization tool, ROS files, and project environment to run successfully.

---

## Forward Kinematics (Simulation)
Before implementing motion control, we modeled the Hiwonder and Kinova arms by deriving their DH parameters to calculate Forward Position Kinematics (FPK). This allowed for accurate mapping of joint states to end-effector poses within the visualization tool.

**Core files:**
* `FiveDOF.py` (Hiwonder DH Model)
* `SixDOF.py` (Kinova DH Model)



---

## Velocity Kinematics (Simulation)
We derived the geometric Jacobian and its inverse to enable velocity-level control. This implementation allows the robot to calculate the necessary joint velocities required to achieve a desired end-effector velocity in task space.

**Core file:**
* `FiveDOFRRMC.py`



---

## Resolved-Rate Motion Control (RRMC)
The final phase involved porting the RRMC calculations to the physical Hiwonder hardware. This script interfaces with a gamepad, allowing a user to drive the robot arm in the $X, Y, \text{and } Z$ directions. This file was modified to include radian-to-degree conversion for the servos.

**Core file:**
* `hiwonder_rrmc.py`

---
