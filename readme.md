**6-DoF Robotic Manipulator Arm (MuJoCo)**

This project implements a real-time inverse kinematics (IK) controller for a 6-degree-of-freedom robotic manipulator in MuJoCo, allowing intuitive Cartesian control of the end-effector using keyboard inputs instead of manual joint tuning.

**1. Goal and Problem Statement**
**Problem**

Controlling a 6-DoF robotic arm by manually adjusting each joint angle is highly unintuitive.
To place the robot’s end-effector at a specific point in 3D space, a user must solve complex nonlinear equations for six joint angles:

**θ1, θ2, θ3, θ4, θ5, θ6**
	​


**Goal**

The goal of this project is to provide an intuitive Cartesian-space control system where:
The user moves the end-effector in X, Y, Z using the keyboard.
The system automatically computes the required joint angles using inverse kinematics.
The robot moves in real-time in MuJoCo.

**2. Tools and Technologies:**
    Python, MujoCo, conda env.

**3. Implemented Features**
 - 6-DoF robotic arm modeled in MuJoCo
 - An analytical inverse kinematics solver for computing
 - **θ1, θ2, θ3, θ4, θ5, θ6**
 - Keyboard-based Cartesian control

 - Real-time visualization in MuJoCo viewer

 - Modular code design

 - IK logic separated

 - Input handling separated

 - Simulation loop separated

**4. Keyboard Controls**
Key	Action
↑ (Up Arrow)	Increase Z (move end-effector up)
↓ (Down Arrow)	Decrease Z (move end-effector down)
→ (Right Arrow)	Increase X (move end-effector positive x)
← (Left Arrow)	Decrease X (move end-effector negative x)
B	Increase Y (move end-effector positive y)
V	Decrease Y (move end-effector negative y)

**5. Installation & Running the Project**

**Step 1** — Clone the repository

    git clone https://github.com/mujtabadev110/Robot_Programming_2025.git

**Step 2** — Create Conda Environment

    conda env create -f environment.yml

**Step 3** — Activate Environment

    conda activate mujoco

**Step 4** — change directory to src

    cd /src

**Step 5** — Run the Simulation

    mjpython main.py


**The MuJoCo viewer will open.**

**Use the keyboard to move the robot’s end-effector in real time.**

**6. Outputs**


<img width="1706" height="1330" alt="Screenshot 2026-01-27 at 7 19 07 PM" src="https://github.com/user-attachments/assets/f7f90ac5-33bf-4442-8293-52bf69d14df7" />
<img width="1706" height="1330" alt="Screenshot 2026-01-27 at 6 56 32 PM" src="https://github.com/user-attachments/assets/01260dc5-f360-46f7-b4c2-0d2e9d052b6a" />

