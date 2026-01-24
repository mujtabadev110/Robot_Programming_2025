import mujoco
import numpy as np
import os
import time
import math
import mujoco.viewer






xml_file = "6RManipulator.xml"

model = mujoco.MjModel.from_xml_path(xml_file)
data = mujoco.MjData(model)


x = 0.42
y = 0.0
z = 0.24

step_size = 0.03 

theta1 = 0.0
theta2 = 0.0
theta3 = 0.0

r1 = 0.0 
r2 = 0.0
r3 = 0.0
a1 = 0.24
a2 = 0.22
a3 = 0.22
phi1 = 0.0
phi2 = 0.0 
phi3 = 0.0


def inverse_kinematics(x, y, z, elbow_up=False):
    
    theta1 = math.atan2(y, x)
    
    r1 = math.sqrt(x**2 + y**2)
    r2 = z - a1 
    phi2 = math.atan2(r2, r1)
    r3 = math.sqrt(r1**2 + r2**2)
    ratio1 = (a3**2 - a2**2 - r3**2) / (-2 * a2 * r3)
    ratio1 = max(-1.0, min(1.0, ratio1))
    phi1 = math.acos(ratio1)

    if elbow_up:
        theta2 = phi2 + phi1
    else:
        theta2 = phi2 - phi1

    ratio3 = (r3**2 - a2**2 - a3**2) / (-2 * a2 * a3)
    ratio3 = max(-1.0, min(1.0, ratio3))
    phi3 = math.acos(ratio3)

    if elbow_up:
        theta3 = -(math.pi - phi3)
    else:
        theta3 = (math.pi - phi3)

    return theta1, theta2, theta3




# 2. Define the keypress handler
def key_callback(keycode):
    global x, y, z
    
    # key = chr(keycode).upper()
    
    if keycode == 265 : z += step_size  # Up
    elif keycode == 264 : z -= step_size  # Down
    elif keycode == 263 : x += step_size  # Right
    elif keycode == 262 : x -= step_size  # Left
    # elif keycode == 'U': y += step_size  # Positive Y
    # elif keycode == 'O': y -= step_size  # Negative Y
    
    else:
        try:
            key = chr(keycode).upper()
            if key == 'B':
                y += step_size  # Q replacement
            elif key == 'V':
                y -= step_size  # E replacement
        except (ValueError, OverflowError):
            # This catches non-character keys to prevent crashes
            pass






# Launch the simulation
with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    start_time = time.time()

    while viewer.is_running():
        current_time = time.time() - start_time
        dt = model.opt.timestep


        theta1, theta2, theta3 = inverse_kinematics(x, y, z, elbow_up=False)

        data.qpos[model.joint("joint A").qposadr] = theta1
        data.qpos[model.joint("joint B").qposadr] = theta2
        data.qpos[model.joint("joint C").qposadr] = theta3


        

        mujoco.mj_step(model, data)
        
        viewer.sync()
        time.sleep(dt)
