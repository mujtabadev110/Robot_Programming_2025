import math
import numpy as np



# theta1 = 0.0
# theta2 = 0.0
# theta3 = 0.0
# theta4 = 0.0
# theta5 = 0.0
# theta6 = 0.0

r1 = 0.0 
r2 = 0.0
r3 = 0.0
a1 = 0.24
a2 = 0.21
a3 = 0.21
phi1 = 0.0
phi2 = 0.0 
phi3 = 0.0


# Inverse Kinematics Function (Analytical)

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

#Orientation Part

    R0_6 = np.array([
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]
        ], dtype=float)

    R0_3 = np.array([
        [-np.cos(theta1)*np.sin(theta3 + theta2), np.sin(theta1), np.cos(theta1)*np.cos(theta2 - theta3)],
        [-np.sin(theta1)*np.sin(theta3 + theta2), -np.cos(theta1), np.sin(theta1)*np.cos(theta2 - theta3)],
        [np.cos(theta2 - theta3), 0.0, np.sin(theta2 + theta3)]
        ], dtype=float)

    invR0_3 = np.linalg.inv(R0_3)
    R3_6 = np.dot(invR0_3, R0_6)


    theta5 = np.arccos(R3_6[2][2])

    theta6 = np.arccos(-R3_6[2][0] / np.sin(theta5))

    theta4 = np.arccos(R3_6[1][2] / np.sin(theta5))


    return theta1, theta2, theta3, theta4, theta5, theta6   
