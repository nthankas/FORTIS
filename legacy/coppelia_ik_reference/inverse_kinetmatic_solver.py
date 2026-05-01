import numpy as np
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# ===================================================================
# FORTIS Inverse Kinematics Solver - by Carlos Vazquez
# ===================================================================
# 1. INITIALIZATION: MY SIMULATION ENVIRONMENT
# This is where I establish the link between my Python script and 
# the CoppeliaSim engine to control my 4 joints.
# ===================================================================
client = RemoteAPIClient()
sim = client.require('sim')

# I'm mapping the joint handles here so I can send position commands to them later.
joint_names = [f'/joint{i+1}' for i in range(4)]
joints = [sim.getObject(name) for name in joint_names]

# These are the physical offsets of my robot's base in the scene.
BASE_X_OFFSET = 0.2286
BASE_Z_OFFSET = 0.127

# I extracted these DH (Denavit-Hartenberg) parameters to define 
# my robot's specific geometry and link lengths.
DH_PARAMS = [
    [0.1270,  -180.0,  0.0250,  np.radians(-90)],
    [0.0000,    85.0,  0.5761,  0.0            ],
    [0.0000,  -170.0,  0.5000,  0.0            ],
    [0.0000,    90.0,  0.1500,  0.0            ],
]

# ===================================================================
# 2. FORWARD KINEMATICS: LOCATING THE TIP
# ===================================================================
def get_state(q):
    """
    I built this to translate joint angles into a 3D coordinate. 
    It also checks if the 'stinger' is pointing perfectly into the wall.
    """
    T = np.eye(4)
    for i in range(4):
        d, offset, r, alpha = DH_PARAMS[i]
        theta = q[i] + np.radians(offset)
        
        # This is the standard transformation matrix I use to move from 
        # one joint's frame to the next.
        M = np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha),  r*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha),  r*np.sin(theta)],
            [0,              np.sin(alpha),                 np.cos(alpha),                d              ],
            [0,              0,                             0,                            1              ]
        ])
        T = T @ M

    # Final position of the end-effector in 3D space.
    x, y, z = T[0, 3], T[1, 3], T[2, 3]

    # I want my tip pointing along +Z. T[2,1] represents the orientation 
    # of my tip; I'm calculating the error relative to my target pose.
    forward_err = T[2, 1] - 1.0

    return np.array([x, y, z, forward_err])



# ===================================================================
# 3. INVERSE KINEMATICS: SOLVING FOR ANGLES
# ===================================================================
def ik_solve_adaptive(target_xyz, q_guess):
    """
    This is my custom IK solver. It uses 'Damped Least Squares' to 
    find the angles needed to hit a specific point on the wall.
    """
    q = np.array(q_guess, dtype=float)
    target_state = np.append(target_xyz, 0.0)

    for _ in range(500):
        current = get_state(q)
        error = target_state - current
        if np.linalg.norm(error) < 1e-4:
            break

        # I use a numerical Jacobian to understand how joint changes 
        # affect the tip's position in real-time.
        J = np.zeros((4, 4))
        delta = 1e-6
        for i in range(4):
            q2 = np.copy(q)
            q2[i] += delta
            J[:, i] = (get_state(q2) - current) / delta

        # Damping factor: I added this to keep the movement stable 
        # even when the arm is fully extended.
        damping = 0.08
        JT = J.T
        inv = JT @ np.linalg.inv(J @ JT + damping**2 * np.eye(4))

        step = inv @ error

        # My Null-space preference: I've biased the robot to stay 
        # within a comfortable middle range of motion.
        q_mid = np.array([0.0, np.radians(35), np.radians(-70), np.radians(-50)])
        null_step = (np.eye(4) - inv @ J) @ (q_mid - q) * 0.1

        q = (q + step + null_step + np.pi) % (2 * np.pi) - np.pi

    return q

# ===================================================================
# 4. MY TARGET SEQUENCE & TRAJECTORY
# ===================================================================
wall_x_arm = 0.65
target_list = [
    [0.65 - BASE_Z_OFFSET, 0.0, wall_x_arm],
    [0.35 - BASE_Z_OFFSET, 0.0, 0.35], # Return to my 'Home' position
]

sim.startSimulation()

# Setting my initial "Scorpion Pose"
current_q = np.array([0.0, np.radians(45), np.radians(-90), np.radians(130)])

try:
    for t in target_list:
        # I calculate the destination angles
        next_q = ik_solve_adaptive(np.array(t), current_q)
        q_start = np.array([sim.getJointPosition(j) for j in joints])
        diff = (next_q - q_start + np.pi) % (2 * np.pi) - np.pi

        # I've implemented a smooth interpolation loop to move between points.
        for step in range(80):
            alpha = step / 79
            smooth = 3*alpha**2 - 2*alpha**3 # My smoothstep curve
            q_interp = q_start + smooth * diff

            # ADAPTIVE LOGIC: I'm overriding J4 here to ensure the 
            # tip is always level, regardless of how J2 and J3 move.
            j2, j3 = q_interp[1], q_interp[2]
            q_interp[3] = -(j2 + j3) + np.radians(175)

            for idx, joint_obj in enumerate(joints):
                sim.setJointTargetPosition(joint_obj, q_interp[idx])
            sim.step()
            time.sleep(0.01)

        current_q = next_q
        time.sleep(0.6)

finally:
    sim.stopSimulation()