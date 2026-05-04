#!/usr/bin/env python3
"""
Subscribes :    /cmd_vel (geometry_msgs/Twist)
                /joint_states (sensor_msgs/JointState)
Publishes :     /leg_velocity_controller/commands (std_msgs/Float64MultiArray)

Twist --> geometry_msgs - has linear.x/y/z and angular.x/y/z
JointState --> sensor_msgs - has name[], poisiton[], velocity[]
Float64MultiArray --> std_msgs - just a list of float [θ̇₀, θ̇₁, θ̇₂]
"""

import numpy as np
import rclpy
import rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float63MultiArray

# ─────────────────────────────────────────────────────────────
# MATH TOOLS / Helper funcitions
# ─────────────────────────────────────────────────────────────

def vhat(v: np.ndarray) -> np.ndarray:
    """unit vector."""
    n = np.linalg.norm(v)
    return v/n if n > 1e-14 else np.zeros(3)

def rot_aa(u: np.ndarray, theta: float) -> np.ndarray:
    """
    Rodrigues rotation matrix - rotate theta radians about axis u.
    Used in: 
    - rotating link by theta_i, about vhat,
    - rotating the body R during IK updates
    """
    x,y,z = vhat(u)
    c,s,t = np.cos(theta), np.sin(theta), 1.0 - np.cos(theta)
    return np.array([
        [t*x*x + c, t*x*y - s*z, t*x*z + s*y],
        [t*x*y + s*z, t*y*y + c, t*y*z - s*x],
        [t*x*z - s*y, t*y*z + s*x, t*z*z + c],
    ])

def rpy_to_R(roll: float, pitch: float, yaw:float) -> np.ndarray:
    """Convert URDF RPY angles to rotaiton matrix (extrinsic XYZ)."""
    Rx = rot_aa(np.array([1.0, 0.0, 0.0]), roll)
    Ry = rot_aa(np.array([0, 1.0, 0.0]), pitch)
    Rz = rot_aa(np.array([0.0, 0.0, 1.0]), yaw)
    return Rz @ Ry @ Rz

def gauss_solve(A: np.ndarray, b: np.ndarray) -> np.ndarray:
    """
    Gaussian elimination with partial pivoting. Returns None if singular
    If A is singular --> physical condition where the robot hits a configuration
    it can't move out of.
    """
    n = len(b)
    M = np.hstack([A.astype(float), b.reshape(-1,1)])
    for c in range(n):
        piv = c + int(np.argmax(np.abs(M[c:, c])))
        M[[c, piv]] = M[[piv, c]]
        if abs(M[c, c]) < 1e-14:
            return None
        M[c] /= M[c,c]
        for r in range(n):
            if r != c:
                M[r] -= M[r,c] * M[c]
    return M[:, n]


