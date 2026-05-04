#!/usr/bin/env python3
"""
Jacobian Velocity Controller — Th-RHex
=======================================
Translates a desired 6-DOF body twist into per-joint velocity commands
for the active tripod of a holonomic RHex robot with mecanum-style omni-legs.
 
Theory
------
Each grounded leg imposes two scalar rolling constraints on the body:
 
    dᵢⱼ · ḃᵢ = 0,   j = 1, 2
 
where ḃᵢ is the contact-point velocity and dᵢⱼ spans the plane perpendicular
to the rolling line φᵢ.  Collecting all six constraints yields:
 
    A · V = B · θ̇   →   V = J · θ̇,   J = A⁻¹B
 
where V ∈ ℝ⁶ is the body twist [vx vy vz ωx ωy ωz] and θ̇ ∈ ℝ³ is the
vector of joint velocities.  Inverting gives:
 
    θ̇ = J⁺ · V_desired
 
Reference: "ThRhex Jacobian — Variable Definitions and Kinematic Equations",
           Mark Yim, March 2026.
 
ROS 2 Interface
---------------
Subscribes:
    /cmd_vel                          geometry_msgs/Twist
    /joint_states                     sensor_msgs/JointState
Publishes:
    /leg_velocity_controller/commands std_msgs/Float64MultiArray
 
Parameters (ros2 param):
    num_legs            int   3 or 6  (default 3)
    max_joint_velocity  float rad/s   (default 5.0)
 
Authors: Andrik
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

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
    return Rz @ Ry @ Rx

def gauss_solve(A: np.ndarray, b: np.ndarray) -> np.ndarray:
    """
    Solve Ax = b via Gaussian elimination with partial pivoting. Used in IK updates.

    Partial pivoting selects the largest-magnitude pivot at each step to minimise floating
    point error amplification.

    Returns
    -------
    x : np.ndarray, or
    None: whent he system is singular (|pivot|<1e-14), indiciating a kinematic
    singularity - the robot cannot move in the requested direction from its current configuration.
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

"""
─────────────────────────────────────────────────────────────
ROBOT GEOMETRY  (extracted from URDF)
─────────────────────────────────────────────────────────────
All quantities expressed in the base_link (world-at-rest) frame.

For each leg we need three thigns from the URDF:
    a_i: joint origin xyz in base_link frame
    vhat_i: joint axis in base_link frame
    L_i: link length in base_link frame
    omgea_i: rolling line angle in ground plane (for IK)
--------------------------------------------------------------

How each constant is derived from the URDF:

A_LOCAL  - xyz in base_link frame, from URDF <origin xyz="..."> of each leg joint.
           Direct read: parent is base_link, so no transfrom needed.

U_LOCAL  - motor spin axis in base_link frame.
           The URDF <axis xyz="0 0 1"> is the joint's OWN frame.
           To express it in base_link: u_world = R_rpy @ [0,0,1]
           where R_rpy = rpy_to_R(roll, pitch, yaw) from the URDF <origin rpy="..."> of that joint.

L_LEGS   - Scalar link length = ||xyz|| of each wheel joint origin.
           The wheel <origin xyz = "..."> is relative to the leg frame,
           so its magnitude equals true leg length regardless of frame.

PHI_3LEG - free rolling direction of each wheel, in the ground plane,
           measured CCW from +X (Forward). Derived as:

            spin = R_leg @ R_wheel @ [0,0,1] (wheel spin in world)
            phi = angle of (spin x Z_world) (perpdicular = roll dir)

A_LOCAL  -->  positions (vectors) — must be in base frame  
L_LEGS   -->  scalar lengths — frame doesn't matter        
U_LOCAL  -->  directions (unit vectors) — must be in base frame
            and THIS one actually requires conversion (tricky one))
"""

# -- Leg joint pivot positions in base_link frame (m) -------------------------
A_LOCAL: np.ndarray = np.array([
    [ 0.102780027506839, -0.0812499972928725, -0.0161739485972958],  # fr
    [ 0.000000000000000,  0.120249997292873,  -0.0161739485972948],  # ml
    [-0.102779972493161, -0.0812499972928725, -0.0161739485972958],  # rr
])

# -- Motor spin axes in base_link frame ---------------------------------------
# URDF leg joint: axis = [0,0,1] in joint frame; RPY rotates it to base_link.
#   fr rpy = (-π/2,  0, -π)   →  u_fr ≈ [ 0, -1,  0]
#   ml rpy = (-π/2,  0,  0)   →  u_ml ≈ [ 0, +1,  0]
#   rr rpy = (-π/2,  0, +π)   →  u_rr ≈ [ 0, -1,  0]  (symmetric to fr)
_R_FR = rpy_to_R(-1.5707963267949, 0.0, -3.14159265358979)
_R_ML = rpy_to_R(-1.5707963267949, 0.0,  0.0)
_R_RR = rpy_to_R(-1.5707963267949, 0.0,  3.14159265358979)
 
U_LOCAL: np.ndarray = np.array([
    vhat(_R_FR @ np.array([0.0, 0.0, 1.0])),   # fr motor axis in base_link
    vhat(_R_ML @ np.array([0.0, 0.0, 1.0])),   # ml motor axis in base_link
    vhat(_R_RR @ np.array([0.0, 0.0, 1.0])),   # rr motor axis in base_link
])
 
# -- Link lenghts (motor pivot + wheel contact, meters)
# All three legs are identical; the different xyz components are artefacts
# of different joint frame orientations - Eucledian length is the same for all legs.
_L_FR = np.linalg.norm([0.000000000000000, 0.0610760514027044, 0.0])
_L_ML = np.linalg.norm([0.000000000000000, 0.0610760514027041, 0.0])
_L_RR = np.linalg.norm([0.000000000000000, 0.0610760514027039, 0.0])

L_LEGS = np.array([_L_FR, _L_ML, _L_RR]) # = 0.061076 meters each
                  
# -- Rolling line angles phi_i (radians, CCW from +Z/forward)-----------------
"""
Derived from wheeljoint RPY via:
    spin_world = R_leg @ R_wheel @ [0,0,1]
    phi = arctan2((spin x z)).y, (spin x z).x)

    fr wheel rpy = (0, -pi/4, pi) --> phi_fr = 135 deg
    ml wheel rpy = (0, -pi/2, 0) --> phi_ml = 90 deg
    rr wheel rpy = (0, +pi/4, -pi) --> phi_rr = 45 deg

verifying: np.linalg.matrix_rank(A) == 6 (confirmed, cond ~= 11.9)
"""
PHI_3LEG: np.ndarray = np.array([
    np.radians(135.0),  # fr - forward-left diagonal
    np.radians(90.0),   # ml - lateral (left)
    np.radians(45.0),   # rr - forward-right diagonal
])


# ══════════════════════════════════════════════════════════════════════════════
# JACOBIAN CONTROLLER NODE
# ══════════════════════════════════════════════════════════════════════════════

class JacobianController(Node):
    """
    ROS 2 node ==> holonomic velocity controller for Th-RHex

    At 100 Hz the node:
        1. Reads current joint angles from /joint_states
        2. Builds the 6x6 constraint matrix A and 6x3 coupling matrix B
        3. Solves J = A⁻¹B and inverts to θ̇ = J⁺ · V_desired.
        4. Safety-clamps θ̇  and publishes to the velocity controller.
    """

    # Control loop frequency (Hz)
    _CONTROL_HZ: float = 100.0

    def __init__(self) -> None:
        super().__init__('jacobian_controller')

        # -- ROS 2 Paramters ------------------------------------------------------
        self.declare_parameter('num_legs',3)
        self.declare_parameter('max_joint_velocity', 5.0) # rad/s hard clamp

        self._num_legs = self.get_parameter('num_legs').value
        self._max_vel = self.get_parameter('max_joint_velocity').value

        # -- Active Tripod Configuration ------------------------------------------
        if self._num_legs == 3:
            self._joints = ['fr_leg_joint', 'ml_leg_joint', 'rr_leg_joint']
            self._a_local = A_LOCAL
            self._u_local = U_LOCAL
            self._L       = L_LEGS
            self._phi     = PHI_3LEG
        else:
            # 6-leg mode: extend when second tripod URDF is integrated.

            self.get_logger().warn(
                '6-leg mode not yet implemented; defaulting to 3 legs'
            )

            self._joints = ['fr_leg_joint', 'ml_leg_joint', 'rr_leg_joint']
            self._a_local = A_LOCAL
            self._u_local = U_LOCAL
            self._L       = L_LEGS
            self._phi     = PHI_3LEG

        # -- Node State -----------------------------------------------------------
        # Dictionaries keyed by joint name; initialized to zero.
        self._joint_pos: dict[str, float] = {n: 0.0 for n in self._joints}
        self._joint_vel: dict[str, float] = {n: 0.0 for n in self._joints}

        # Desired 6-DOF body twist V = [Vx, Vy, Vz, ωx, ωy, ωz]
        self._cmd_vel: np.ndarray = np.zeros(6)

        # -- ROS 2 Subscriptions --------------------------------------------------
        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )
        
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_callback,
            10
        )

        # -- ROS 2 Publisher ------------------------------------------------------
        self._vel_pub = self.create_publisher(
            Float64MultiArray,
            '/leg_velocity_controller/commands',
            10
        )

        # -- 100 HZ control loop --------------------------------------------------
        self.create_timer(1.0 / self._CONTROL_HZ, self._control_loop)

        self.get_logger().info(
            f'Jacobian controller ready - {self._num_legs}-leg mode '
            f'| joints: {self._joints} | max_vel: {self._max_vel:.2f} rad/s'
        )

        # -- Subscriber Callbacks -------------------------------------------------

    def _joint_state_callback(self, msg: JointState) -> None:
        """ Cache the latest joint positions and velocites from Gazebo."""
        for i, name in enumerate(msg.name):
            if name in self._joint_pos:
                self._joint_pos[name] = msg.position[i]
                self._joint_vel[name] = msg.velocity[i]

    def _cmd_vel_callback(self, msg: Twist) -> None:
        """
        Map a ROS 2 Twist message to the 6-DOF body twist vector V.

        For ground locomotion vz, wx, wy, are typically zero; the full
        6-DOF interface is kept so the Jacobian math is unmodified 
        """
        self._cmd_vel = np.array([
            msg.linear.x,    # vx  — forward velocity (m/s)
            msg.linear.y,    # vy  — lateral velocity  (m/s)
            msg.linear.z,    # vz  — vertical velocity (m/s) — normally 0
            msg.angular.x,   # ωx  — roll rate  (rad/s) — normally 0
            msg.angular.y,   # ωy  — pitch rate (rad/s) — normally 0
            msg.angular.z,   # ωz  — yaw rate   (rad/s)
        ])

# -- Jacobian Construction ------------------------------------------------------

    def _build_jacobian(self, qa:np.ndarray) -> np.ndarray | None:
        """ 
        Build the 6x3 velocity Jacobian J = A⁻¹B for the current joint angles.

        For each leg i and each perpendicular direction d_i, (j=1,2):

            A row : [dᵢⱼᵀ  |  (rᵢ x dᵢⱼ)ᵀ]        — eq. (22) in reference
            B[row, i] : dᵢⱼ · (ûᵢ x Lᵢ)             — eq. (23) in reference
        
        where rᵢ = aᵢ - p  is the lever arm from the body centroid to joint i,
        and ûᵢ x Lᵢ is the contact-point velocity per unit joint rate (crank).
 
        Parameters
        ----------
        qa: current joint angles (3,) in radians

        Returns
        -------
        J : 6x3 Jacobian matrix, or 
        None if A is singular (kinematic singularity)
        """
        # Body centroid at origin in base_link frame (working frame)
        p = np.zeros(3)
        base_dir = np.array([0.0, 0.0, -1.0]) # link points down at theta=0 (-Z)

        A_rows: list[np.ndarray] = []
        B_rows: list[np.ndarray] = []

        for i in range(3):
            ai = self._a_local[i]   # joint position (from URDF)
            ui = self._u_local[i]   # joint axis (computed from URDF RPY + axis)

            # -- Forward Kinematics --------------------------------------------
            # Rotate the base link direction by by θᵢ about ûᵢ (Rodrigues eq. 4)
            dir_i = rot_aa(ui, qa[i]) @ base_dir
            li    = dir_i * self._L[i]              # link vector aᵢ --> bᵢ  (eq. 5)
            ri    = ai - p                          # lever arm centroid --> joint i 

            # -- Rolling Constraint Directions --------------------------------------------
            # nᵢ is the free-rolling direction: dᵢ₁, dᵢ₂ are its two
            # perpendicular direction (the *blocked* directions)
            ni = np.array([np.cos(self._phi[i]),
                           np.sin(self._phi[i]), 0.0])
            tmp = (np.array([1.0, 0.0, 0.0]) if abs(ni[0]) < 0.9
                   else np.array([0.0, 1.0, 0.0]))
            d1 = vhat(np.cross(ni, tmp))
            d2 = vhat(np.cross(ni, d1))

            # ûᵢ × Lᵢ — tangential velocity of contact point per unit joint rate
            u_cross_l = np.cross(ui, li)

            for d in (d1,d2):
                # A row: body twist --> contact-point veloctiy in blocked direction
                # [dᵢⱼᵀ  |  (rᵢ × dᵢⱼ)ᵀ] -- Eq(22) 

                #   - First 3 elemenents --> how body translation produce
                #     velocity in the blocked direction dᵢⱼ
                #   - Last 3 elements --> how does body rotation via 
                #     the lever arm rᵢ produce velocity in the blocked direction dᵢⱼ 
                A_rows.append(np.concatenate([d, np.cross(ri,d)]))

                # B entry: joint rate --> contact-point velocity in blocked direction
                # Only column i is non-zero (leg i only drives its own contact)
                b    = np.zeros(3)
                b[i] = np.dot(d, u_cross_l)
                B_rows.append(b)

        A = np.array(A_rows) # (6x6)
        B = np.array(B_rows) # (6x3)

        # Singularity guard - A losing rank means holonomic control breaks down
        if np.linalg.matrix_rank(A) < 6:
            self.get_logger().warn(
                'A matrix lost rank - robot is in a singular configuration. '
                'Holding position.'
            )
            return None

        # Solve AJ = B column by column --> J = A⁻¹B
        cols = [gauss_solve(A.copy(), B[:,c]) for c in range(3)]
        if any(c is None for c in cols):
            return None
        
        return np.column_stack(cols) # J: (6,3)

    # -- Control Loop --------------------------------------------------------------------

    def _control_loop(self) -> None:
        """

        100 Hz main loop:
            1. Short-circuit on zero command (hold-position).
            2. Build J from current joint angles.
            3. Compute θ̇ = J⁺ · V_desired via pseudoinverse.
            4. Proportionally clamp to max_joint_velocity.
            5. Publish Float64MultiArray to the velocity controller
        """

        # Zero command -- send zero velocities immediately
        if np.allclose(self._cmd_vel, 0.0, atol=1e-6):
            self._publish(np.zeros(len(self._joints)))
            return
        
        # Current joint angles as ordered array
        qa = np.array([self._joint_pos[n] for n in self._joints])

        # Build Jacobian for current configuration
        J = self._build_jacobian(qa)
        if J is None:
            self._publish(np.zeros(len(self._joints)))
            return
        
        # Invert: θ̇ = J⁺ · V_desired
        # Moore-Penrose pseudoinverse
        # pinv handles near-singular configuration more gracefully than inv.
        dqa = np.linalg.pinv(J) @ self._cmd_vel

        # Proportional velocity clamp - scale all joints together so the
        # commanded direction is perserved even when one joint saturates.
        peak = np.max(np.abs(dqa))
        if peak > self._max_vel:
            dqa *= self._max_vel / peak
        
        self._publish(dqa)
    
    def _publish(self, dqa: np.ndarray) -> None:
        """ Publish joint velocty commands as a Float62MultiArray. """
        msg      = Float64MultiArray()
        msg.data = dqa.tolist()
        self._vel_pub.publish(msg)

# ══════════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ══════════════════════════════════════════════════════════════════════════════
 
def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = JacobianController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

