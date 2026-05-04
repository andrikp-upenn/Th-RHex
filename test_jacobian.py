import numpy as np
import math

# ── math utilities (copied from jacobian_controller.py) ──────────────────────
def vhat(v):
    n = np.linalg.norm(v)
    return v/n if n > 1e-14 else np.zeros(3)

def rot_aa(u, theta):
    x,y,z = vhat(u)
    c,s,t = np.cos(theta), np.sin(theta), 1.0-np.cos(theta)
    return np.array([
        [t*x*x+c,   t*x*y-s*z, t*x*z+s*y],
        [t*x*y+s*z, t*y*y+c,   t*y*z-s*x],
        [t*x*z-s*y, t*y*z+s*x, t*z*z+c  ],
    ])

def rpy_to_R(roll, pitch, yaw):
    return rot_aa([0,0,1],yaw) @ rot_aa([0,1,0],pitch) @ rot_aa([1,0,0],roll)

def gauss_solve(A, b):
    n = len(b)
    M = np.hstack([A.astype(float), b.reshape(-1,1)])
    for c in range(n):
        piv = c + int(np.argmax(np.abs(M[c:,c])))
        M[[c,piv]] = M[[piv,c]]
        if abs(M[c,c]) < 1e-14: return None
        M[c] /= M[c,c]
        for r in range(n):
            if r != c: M[r] -= M[r,c]*M[c]
    return M[:,n]

# ── geometry constants (from jacobian_controller.py) ─────────────────────────
A_LOCAL = np.array([
    [ 0.102780027506839, -0.0812499972928725, -0.0161739485972958],
    [ 0.000000000000000,  0.120249997292873,  -0.0161739485972948],
    [-0.102779972493161, -0.0812499972928725, -0.0161739485972958],
])

_R_FR = rpy_to_R(-1.5707963267949, 0.0, -3.14159265358979)
_R_ML = rpy_to_R(-1.5707963267949, 0.0,  0.0)
_R_RR = rpy_to_R(-1.5707963267949, 0.0,  3.14159265358979)

U_LOCAL = np.array([
    vhat(_R_FR @ np.array([0.0, 0.0, 1.0])),
    vhat(_R_ML @ np.array([0.0, 0.0, 1.0])),
    vhat(_R_RR @ np.array([0.0, 0.0, 1.0])),
])

L_LEGS = np.array([
    np.linalg.norm([0.0, 0.0610760514027044, 0.0]),
    np.linalg.norm([0.0, 0.0610760514027041, 0.0]),
    np.linalg.norm([0.0, 0.0610760514027039, 0.0]),
])

PHI_3LEG = np.array([np.radians(135.0), np.radians(90.0), np.radians(45.0)])

# ── verify U_LOCAL world-frame axes ──────────────────────────────────────────
print("=== U_LOCAL (motor axes in base_link frame) ===")
labels = ['fr','ml','rr']
for i,lbl in enumerate(labels):
    print(f"  u_{lbl} = {U_LOCAL[i].round(6)}")

# ── verify PHI values from wheel joint RPY ───────────────────────────────────
print("\n=== PHI verification from wheel joint RPY ===")
wheel_rpys = [
    (0.0, -0.785398163397443,  3.14159265358979),  # fr_wheel
    (0.0, -1.5707963267949,    0.0),                # ml_wheel
    (0.0,  0.785398163397454, -3.14159265358979),   # rr_wheel
]
leg_Rs = [_R_FR, _R_ML, _R_RR]
for i,(rpy,lbl) in enumerate(zip(wheel_rpys,labels)):
    R_whl = rpy_to_R(*rpy)
    spin_world = vhat(leg_Rs[i] @ R_whl @ np.array([0.0,0.0,1.0]))
    roll_dir = np.cross(spin_world, np.array([0.0,0.0,1.0]))
    roll_dir = vhat(roll_dir)
    phi_computed = np.degrees(np.arctan2(roll_dir[1], roll_dir[0]))
    phi_code     = np.degrees(PHI_3LEG[i])
    match = "✓" if abs(phi_computed - phi_code) < 0.5 or abs(abs(phi_computed-phi_code)-360) < 0.5 else "MISMATCH"
    print(f"  {lbl}: computed phi={phi_computed:.2f}°  code={phi_code:.1f}°  {match}")

# ── build A and B at theta = [0.4, 0.4, 0.4] ─────────────────────────────────
print("\n=== Jacobian at theta=[0.4, 0.4, 0.4] rad ===")
qa = np.array([0.4, 0.4, 0.4])
p = np.zeros(3)
base_dir = np.array([0.0, 0.0, -1.0])
A_rows, B_rows = [], []
for i in range(3):
    ai = A_LOCAL[i]; ui = U_LOCAL[i]
    dir_i = rot_aa(ui, qa[i]) @ base_dir
    li    = dir_i * L_LEGS[i]
    ri    = ai - p
    ni  = np.array([np.cos(PHI_3LEG[i]), np.sin(PHI_3LEG[i]), 0.0])
    tmp = np.array([1.0,0.0,0.0]) if abs(ni[0]) < 0.9 else np.array([0.0,1.0,0.0])
    d1  = vhat(np.cross(ni, tmp))
    d2  = vhat(np.cross(ni, d1))
    u_cross_l = np.cross(ui, li)
    for d in (d1, d2):
        A_rows.append(np.concatenate([d, np.cross(ri, d)]))
        b = np.zeros(3); b[i] = np.dot(d, u_cross_l)
        B_rows.append(b)

A = np.array(A_rows)
B = np.array(B_rows)

rank = np.linalg.matrix_rank(A)
cond = np.linalg.cond(A)
print(f"  rank(A) = {rank}  (want 6)  {'✓' if rank==6 else 'FAIL'}")
print(f"  cond(A) = {cond:.4f}  (want <50)  {'✓' if cond<50 else 'FAIL'}")

# compute J = A^-1 B
cols = [gauss_solve(A.copy(), B[:,c]) for c in range(3)]
if any(c is None for c in cols):
    print("  gauss_solve FAILED (singular)")
else:
    J = np.column_stack(cols)
    print(f"\n  J (6x3) =")
    row_labels = ['vx','vy','vz','wx','wy','wz']
    for r,lbl in enumerate(row_labels):
        print(f"    {lbl}: {J[r].round(6)}")
    near_zero = np.allclose(J[2:5], 0, atol=1e-6)
    not_all_zero = not np.allclose(J, 0)
    print(f"\n  rows vz,wx,wy near zero: {'✓' if near_zero else 'FAIL'}")
    print(f"  J not all zeros:          {'✓' if not_all_zero else 'FAIL'}")
