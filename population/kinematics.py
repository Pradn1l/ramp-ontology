"""
KUKA KR 30 HA kinematics: Modified DH forward kinematics and geometric
Jacobian, used to derive DT-provenance descriptors (condition number,
manipulability, wrist manipulability, joint velocity reserves) from the
joint angles already present in *_dt.csv, per plan.md Section 3(c).

DH parameters from dhm_parameters.txt (RoboDK export, Modified DH convention).
"""
import numpy as np

# Joint, alpha_deg, a_mm, theta_offset_deg, d_mm
DHM_TABLE = [
    (1, 0.0, 0.0, 0.0, 815.0),
    (2, -90.0, 350.0, 0.0, 0.0),
    (3, 0.0, 850.0, -90.0, 0.0),
    (4, -90.0, 145.0, 0.0, 820.0),
    (5, 90.0, 0.0, 0.0, 0.0),
    (6, -90.0, 0.0, 180.0, 170.0),
]


def _mdh_transform(alpha, a, theta, d):
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([
        [ct, -st, 0.0, a],
        [st * ca, ct * ca, -sa, -sa * d],
        [st * sa, ct * sa, ca, ca * d],
        [0.0, 0.0, 0.0, 1.0],
    ])


def forward_kinematics(q_deg):
    """
    q_deg: iterable of 6 joint angles in degrees (robot convention, as stored
    in *_dt.csv joint_1..joint_6).
    Returns: list of 7 homogeneous transforms T_0_0..T_0_6 (base to joint i).
    """
    q = np.deg2rad(np.asarray(q_deg, dtype=float))
    T = np.eye(4)
    frames = [T.copy()]
    for (j, alpha_deg, a, theta_off_deg, d) in DHM_TABLE:
        alpha = np.deg2rad(alpha_deg)
        theta = q[j - 1] + np.deg2rad(theta_off_deg)
        Ti = _mdh_transform(alpha, a, theta, d)
        T = T @ Ti
        frames.append(T.copy())
    return frames


def geometric_jacobian(q_deg):
    """
    6x6 geometric Jacobian (linear over angular rows) at joint angles q_deg,
    for a chain of 6 revolute joints in this Modified DH table.

    Column i uses the z-axis/origin of frames[i+1] (the frame produced by
    applying DH row i, i.e. joint i+1's own row), not frames[i]. Validated
    against a central-difference numerical Jacobian to ~1e-7 over random
    configurations; using frames[i] instead reproduces a spurious rank
    deficiency (see kinematics validation note).
    """
    frames = forward_kinematics(q_deg)
    o_n = frames[-1][:3, 3]
    J = np.zeros((6, 6))
    for i in range(6):
        z_i = frames[i + 1][:3, 2]
        o_i = frames[i + 1][:3, 3]
        Jv = np.cross(z_i, o_n - o_i)
        Jw = z_i
        J[:3, i] = Jv
        J[3:, i] = Jw
    J[:3, :] /= 1000.0  # mm -> m: puts the linear block on a comparable
    # numeric scale to the (unitless-direction) angular block, so condition
    # number and manipulability are not artifacts of the length unit choice.
    return J


def condition_number(J):
    sv = np.linalg.svd(J, compute_uv=False)
    sv = sv[sv > 1e-12]
    if len(sv) == 0:
        return np.inf
    return float(sv.max() / sv.min())


def manipulability(J):
    """Yoshikawa manipulability w = sqrt(det(J J^T))."""
    JJt = J @ J.T
    det = np.linalg.det(JJt)
    return float(np.sqrt(max(det, 0.0)))


def wrist_manipulability(J):
    """Manipulability of the angular (wrist, joints 4-6 contribution) rows only."""
    Jw = J[3:, :]
    JJt = Jw @ Jw.T
    det = np.linalg.det(JJt)
    return float(np.sqrt(max(det, 0.0)))


def kinematic_descriptors(q_deg):
    J = geometric_jacobian(q_deg)
    return dict(
        kappa=condition_number(J),
        manip=manipulability(J),
        wrist_manip=wrist_manipulability(J),
        J=J,
    )
