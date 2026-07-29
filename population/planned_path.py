"""
Planned path `a` geometry (plan.md Section 1: three-element decomposition,
element a) and Stage 2 -- twin sampling / DT association (plan.md Section 4).

Path a: position linearly interpolated between consecutive motion-block
targets; orientation along the SO(3) geodesic (built from KUKA A/B/C Euler
angles -> rotation matrices -> quaternion slerp); TCP speed nominally
constant at the commanded value on each segment.

Arc length s is measured along the position polyline in mm, s=0 at the first
retained block's target (per-program local origin).
"""
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from dataclasses import dataclass


def euler_kuka_to_rotation(a_deg, b_deg, c_deg):
    """
    KUKA A/B/C Euler angles -> Rotation. KUKA convention: intrinsic Z-Y-X
    (yaw=A about Z, pitch=B about Y, roll=C about X), i.e. R = Rz(A) Ry(B) Rx(C).
    """
    return Rotation.from_euler("ZYX", [a_deg, b_deg, c_deg], degrees=True)


@dataclass
class PlannedPath:
    s_targets: np.ndarray          # arc length at each block target, shape (N,)
    xyz_targets: np.ndarray        # shape (N,3)
    rotations: Rotation             # length-N Rotation stack, one per target
    commanded_vel_cp_mps: np.ndarray  # shape (N,) $VEL.CP in force at each block (m/s)
    blend_switch: list              # length N, blend switch string or None
    blocks: list                    # original MotionBlock list (same order)
    total_length_mm: float


def build_planned_path(blocks) -> PlannedPath:
    xyz = np.array([[b.x, b.y, b.z] for b in blocks], dtype=float)
    seg_lengths = np.zeros(len(blocks))
    seg_lengths[1:] = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    s = np.cumsum(seg_lengths)

    rots = Rotation.from_euler(
        "ZYX",
        np.array([[b.a, b.b, b.c] for b in blocks], dtype=float),
        degrees=True,
    )
    vel_cp = np.array([b.commanded_vel_cp for b in blocks], dtype=float)
    blend = [b.blend_switch for b in blocks]

    return PlannedPath(
        s_targets=s,
        xyz_targets=xyz,
        rotations=rots,
        commanded_vel_cp_mps=vel_cp,
        blend_switch=blend,
        blocks=blocks,
        total_length_mm=float(s[-1]) if len(s) else 0.0,
    )


def position_at_s(path: PlannedPath, s_query: np.ndarray) -> np.ndarray:
    """Piecewise-linear position interpolation along arc length."""
    s_query = np.atleast_1d(s_query)
    out = np.zeros((len(s_query), 3))
    for k in range(3):
        out[:, k] = np.interp(s_query, path.s_targets, path.xyz_targets[:, k])
    return out


def orientation_at_s(path: PlannedPath, s_query) -> Rotation:
    """SO(3) geodesic (slerp) orientation interpolation along arc length."""
    s_query = np.atleast_1d(s_query).astype(float)
    s_unique, idx_unique = np.unique(path.s_targets, return_index=True)
    if len(s_unique) < 2:
        return Rotation.concatenate([path.rotations[idx_unique[0]]] * len(s_query))
    slerp = Slerp(s_unique, path.rotations[idx_unique])
    s_clamped = np.clip(s_query, s_unique[0], s_unique[-1])
    return slerp(s_clamped)


def commanded_speed_at_s(path: PlannedPath, s_query) -> np.ndarray:
    """Piecewise-constant commanded TCP speed: the block's own $VEL.CP governs
    the segment ENDING at that block's target (KRL semantics: the speed set
    before a motion line applies to that line's motion)."""
    s_query = np.atleast_1d(s_query)
    out = np.full(len(s_query), path.commanded_vel_cp_mps[0])
    for i in range(1, len(path.s_targets)):
        mask = (s_query > path.s_targets[i - 1]) & (s_query <= path.s_targets[i])
        out[mask] = path.commanded_vel_cp_mps[i]
    return out


def angular_twist_rate(path: PlannedPath, s_query, ds=0.5) -> np.ndarray:
    """
    rho(s) = ||omega_a(s)|| / ||pdot_a(s)|| in deg/mm (plan.md Section 1
    region taxonomy). Central difference in arc length on the orientation
    geodesic and position polyline.
    """
    s_query = np.atleast_1d(s_query).astype(float)
    s_lo = np.clip(s_query - ds / 2, path.s_targets[0], path.s_targets[-1])
    s_hi = np.clip(s_query + ds / 2, path.s_targets[0], path.s_targets[-1])
    denom = np.maximum(s_hi - s_lo, 1e-9)

    r_lo = orientation_at_s(path, s_lo)
    r_hi = orientation_at_s(path, s_hi)
    rel = r_lo.inv() * r_hi
    angle_deg = rel.magnitude() * 180.0 / np.pi

    p_lo = position_at_s(path, s_lo)
    p_hi = position_at_s(path, s_hi)
    dpos = np.linalg.norm(p_hi - p_lo, axis=1)
    dpos = np.maximum(dpos, 1e-9)

    return angle_deg / dpos
