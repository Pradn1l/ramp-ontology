"""
Stage 2 -- twin sampling / DT association (plan.md Section 4 Stage 2).

Associates the digital-twin CSV (*_dt.csv: joint angles + pose + segment
length along the planned path) with the retained first-layer planned path,
by matching cumulative arc length. The DT csv spans the whole KRL program
(all layers); it is truncated to the same arc-length extent as the retained
first-layer motion blocks (Stage 1), then kinematic descriptors are computed
at each retained DT row via src/kinematics.py.

All quantities produced here are DT-provenance (never mixed with RSI).
"""
import csv
import numpy as np
from dataclasses import dataclass

from kinematics import kinematic_descriptors
from config import JOINT_VEL_MAX_DEG_S


@dataclass
class TwinSamples:
    s_mm: np.ndarray             # cumulative arc length per retained DT row (mm)
    xyz: np.ndarray              # (N,3) pose position from DT csv
    abc: np.ndarray               # (N,3) pose A/B/C from DT csv (deg)
    q_deg: np.ndarray             # (N,6) joint angles (deg)
    kappa: np.ndarray             # Jacobian condition number
    manip: np.ndarray             # manipulability w
    wrist_manip: np.ndarray       # wrist manipulability w_wr
    joint_reserves: np.ndarray    # (N,6) per-joint velocity reserve r_i in [0,1]
    min_reserve: np.ndarray       # (N,) min over joints
    critical_joint: np.ndarray    # (N,) 1-based index of joint attaining min reserve
    q5_deg: np.ndarray            # (N,) wrist joint 5 angle, for near-singularity test


def load_dt_csv(path):
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        rows = list(csv.DictReader(f))
    return rows


def truncate_dt_to_layer(dt_rows, layer_length_mm, tol_mm=2.0):
    """
    The DT csv's own segment_length_mm gives per-row incremental arc length
    (matches the plan's planned polyline construction exactly, since both
    are built from the same KRL targets). Cumulative arc length is already
    provided as cumulative_time_s's positional counterpart via cumsum of
    segment_length_mm; truncate at the point where cumulative arc length
    first reaches the retained first-layer's total length.
    """
    seg = np.array([float(r["segment_length_mm"]) for r in dt_rows])
    cum = np.cumsum(seg)
    # first index where cumulative arc length exceeds the retained layer's
    # total length (with a small tolerance for floating point roundoff)
    cutoff_idx = int(np.searchsorted(cum, layer_length_mm + tol_mm, side="right"))
    cutoff_idx = max(cutoff_idx, 1)
    return dt_rows[:cutoff_idx], cum[:cutoff_idx]


def compute_twin_samples(dt_rows, s_mm) -> TwinSamples:
    n = len(dt_rows)
    xyz = np.array([[float(r["pose_x"]), float(r["pose_y"]), float(r["pose_z"])] for r in dt_rows])
    abc = np.array([[float(r["pose_a"]), float(r["pose_b"]), float(r["pose_c"])] for r in dt_rows])
    q_deg = np.array([[float(r[f"joint_{j}"]) for j in range(1, 7)] for r in dt_rows])

    kappa = np.zeros(n)
    manip = np.zeros(n)
    wrist_manip = np.zeros(n)
    for i in range(n):
        d = kinematic_descriptors(q_deg[i])
        kappa[i] = d["kappa"]
        manip[i] = d["manip"]
        wrist_manip[i] = d["wrist_manip"]

    # per-joint velocity reserve: r_i = 1 - |qdot_i| / qdot_i_max, qdot from
    # central differences of q_deg over arc length converted via the DT's own
    # per-segment time (segment_time_s), giving true angular rate, not an
    # arc-length surrogate.
    seg_time = np.array([float(r["segment_time_s"]) for r in dt_rows])
    qdot = np.zeros_like(q_deg)
    with np.errstate(divide="ignore", invalid="ignore"):
        for j in range(6):
            dq = np.gradient(q_deg[:, j])
            dt = np.gradient(seg_time.cumsum())
            dt[dt == 0] = np.nan
            qdot[:, j] = dq / dt
    qdot = np.nan_to_num(qdot, nan=0.0, posinf=0.0, neginf=0.0)

    reserves = np.zeros_like(qdot)
    for j in range(6):
        qmax = JOINT_VEL_MAX_DEG_S[j + 1]
        reserves[:, j] = 1.0 - np.abs(qdot[:, j]) / qmax
    reserves = np.clip(reserves, -10.0, 1.0)

    min_reserve = reserves.min(axis=1)
    critical_joint = reserves.argmin(axis=1) + 1

    return TwinSamples(
        s_mm=s_mm,
        xyz=xyz,
        abc=abc,
        q_deg=q_deg,
        kappa=kappa,
        manip=manip,
        wrist_manip=wrist_manip,
        joint_reserves=reserves,
        min_reserve=min_reserve,
        critical_joint=critical_joint,
        q5_deg=q_deg[:, 4],
    )


def load_and_associate(dt_csv_path, layer_length_mm) -> TwinSamples:
    dt_rows = load_dt_csv(dt_csv_path)
    dt_rows_trunc, s_mm = truncate_dt_to_layer(dt_rows, layer_length_mm)
    return compute_twin_samples(dt_rows_trunc, s_mm)
