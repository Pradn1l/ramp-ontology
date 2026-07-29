"""
Physical lane -- RSI reconstruction (plan.md Section 3b, Section 4 physical
lane). Loads RSI telemetry, aligns/trims it to the retained first layer, and
performs the monotone closest-point projection of executed samples onto the
planned polyline `a`, needed for all deviation metrics (e_perp, e_theta, dv,
rW). Never mixes RSI and DT quantities in one metric.
"""
import csv
import numpy as np
from dataclasses import dataclass
from scipy.spatial.transform import Rotation

import planned_path as PP
from config import DEVIATION, RSI_SAMPLE_PERIOD_S


@dataclass
class RsiRun:
    t_s: np.ndarray          # time since first retained sample (s)
    xyz: np.ndarray          # (N,3) RSI Cartesian actual position (mm)
    abc: np.ndarray          # (N,3) RSI Cartesian actual orientation A/B/C (deg)
    joints: np.ndarray       # (N,6) RSI actual joint angles (deg)


def load_rsi_dat(path):
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        rows = list(csv.DictReader(f))
    return rows


def find_layer_window(rsi_rows, path: PP.PlannedPath, z_tol_mm=0.5, start_tol_mm=3.0):
    """
    Alignment rule (plan.md 3b): find the first print point (XYZ of the first
    deposition target) in the RSI stream; that sample starts the executed
    trajectory. Trim everything before it (approach moves) and everything
    after the layer's last print point.

    Start: the FIRST RSI sample (in file order) that comes within
    start_tol_mm of the first planned target -- i.e. the first local
    approach to that point, not the closest point anywhere in the whole
    multi-layer file. A raw global argmin over the whole RSI stream can
    accidentally match a later layer's pass through similar XY(Z) coordinates
    more closely than the true first-layer entry (observed on 'custom', whose
    printed footprint repeats across 11 stacked layers); restricting to the
    first crossing of a generous tolerance ball avoids that trap while still
    landing within a sub-mm/low-mm neighborhood of the true start.

    End: for planar multi-layer files (Z constant within the retained layer),
    the first sample after start whose Z leaves the retained layer's Z band
    by more than z_tol_mm (next layer begins). For non-planar single-layer
    files (Z not constant -- the whole .src IS one layer, so there is no
    next layer in this RSI file at all), the end is the sample closest to the
    last planned target, searched only after `start_idx + min_samples` to
    avoid matching the coincident start/end target of a closed loop too
    early.
    """
    xyz = np.array([[float(r["X"]), float(r["Y"]), float(r["Z"])] for r in rsi_rows])
    target0 = path.xyz_targets[0]
    d0 = np.linalg.norm(xyz - target0, axis=1)
    # require BOTH the full 3D distance and the Z-component alone within
    # tolerance, so a vertical descent that is still far in Z (but happens to
    # pass near the target's XY column) is not mistaken for arrival.
    z_close = np.abs(xyz[:, 2] - target0[2]) < z_tol_mm
    within_tol = np.where((d0 < start_tol_mm) & z_close)[0]
    start_idx = int(within_tol.min()) if len(within_tol) else int(d0.argmin())

    z0 = path.xyz_targets[0, 2]
    z_is_const = np.allclose(path.xyz_targets[:, 2], z0, atol=z_tol_mm)

    if z_is_const:
        z_after = xyz[start_idx:, 2]
        leave_mask = np.abs(z_after - z0) > z_tol_mm
        idxs = np.where(leave_mask)[0]
        end_idx = start_idx + (idxs.min() if len(idxs) else len(z_after) - 1)
    else:
        target_last = path.xyz_targets[-1]
        # search only in a window starting after enough arc length has
        # plausibly been traveled, to avoid the coincident-start-point trap
        # on closed loops; use a generous minimum sample count based on RSI
        # rate and the commanded speed of the last block.
        approx_total_time_s = path.total_length_mm / max(
            path.commanded_vel_cp_mps[path.commanded_vel_cp_mps > 0].mean() * 1000.0, 1.0
        )
        min_samples = int(0.5 * approx_total_time_s / RSI_SAMPLE_PERIOD_S)
        search_from = start_idx + max(min_samples, 1)
        search_from = min(search_from, len(xyz) - 1)
        d_last = np.linalg.norm(xyz[search_from:] - target_last, axis=1)
        end_idx = search_from + int(d_last.argmin())

    return start_idx, end_idx


def build_rsi_run(rsi_rows, start_idx, end_idx) -> RsiRun:
    rows = rsi_rows[start_idx:end_idx + 1]
    xyz = np.array([[float(r["X"]), float(r["Y"]), float(r["Z"])] for r in rows])
    abc = np.array([[float(r["A"]), float(r["B"]), float(r["C"])] for r in rows])
    joints = np.array([[float(r[f"AxisAct{j}"]) for j in range(1, 7)] for r in rows])
    n = len(rows)
    t_s = np.arange(n) * RSI_SAMPLE_PERIOD_S
    return RsiRun(t_s=t_s, xyz=xyz, abc=abc, joints=joints)


DEFAULT_LOOKAHEAD_SEGS = 25
# Bounding the forward search window is not just a speed optimization: an
# unbounded (or overly large) window lets the very first sample match a
# segment anywhere in the whole path, including a later occurrence of similar
# geometry (e.g. 'custom' has a duplicated first target and a footprint that
# re-approaches nearby XY(Z) coordinates later on) -- observed to jump the
# entire monotone projection to s~=total_length on sample 0 and get stuck
# there (e_perp inflated to hundreds of mm). A window of 25 segments fixed
# this on all four programs (verified against a range 5-50 with identical
# results) while remaining far larger than the largest plausible per-sample
# arc-length advance at 4 ms RSI rate and these commanded speeds.


def monotone_closest_point_projection(run: RsiRun, path: PP.PlannedPath, lookahead_segs=DEFAULT_LOOKAHEAD_SEGS):
    """
    Projects each executed sample onto the piecewise-linear planned polyline
    `a`, enforcing a monotonically non-decreasing arc-length association
    (never allowed to run backwards), per plan.md 3b. Forward-only
    nearest-segment search: for each sample, only segments at or after the
    previous sample's matched segment are considered (vectorized over the
    segment axis with numpy instead of a per-segment Python loop).

    lookahead_segs bounds how many segments ahead of the current match are
    searched per sample (None = search to the end of the path). A bounded
    window is a large speedup on long paths (e.g. cladding: 360 segments x
    ~29k samples) and is safe because RSI sampling density along a slowly
    executed path means the true match rarely jumps far ahead between
    consecutive 4 ms samples; the window defaults to the full path when
    lookahead_segs is None to guarantee correctness first.

    Returns: s_proj (arc length of the projected point on `a`, mm),
    e_perp (lateral distance from sample to its projection, mm).
    """
    n = len(run.xyz)
    s_proj = np.zeros(n)
    e_perp = np.zeros(n)

    seg_starts = path.xyz_targets[:-1]
    seg_vecs = np.diff(path.xyz_targets, axis=0)
    seg_lens = np.linalg.norm(seg_vecs, axis=1)
    seg_lens_safe = np.where(seg_lens > 1e-9, seg_lens, 1.0)
    seg_len_sq = seg_lens_safe ** 2
    n_segs = len(seg_starts)

    cur_seg = 0
    prev_s = 0.0
    for i in range(n):
        p = run.xyz[i]
        j_hi = n_segs if lookahead_segs is None else min(n_segs, cur_seg + lookahead_segs)
        w = p - seg_starts[cur_seg:j_hi]                       # (k,3)
        t = np.einsum("ij,ij->i", w, seg_vecs[cur_seg:j_hi]) / seg_len_sq[cur_seg:j_hi]
        t_clamped = np.clip(t, 0.0, 1.0)
        proj = seg_starts[cur_seg:j_hi] + t_clamped[:, None] * seg_vecs[cur_seg:j_hi]
        d2 = np.sum((p - proj) ** 2, axis=1)

        best_rel = int(np.argmin(d2))
        best_seg = cur_seg + best_rel
        best_s = path.s_targets[best_seg] + t_clamped[best_rel] * seg_lens[best_seg]
        best_d2 = d2[best_rel]

        s_proj[i] = max(best_s, prev_s)
        e_perp[i] = np.sqrt(best_d2)
        prev_s = s_proj[i]
        cur_seg = best_seg

    return s_proj, e_perp
