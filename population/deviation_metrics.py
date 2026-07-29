"""
Physical-lane deviation metrics (plan.md Section 1 deviation metrics,
Section 4 physical lane). Computed on the RSI-measured executed path `c`
relative to the planned path `a`, via the monotone closest-point projection
already computed in rsi_reconstruction.py.

- e_perp: lateral (position) deviation -- already produced by the projection.
- e_theta: geodesic orientation deviation between executed and planned
  orientation at the same projected arc length.
- delta_v: relative TCP speed dip, from differentiating logged RSI positions.
- rW: exit-ringing RMS over a window following a feature.

Deviation within a declared BlendZone up to the commanded tolerance is
designed, not infidelity: samples whose projected arc length falls inside a
BlendZone AND whose e_perp does not exceed the zone's blend distance are
excluded from "infidelity" aggregates (but retained in the raw per-sample
series for plotting).
"""
import numpy as np
from scipy.spatial.transform import Rotation

import planned_path as PP
from config import THETA, DEVIATION, RSI_SAMPLE_PERIOD_S


def orientation_deviation_deg(run_abc: np.ndarray, path: PP.PlannedPath, s_proj: np.ndarray) -> np.ndarray:
    """Geodesic angle between executed orientation (RSI A/B/C) and planned
    orientation interpolated (SO(3) slerp) at the projected arc length."""
    r_exec = Rotation.from_euler("ZYX", run_abc, degrees=True)
    r_plan = PP.orientation_at_s(path, s_proj)
    rel = r_plan.inv() * r_exec
    return rel.magnitude() * 180.0 / np.pi


def tcp_speed_mm_s(run_xyz: np.ndarray, dt_s: float = RSI_SAMPLE_PERIOD_S) -> np.ndarray:
    """TCP speed by central-difference differentiation of logged RSI
    positions (plan.md: 'TCP speed obtained by differentiating logged RSI
    positions')."""
    v = np.zeros(len(run_xyz))
    if len(run_xyz) < 2:
        return v
    diffs = np.linalg.norm(np.diff(run_xyz, axis=0), axis=1) / dt_s
    v[0] = diffs[0]
    v[-1] = diffs[-1]
    v[1:-1] = 0.5 * (diffs[:-1] + diffs[1:])
    return v


def speed_dip_pct(v_exec_mm_s: np.ndarray, v_cmd_mm_s: np.ndarray, smooth_window=None) -> np.ndarray:
    """delta_v = (v_cmd - ||pdot_c||) / v_cmd per sample. Positive = dip below
    commanded; smoothed with a simple moving average to suppress 4 ms sample
    noise before reporting (per config.DEVIATION.speed_smoothing_window_samples)."""
    if smooth_window is None:
        smooth_window = DEVIATION["speed_smoothing_window_samples"]
    if smooth_window > 1:
        kernel = np.ones(smooth_window) / smooth_window
        v_smooth = np.convolve(v_exec_mm_s, kernel, mode="same")
    else:
        v_smooth = v_exec_mm_s
    with np.errstate(divide="ignore", invalid="ignore"):
        dv = (v_cmd_mm_s - v_smooth) / v_cmd_mm_s
    return np.nan_to_num(dv, nan=0.0, posinf=0.0, neginf=0.0) * 100.0


def in_blend_zone_within_tolerance(s_proj, e_perp, blend_zones):
    """
    Returns a boolean mask (len == len(s_proj)), always all-False in this
    corpus.

    plan.md's designed-tolerance carve-out ("deviation within a declared
    approximation zone up to the commanded tolerance is designed and must
    NOT be counted as infidelity") applies to $APO.CDIS-governed (C_DIS)
    blending, where CDIS is the actual geometric switch-over radius the
    controller targets. Every motion block in this data package uses C_VEL
    (or C_SPL) blending instead -- a velocity-criterion switch -- so
    $APO.CDIS is present in the header but is not the parameter actually
    governing the realized blend geometry here (confirmed: only one of
    C_DIS/C_VEL/C_ORI is ever active per block, and this corpus exclusively
    uses C_VEL/C_SPL). Exempting deviation via a CDIS distance the controller
    was never asked to honor would misclassify genuine infidelity as
    "designed" -- so no exemption is applied, and BlendZone/blendDistanceMm
    are retained in the ontology/KG purely as descriptive plan-spine data.
    """
    return np.zeros(len(s_proj), dtype=bool)


def exit_ringing_rms(e_perp: np.ndarray, s_proj: np.ndarray, feature_end_s_mm: float,
                      window_mm: float = None) -> float:
    """rW: lateral-deviation RMS over the arc-length window following a
    feature (e.g. a flagged region's end), per plan.md 'exit-ringing RMS over
    a window following a feature'."""
    if window_mm is None:
        window_mm = THETA["ringing_window_mm"]
    mask = (s_proj >= feature_end_s_mm) & (s_proj <= feature_end_s_mm + window_mm)
    if not mask.any():
        return float("nan")
    return float(np.sqrt(np.mean(e_perp[mask] ** 2)))


def aggregate_region_deviation(region, s_proj, e_perp, e_theta, dv_pct, designed_tolerance_mask):
    """
    Per-region aggregates for a DeviationObservation.

    designed_tolerance_mask: boolean array from in_blend_zone_within_tolerance
    -- True where a sample's deviation is fully explained by a declared blend
    tolerance (designed, not infidelity).

    lateral RMS/max and orientation max are computed over "counted" samples,
    i.e. span samples NOT covered by designed_tolerance_mask. If every sample
    in the region's span is within its designed tolerance, the region stayed
    within tolerance end to end; counted falls back to the full span so RMS
    is still reported (a "how close to the edge did it get" number), and
    outside_designed_tolerance is set False to record that fact.
    Returns None if no RSI samples project into the region's span.
    """
    span_mask = (s_proj >= region.start_s_mm) & (s_proj <= region.end_s_mm)
    if not span_mask.any():
        return None
    exceeded_tolerance = span_mask & (~designed_tolerance_mask)
    outside_designed_tolerance = bool(exceeded_tolerance.any())
    counted = exceeded_tolerance if outside_designed_tolerance else span_mask

    lateral_rms = float(np.sqrt(np.mean(e_perp[counted] ** 2)))
    lateral_max = float(e_perp[counted].max())
    orientation_max = float(e_theta[counted].max())
    speed_dip = float(np.max(dv_pct[span_mask]))
    rW = exit_ringing_rms(e_perp, s_proj, region.end_s_mm)

    return dict(
        lateral_rms_mm=lateral_rms,
        lateral_max_mm=lateral_max,
        orientation_max_deg=orientation_max,
        speed_dip_pct=speed_dip,
        speed_dip_duration_ms=None,
        exit_ringing_rms_mm=rW,
        sampling_rate_hz=250.0,
        outside_designed_tolerance=outside_designed_tolerance,
    )
