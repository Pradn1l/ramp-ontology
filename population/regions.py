"""
Stage 3 -- region taggers (plan.md Section 1 region taxonomy, Section 4
Stage 3). Applies threshold set Theta (config.THETA) to the planned path +
twin samples to produce Region individuals. Subclasses are NOT disjoint by
design; a given arc-length interval may carry several tags.

Each Region carries: tag (subclass), start/end arc length, and (for
KinematicProfile) the DT-provenance descriptors aggregated over the interval.
"""
import numpy as np
from dataclasses import dataclass, field
from typing import Optional, List

from config import THETA
import planned_path as PP


@dataclass
class Region:
    tag: str                       # CornerRegion | SharpReorientationRegion | ...
    start_s_mm: float
    end_s_mm: float
    block_index_at_vertex: Optional[int] = None  # for corner/blend zones
    corner_angle_deg: Optional[float] = None
    angular_twist_rate_deg_per_mm: Optional[float] = None
    min_planned_radius_mm: Optional[float] = None
    kappa_max: Optional[float] = None
    manip_min: Optional[float] = None
    wrist_manip_min: Optional[float] = None
    min_joint_reserve: Optional[float] = None
    critical_joint: Optional[int] = None
    blend_distance_mm: Optional[float] = None  # for BlendZone


MIN_SEGMENT_LENGTH_MM = 0.5
# KRL programs in this corpus emit near-duplicate consecutive targets around
# a $VEL.CP change (see e.g. custom_l1_v200_s1_00000.src lines 3-4: the same
# X/Y/Z repeated with only the commanded speed changed). A segment shorter
# than this is treated as a non-move for direction/orientation-change
# purposes: its direction is dominated by floating-point/print noise, not a
# real geometric feature, and using it directly as v_in/v_out at the
# neighboring vertex corrupts that vertex's corner test (observed on P02,
# where the true ~59 deg corner was smeared to ~0.1 deg because one of its
# two neighbor targets was a 0.94 mm near-duplicate of the vertex itself).


def _find_prev_distinct(path: PP.PlannedPath, i: int, min_len=MIN_SEGMENT_LENGTH_MM) -> int:
    """Walk backward from i to the nearest earlier target at least min_len
    away from targets[i] (skipping near-duplicate points). Returns i if none
    found (start of path)."""
    j = i - 1
    while j >= 0 and np.linalg.norm(path.xyz_targets[i] - path.xyz_targets[j]) < min_len:
        j -= 1
    return max(j, 0)


def _find_next_distinct(path: PP.PlannedPath, i: int, min_len=MIN_SEGMENT_LENGTH_MM) -> int:
    """Walk forward from i to the nearest later target at least min_len away
    from targets[i] (skipping near-duplicate points). Returns i if none found
    (end of path)."""
    n = len(path.xyz_targets)
    j = i + 1
    while j < n and np.linalg.norm(path.xyz_targets[i] - path.xyz_targets[j]) < min_len:
        j += 1
    return min(j, n - 1)


def _vertex_direction_change_deg(path: PP.PlannedPath, i: int) -> float:
    """Angle between incoming and outgoing translation direction at target i,
    using the nearest genuinely-displaced neighbor on each side (skipping
    near-duplicate/zero-length segments -- see MIN_SEGMENT_LENGTH_MM)."""
    if i <= 0 or i >= len(path.xyz_targets) - 1:
        return 0.0
    j_prev = _find_prev_distinct(path, i)
    j_next = _find_next_distinct(path, i)
    if j_prev == i or j_next == i:
        return 0.0
    v_in = path.xyz_targets[i] - path.xyz_targets[j_prev]
    v_out = path.xyz_targets[j_next] - path.xyz_targets[i]
    n_in, n_out = np.linalg.norm(v_in), np.linalg.norm(v_out)
    if n_in < 1e-6 or n_out < 1e-6:
        return 0.0
    cos_ang = np.clip(np.dot(v_in, v_out) / (n_in * n_out), -1.0, 1.0)
    return float(np.degrees(np.arccos(cos_ang)))


def _vertex_orientation_change_deg(path: PP.PlannedPath, i: int) -> float:
    """Geodesic orientation change across the same distinct-neighbor window
    used by _vertex_direction_change_deg, for consistency (a near-duplicate
    target should not corrupt the orientation-change test either)."""
    if i <= 0 or i >= len(path.rotations) - 1:
        return 0.0
    j_prev = _find_prev_distinct(path, i)
    j_next = _find_next_distinct(path, i)
    if j_prev == i or j_next == i:
        return 0.0
    rel = path.rotations[j_prev].inv() * path.rotations[j_next]
    return float(np.degrees(rel.magnitude()))


def _local_radius_mm(path: PP.PlannedPath, i: int) -> float:
    """
    Local contour radius at vertex i, estimated from the circle fit through
    the nearest genuinely-displaced neighbor on each side (skipping
    near-duplicate targets, same rule as _vertex_direction_change_deg) and i
    itself (menger curvature -> radius = 1/curvature). Returns +inf for
    near-collinear points (radius undefined / very large).
    """
    if i <= 0 or i >= len(path.xyz_targets) - 1:
        return np.inf
    j_prev = _find_prev_distinct(path, i)
    j_next = _find_next_distinct(path, i)
    if j_prev == i or j_next == i:
        return np.inf
    p0, p1, p2 = path.xyz_targets[j_prev], path.xyz_targets[i], path.xyz_targets[j_next]
    a = np.linalg.norm(p1 - p0)
    b = np.linalg.norm(p2 - p1)
    c = np.linalg.norm(p2 - p0)
    if a < 1e-6 or b < 1e-6 or c < 1e-6:
        return np.inf
    area2 = np.linalg.norm(np.cross(p1 - p0, p2 - p0))
    if area2 < 1e-9:
        return np.inf
    radius = (a * b * c) / (2 * area2)
    return float(radius)


def tag_corner_and_sharp_reorientation_regions(path: PP.PlannedPath) -> List[Region]:
    regions = []
    n = len(path.xyz_targets)
    for i in range(1, n - 1):
        dir_change = _vertex_direction_change_deg(path, i)
        orient_change = _vertex_orientation_change_deg(path, i)
        j_prev = _find_prev_distinct(path, i)
        j_next = _find_next_distinct(path, i)
        seg_in_len = path.s_targets[i] - path.s_targets[j_prev]
        seg_out_len = path.s_targets[j_next] - path.s_targets[i]
        window = 0.5 * min(seg_in_len, seg_out_len) if min(seg_in_len, seg_out_len) > 0 else 0.0
        s0, s1 = path.s_targets[i] - window, path.s_targets[i] + window

        if (dir_change >= THETA["corner_angle_deg_c"]
                and orient_change <= THETA["corner_max_orient_change_deg"]):
            regions.append(Region(
                tag="CornerRegion", start_s_mm=s0, end_s_mm=s1,
                block_index_at_vertex=i, corner_angle_deg=dir_change,
            ))

        translation = np.linalg.norm(path.xyz_targets[j_next] - path.xyz_targets[j_prev])
        if translation > 1e-6:
            twist_rate = orient_change / translation
        else:
            twist_rate = np.inf if orient_change > 0 else 0.0
        if twist_rate >= THETA["twist_rate_c_deg_per_mm"]:
            regions.append(Region(
                tag="SharpReorientationRegion", start_s_mm=s0, end_s_mm=s1,
                block_index_at_vertex=i, angular_twist_rate_deg_per_mm=twist_rate,
            ))
    return regions


def tag_subnozzle_feature_regions(path: PP.PlannedPath) -> List[Region]:
    regions = []
    n = len(path.xyz_targets)
    for i in range(1, n - 1):
        r = _local_radius_mm(path, i)
        if r < THETA["min_feasible_radius_mm"]:
            seg_in_len = path.s_targets[i] - path.s_targets[i - 1]
            seg_out_len = path.s_targets[i + 1] - path.s_targets[i]
            window = 0.5 * min(seg_in_len, seg_out_len) if min(seg_in_len, seg_out_len) > 0 else 0.0
            regions.append(Region(
                tag="SubNozzleFeatureRegion",
                start_s_mm=path.s_targets[i] - window,
                end_s_mm=path.s_targets[i] + window,
                block_index_at_vertex=i, min_planned_radius_mm=r,
            ))
    return regions


def tag_blend_zones(path: PP.PlannedPath, header) -> List[Region]:
    """
    BlendZone: designed approximation interval at each interior vertex whose
    outgoing block carries a C_VEL/C_SPL blend switch, extent = active
    $APO.CDIS (mm), symmetric around the vertex, clipped to not exceed half
    of either adjacent segment (cannot overlap beyond the segment itself).
    """
    regions = []
    n = len(path.blocks)
    cdis = header.apo_cdis_mm if header.apo_cdis_mm is not None else 0.0
    for i in range(1, n - 1):
        if path.blend_switch[i] not in ("C_VEL", "C_SPL"):
            continue
        seg_in_len = path.s_targets[i] - path.s_targets[i - 1]
        seg_out_len = path.s_targets[i + 1] - path.s_targets[i]
        half_extent = min(cdis, seg_in_len / 2 if seg_in_len > 0 else cdis,
                           seg_out_len / 2 if seg_out_len > 0 else cdis)
        regions.append(Region(
            tag="BlendZone",
            start_s_mm=path.s_targets[i] - half_extent,
            end_s_mm=path.s_targets[i] + half_extent,
            block_index_at_vertex=i, blend_distance_mm=cdis,
        ))
    return regions


def tag_near_singularity_and_velocity_limited_regions(path: PP.PlannedPath, twin) -> List[Region]:
    """
    Contiguous-run tagging over the DT-sampled arc length: a NearSingularity
    (resp. VelocityLimited) region is a maximal contiguous run of DT samples
    satisfying the threshold test, reported as one Region spanning that run's
    arc-length extent with aggregated (max kappa / min manip / min reserve)
    descriptors.
    """
    regions = []
    s = twin.s_mm
    n = len(s)

    near_sing_mask = (np.abs(twin.q5_deg) < THETA["q5_c_deg"]) | (twin.kappa >= THETA["kappa_c"])
    vel_limited_mask = twin.min_reserve < THETA["joint_reserve_rc"]

    def runs_from_mask(mask):
        runs = []
        i = 0
        while i < n:
            if mask[i]:
                j = i
                while j + 1 < n and mask[j + 1]:
                    j += 1
                runs.append((i, j))
                i = j + 1
            else:
                i += 1
        return runs

    for (i0, i1) in runs_from_mask(near_sing_mask):
        regions.append(Region(
            tag="NearSingularityRegion",
            start_s_mm=float(s[i0]), end_s_mm=float(s[i1]),
            kappa_max=float(twin.kappa[i0:i1 + 1].max()),
            manip_min=float(twin.manip[i0:i1 + 1].min()),
            wrist_manip_min=float(twin.wrist_manip[i0:i1 + 1].min()),
        ))

    for (i0, i1) in runs_from_mask(vel_limited_mask):
        seg = twin.joint_reserves[i0:i1 + 1]
        min_r = seg.min()
        crit_j = int(twin.critical_joint[i0:i1 + 1][seg.min(axis=1).argmin()])
        regions.append(Region(
            tag="VelocityLimitedRegion",
            start_s_mm=float(s[i0]), end_s_mm=float(s[i1]),
            min_joint_reserve=float(min_r), critical_joint=crit_j,
        ))

    return regions


def attach_kinematic_profile(region: Region, path: PP.PlannedPath, twin) -> None:
    """Fill in DT-provenance kinematic descriptors over a region's arc-length
    span for regions that don't already carry them (Corner/SharpReorientation/
    SubNozzleFeature/BlendZone), by aggregating twin samples in-span."""
    if region.kappa_max is not None:
        return  # already has kinematic descriptors (NearSingularity/VelocityLimited)
    mask = (twin.s_mm >= region.start_s_mm) & (twin.s_mm <= region.end_s_mm)
    if not mask.any():
        # fall back to nearest sample
        idx = int(np.argmin(np.abs(twin.s_mm - 0.5 * (region.start_s_mm + region.end_s_mm))))
        mask = np.zeros_like(twin.s_mm, dtype=bool)
        mask[idx] = True
    region.kappa_max = float(twin.kappa[mask].max())
    region.manip_min = float(twin.manip[mask].min())
    region.wrist_manip_min = float(twin.wrist_manip[mask].min())
    region.min_joint_reserve = float(twin.joint_reserves[mask].min())
    crit_idx = np.where(mask)[0][twin.joint_reserves[mask].min(axis=1).argmin()]
    region.critical_joint = int(twin.critical_joint[crit_idx])


def tag_all_regions(path: PP.PlannedPath, header, twin) -> List[Region]:
    regions = []
    regions += tag_corner_and_sharp_reorientation_regions(path)
    regions += tag_subnozzle_feature_regions(path)
    regions += tag_blend_zones(path, header)
    regions += tag_near_singularity_and_velocity_limited_regions(path, twin)
    for r in regions:
        attach_kinematic_profile(r, path, twin)
    return regions
