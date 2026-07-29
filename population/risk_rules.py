"""
The six a priori risk rules (plan.md Section 1, Table 1 of the paper).
Each rule inspects Stage-3 Region objects (plus the planned path / header)
and yields a priori RiskFlag entries later consumed by kg_population.add_risk_flags
and by the D3 flag-count table.

Rule -> RiskType -> suggested Remedy mapping is fixed here (controller-neutral
capability classes only, per the ontology's Remedy class).
"""
import numpy as np
from config import THETA

# KR 30 HA nominal path acceleration at 100% ACC.CP (mm/s^2), used only as the
# scale factor for AccelerationLimitedSegment's ramp-distance check. Not in
# the data package; disclosed as an external nameplate reference like
# JOINT_VEL_MAX_DEG_S (config.py), consistent with plan.md's kinematics
# fallback instruction.
NOMINAL_PATH_ACC_MM_S2_AT_100PCT = 2500.0


def rule_blending_distortion(region_entries, path, blend_distance_mm):
    """
    Rule 1 -- BlendingDistortionRisk: a LIN/SLIN segment with an active
    C_VEL/C_SPL switch whose length is below a threshold relative to the
    predicted (nominal, unclipped) blend extent implied by $APO.CDIS -- i.e.
    the requested blend zone would overlap into the adjacent segment before
    any clipping is applied. Uses the nominal 2*CDIS zone extent, not the
    Region's stored (clipped-to-segment) extent -- the Region's own extent is
    capped at the shorter adjacent segment by construction (see
    regions.tag_blend_zones), which would make an extent/segment-length ratio
    tautologically <= 1 and non-discriminating.
    """
    flags = []
    s_targets = path.s_targets
    nominal_zone_extent = 2.0 * blend_distance_mm
    for ru, tag, r in region_entries:
        if tag != "BlendZone":
            continue
        i = r.block_index_at_vertex
        seg_in = s_targets[i] - s_targets[i - 1] if i > 0 else np.inf
        seg_out = s_targets[i + 1] - s_targets[i] if i < len(s_targets) - 1 else np.inf
        shorter = min(seg_in, seg_out)
        if shorter > 0 and nominal_zone_extent / shorter >= THETA["blend_overlap_ratio_c"]:
            flags.append(dict(region_uri=ru, tag=tag, idx=None, risk_name="BlendingDistortion",
                               remedies=["BlendRadiusAdjustment", "VelocityReduction"]))
    return flags


def rule_acceleration_limited(blocks, path, acc_cp_pct):
    """
    Rule 2 -- AccelerationLimitedSegment: commanded $VEL.CP not achievable
    within the segment under $ACC.CP. Ramp distance to reach v_cmd from rest
    at acceleration a = (ACC.CP% /100) * nominal_path_acc is
    d_ramp = v_cmd^2 / (2a); flagged when d_ramp exceeds the segment length
    (the controller cannot reach commanded speed before the segment ends).
    Returns a list of block_index values (this rule is segment-native per
    Table 1, not Region-based).
    """
    flagged = []
    a = max(acc_cp_pct, 1e-6) / 100.0 * NOMINAL_PATH_ACC_MM_S2_AT_100PCT
    for i in range(1, len(blocks)):
        seg_len = path.s_targets[i] - path.s_targets[i - 1]
        if seg_len <= 0:
            continue
        v_cmd_mm_s = blocks[i].commanded_vel_cp * 1000.0
        d_ramp = v_cmd_mm_s ** 2 / (2 * a)
        if d_ramp > seg_len:
            flagged.append(i)
    return flagged


def rule_wrist_load(region_entries):
    """
    Rule 4 -- WristLoadRisk: reorientation rate above threshold (already the
    SharpReorientationRegion criterion) with wrist-joint demand concentrated
    at axes A4-A6. Uses the region's KinematicProfile criticalJoint: flagged
    when the critical joint is 4, 5, or 6.
    """
    flags = []
    for ru, tag, r in region_entries:
        if tag != "SharpReorientationRegion":
            continue
        crit = r.critical_joint
        if crit is not None and crit in (4, 5, 6):
            flags.append(dict(region_uri=ru, tag=tag, idx=None, risk_name="WristJointLoading",
                               remedies=["ReorientationRedistribution", "JerkRelaxation"]))
        elif crit is None:
            # criticalJoint may be unset if reserves weren't the binding
            # constraint; still flag on twist rate alone per Table 1 (angular
            # twist concentrated over a short translation is itself a wrist
            # load indicator for this spherical-wrist robot, joints 4-6).
            flags.append(dict(region_uri=ru, tag=tag, idx=None, risk_name="WristJointLoading",
                               remedies=["ReorientationRedistribution", "JerkRelaxation"]))
    return flags


def rule_joint_saturation(region_entries):
    """Rule 3 -- JointSaturationRisk: min joint-velocity reserve below
    threshold. Directly the VelocityLimitedRegion criterion."""
    flags = []
    for ru, tag, r in region_entries:
        if tag != "VelocityLimitedRegion":
            continue
        flags.append(dict(region_uri=ru, tag=tag, idx=None, risk_name="KinematicVelocityLimitation",
                           remedies=["VelocityReduction", "LookAheadExtension"]))
    return flags


def rule_singularity_proximity(region_entries):
    """Rule 5 -- SingularityProximityRisk: manipulability index below
    threshold. Applied on NearSingularityRegion (or any region whose
    KinematicProfile.manipulabilityMin < manipulability_c)."""
    flags = []
    for ru, tag, r in region_entries:
        if r.manip_min is not None and r.manip_min < THETA["manipulability_c"]:
            flags.append(dict(region_uri=ru, tag=tag, idx=None, risk_name="SingularityProximity",
                               remedies=["VelocityReduction", "JerkRelaxation"]))
    return flags


def rule_hybrid_deposition(region_entries, substrate_surface_angle_by_region=None):
    """
    Rule 6 -- HybridDepositionFlag: segment over a substrate region whose
    surface angle exceeds threshold. This data package contains no substrate
    surface-angle information for any of the four programs (no
    HybridPrintingContext / Substrate data in Data_Package); the rule is
    implemented mechanically but yields zero flags here, and that absence is
    reported rather than fabricated.
    """
    flags = []
    if not substrate_surface_angle_by_region:
        return flags
    for ru, tag, r in region_entries:
        angle = substrate_surface_angle_by_region.get(ru)
        if angle is not None and angle > THETA["hybrid_surface_angle_c_deg"]:
            flags.append(dict(region_uri=ru, tag=tag, idx=None, risk_name="GeometricInfeasibility",
                               remedies=["DesignFilleting"]))
    return flags


def rule_geometric_infeasibility(region_entries):
    """SubNozzleFeatureRegion -> GeometricInfeasibility risk type directly
    (planned feature unachievable regardless of controller settings)."""
    flags = []
    for ru, tag, r in region_entries:
        if tag != "SubNozzleFeatureRegion":
            continue
        flags.append(dict(region_uri=ru, tag=tag, idx=None, risk_name="GeometricInfeasibility",
                           remedies=["DesignFilleting"]))
    return flags


def apply_all_rules(region_entries, blocks, path, header):
    flags = []
    flags += rule_blending_distortion(region_entries, path, header.apo_cdis_mm or 0.0)
    flags += rule_wrist_load(region_entries)
    flags += rule_joint_saturation(region_entries)
    flags += rule_singularity_proximity(region_entries)
    flags += rule_geometric_infeasibility(region_entries)
    flags += rule_hybrid_deposition(region_entries)  # empty: no substrate data

    accel_limited_indices = rule_acceleration_limited(
        blocks, path, header.acc_cp_pct if header.acc_cp_pct is not None else 100.0
    )
    return flags, accel_limited_indices
