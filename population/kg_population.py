"""
Stage 4 -- KG population (plan.md Section 4 Stage 4). Emits RDF triples
conforming to ontology.rdf (ABox only), one graph per run, merged into a
single ramp_abox.ttl covering all four programs (P01-P04).

Every quantitative assertion carries hasProvenance (DigitalTwinSimulated or
RsiMeasured), per the ontology's Provenance discipline. Only real ontology
IRIs are used; nothing is invented.
"""
from rdflib import Graph, Namespace, Literal, RDF, XSD, URIRef
from rdflib.namespace import RDFS

from config import RAMP_NS

RAMP = Namespace(RAMP_NS)


def new_graph() -> Graph:
    g = Graph()
    g.bind("ramp", RAMP)
    return g


def program_uri(pid: str) -> URIRef:
    return RAMP[f"program_{pid}"]


def command_uri(pid: str, block_index: int) -> URIRef:
    return RAMP[f"program_{pid}_cmd_{block_index}"]


def pose_uri(pid: str, block_index: int) -> URIRef:
    return RAMP[f"program_{pid}_pose_{block_index}"]


def blending_uri(pid: str, block_index: int) -> URIRef:
    return RAMP[f"program_{pid}_blend_{block_index}"]


def dynamics_uri(pid: str) -> URIRef:
    return RAMP[f"program_{pid}_dynamics"]


def region_uri(pid: str, tag: str, idx: int) -> URIRef:
    return RAMP[f"program_{pid}_region_{tag}_{idx}"]


def kinprofile_uri(pid: str, tag: str, idx: int) -> URIRef:
    return RAMP[f"program_{pid}_kinprofile_{tag}_{idx}"]


def riskflag_uri(pid: str, tag: str, idx: int, risk_name: str) -> URIRef:
    return RAMP[f"program_{pid}_riskflag_{tag}_{idx}_{risk_name}"]


def execrecord_uri(pid: str) -> URIRef:
    return RAMP[f"program_{pid}_execrecord"]


def observation_uri(pid: str, tag: str, idx: int) -> URIRef:
    return RAMP[f"program_{pid}_obs_{tag}_{idx}"]


MOTION_CLASS_BY_TYPE = {
    "LIN": RAMP.LinearMotion,
    "SLIN": RAMP.SplineLinearMotion,
    "PTP": RAMP.JointMotion,
}

REGION_CLASS_BY_TAG = {
    "CornerRegion": RAMP.CornerRegion,
    "SharpReorientationRegion": RAMP.SharpReorientationRegion,
    "NearSingularityRegion": RAMP.NearSingularityRegion,
    "VelocityLimitedRegion": RAMP.VelocityLimitedRegion,
    "SubNozzleFeatureRegion": RAMP.SubNozzleFeatureRegion,
    "BlendZone": RAMP.BlendZone,
}

RISK_TYPE_URI = {
    "BlendingDistortion": RAMP.BlendingDistortion,
    "WristJointLoading": RAMP.WristJointLoading,
    "KinematicVelocityLimitation": RAMP.KinematicVelocityLimitation,
    "SingularityProximity": RAMP.SingularityProximity,
    "GeometricInfeasibility": RAMP.GeometricInfeasibility,
}

REMEDY_URI = {
    "VelocityReduction": RAMP.VelocityReduction,
    "JerkRelaxation": RAMP.JerkRelaxation,
    "LookAheadExtension": RAMP.LookAheadExtension,
    "BlendRadiusAdjustment": RAMP.BlendRadiusAdjustment,
    "DesignFilleting": RAMP.DesignFilleting,
    "ReorientationRedistribution": RAMP.ReorientationRedistribution,
}


def add_program(g: Graph, pid: str, header) -> URIRef:
    p = program_uri(pid)
    g.add((p, RDF.type, RAMP.ToolpathProgram))

    dyn = dynamics_uri(pid)
    g.add((dyn, RDF.type, RAMP.DynamicsSpecification))
    if header.acc_cp_pct is not None:
        g.add((dyn, RAMP.pathAccelerationPct, Literal(header.acc_cp_pct, datatype=XSD.double)))
    if header.jerk_cp_pct is not None:
        g.add((dyn, RAMP.pathJerkPct, Literal(header.jerk_cp_pct, datatype=XSD.double)))
    if header.advance_blocks is not None:
        g.add((dyn, RAMP.lookAheadBlocks, Literal(header.advance_blocks, datatype=XSD.integer)))
    g.add((p, RAMP.hasDynamics, dyn))
    return p


def add_commands(g: Graph, pid: str, blocks):
    prev_uri = None
    for b in blocks:
        cmd = command_uri(pid, b.block_index)
        g.add((cmd, RDF.type, MOTION_CLASS_BY_TYPE[b.motion_type]))
        g.add((cmd, RAMP.blockIndex, Literal(b.block_index, datatype=XSD.integer)))
        g.add((cmd, RAMP.sourceLine, Literal(b.source_line, datatype=XSD.integer)))
        if not (b.commanded_vel_cp != b.commanded_vel_cp):  # not NaN
            g.add((cmd, RAMP.commandedSpeedMmPerS, Literal(b.commanded_vel_cp * 1000.0, datatype=XSD.double)))

        pose = pose_uri(pid, b.block_index)
        g.add((pose, RDF.type, RAMP.Pose))
        g.add((pose, RAMP.px, Literal(b.x, datatype=XSD.double)))
        g.add((pose, RAMP.py, Literal(b.y, datatype=XSD.double)))
        g.add((pose, RAMP.pz, Literal(b.z, datatype=XSD.double)))
        g.add((pose, RAMP.eulerA, Literal(b.a, datatype=XSD.double)))
        g.add((pose, RAMP.eulerB, Literal(b.b, datatype=XSD.double)))
        g.add((pose, RAMP.eulerC, Literal(b.c, datatype=XSD.double)))
        g.add((cmd, RAMP.hasTargetPose, pose))

        if b.blend_switch in ("C_VEL", "C_SPL"):
            blend = blending_uri(pid, b.block_index)
            g.add((blend, RDF.type, RAMP.BlendingSpecification))
            g.add((cmd, RAMP.hasBlending, blend))

        g.add((program_uri(pid), RAMP.hasCommand, cmd))
        if prev_uri is not None:
            g.add((prev_uri, RAMP.nextCommand, cmd))
        prev_uri = cmd


def fill_blend_specs(g: Graph, pid: str, header, blocks):
    """BlendingSpecification datatype properties use the header's active
    $APO.CDIS/$APO.CVEL/$APO.CORI values in force for that block (this corpus
    keeps these constant per program after the header, so one shared value
    set is applied to every blend spec node created in add_commands)."""
    for b in blocks:
        if b.blend_switch not in ("C_VEL", "C_SPL"):
            continue
        blend = blending_uri(pid, b.block_index)
        if header.apo_cdis_mm is not None:
            g.add((blend, RAMP.blendDistanceMm, Literal(header.apo_cdis_mm, datatype=XSD.double)))
        if header.apo_cvel_pct is not None:
            g.add((blend, RAMP.blendVelocityPct, Literal(header.apo_cvel_pct, datatype=XSD.double)))
        if header.apo_cori_deg is not None:
            g.add((blend, RAMP.blendOrientationDeg, Literal(header.apo_cori_deg, datatype=XSD.double)))


def add_regions(g: Graph, pid: str, regions):
    """Adds Region individuals and KinematicProfile (DT-provenanced) nodes;
    returns a list of (region_uri, tag, region_obj) for risk-rule consumption
    downstream and for link_spans_command."""
    counters = {}
    out = []
    for r in regions:
        idx = counters.get(r.tag, 0)
        counters[r.tag] = idx + 1
        ru = region_uri(pid, r.tag, idx)
        g.add((ru, RDF.type, REGION_CLASS_BY_TAG[r.tag]))
        g.add((ru, RAMP.startArcLengthMm, Literal(r.start_s_mm, datatype=XSD.double)))
        g.add((ru, RAMP.endArcLengthMm, Literal(r.end_s_mm, datatype=XSD.double)))
        if r.corner_angle_deg is not None:
            g.add((ru, RAMP.cornerAngleDeg, Literal(r.corner_angle_deg, datatype=XSD.double)))
        if r.angular_twist_rate_deg_per_mm is not None:
            g.add((ru, RAMP.angularTwistRateDegPerMm, Literal(r.angular_twist_rate_deg_per_mm, datatype=XSD.double)))
        if r.min_planned_radius_mm is not None and r.min_planned_radius_mm != float("inf"):
            g.add((ru, RAMP.minPlannedRadiusMm, Literal(r.min_planned_radius_mm, datatype=XSD.double)))

        if r.kappa_max is not None:
            kp = kinprofile_uri(pid, r.tag, idx)
            g.add((kp, RDF.type, RAMP.KinematicProfile))
            g.add((kp, RAMP.jacobianConditionNumberMax, Literal(r.kappa_max, datatype=XSD.double)))
            g.add((kp, RAMP.manipulabilityMin, Literal(r.manip_min, datatype=XSD.double)))
            g.add((kp, RAMP.wristManipulabilityMin, Literal(r.wrist_manip_min, datatype=XSD.double)))
            if r.min_joint_reserve is not None:
                g.add((kp, RAMP.minJointVelocityReserve, Literal(r.min_joint_reserve, datatype=XSD.double)))
            if r.critical_joint is not None:
                g.add((kp, RAMP.criticalJoint, Literal(r.critical_joint, datatype=XSD.integer)))
            g.add((kp, RAMP.hasProvenance, RAMP.DigitalTwinSimulated))
            g.add((ru, RAMP.hasKinematicProfile, kp))

        g.add((program_uri(pid), RAMP.hasRegion, ru))
        out.append((ru, r.tag, r))
    return out


def link_spans_command(g: Graph, pid: str, region_entries, path):
    """Link each region to motion commands whose target arc length falls
    inside [start_s_mm, end_s_mm]."""
    for ru, tag, r in region_entries:
        for i, s in enumerate(path.s_targets):
            if r.start_s_mm - 1e-6 <= s <= r.end_s_mm + 1e-6:
                g.add((ru, RAMP.spansCommand, command_uri(pid, i)))


def add_risk_flags(g: Graph, pid: str, flags):
    """
    flags: list of dicts with keys: region_uri, tag, idx, risk_name, remedies
    (list of remedy names). Emits RiskFlag individuals per the ontology's
    RiskFlag/RiskType/Remedy classes.

    Each flag gets a fresh serial index here (not flag["idx"], which
    risk_rules.py leaves as None) so that every emitted RiskFlag is a
    distinct node -- using a constant/None index collapses every flag of the
    same (tag, risk_name) pair in a program onto one shared URI, which
    silently undercounts D3/CQ-style COUNT(DISTINCT ?flag) queries.
    """
    counters = {}
    for flag in flags:
        key = (flag["tag"], flag["risk_name"])
        serial = counters.get(key, 0)
        counters[key] = serial + 1
        rf = riskflag_uri(pid, flag["tag"], serial, flag["risk_name"])
        g.add((rf, RDF.type, RAMP.RiskFlag))
        g.add((rf, RAMP.hasRiskType, RISK_TYPE_URI[flag["risk_name"]]))
        for remedy in flag.get("remedies", []):
            g.add((rf, RAMP.suggestsRemedy, REMEDY_URI[remedy]))
        g.add((flag["region_uri"], RAMP.hasRiskFlag, rf))


def add_execution_record(g: Graph, pid: str, header, run_label: str) -> URIRef:
    er = execrecord_uri(pid)
    g.add((er, RDF.type, RAMP.ExecutionRecord))
    if header.apo_cdis_mm is not None:
        g.add((er, RAMP.runApoCdisMm, Literal(header.apo_cdis_mm, datatype=XSD.double)))
    if header.acc_cp_pct is not None:
        g.add((er, RAMP.runPathAccelerationPct, Literal(header.acc_cp_pct, datatype=XSD.double)))
    if header.jerk_cp_pct is not None:
        g.add((er, RAMP.runPathJerkPct, Literal(header.jerk_cp_pct, datatype=XSD.double)))
    if header.advance_blocks is not None:
        g.add((er, RAMP.runLookAheadBlocks, Literal(header.advance_blocks, datatype=XSD.integer)))
    g.add((program_uri(pid), RAMP.hasExecutionRecord, er))
    return er


def add_deviation_observation(g: Graph, pid: str, tag: str, idx: int, region_u: URIRef,
                               metrics: dict) -> URIRef:
    """metrics keys: lateral_rms_mm, lateral_max_mm, orientation_max_deg,
    speed_dip_pct, speed_dip_duration_ms, exit_ringing_rms_mm, sampling_rate_hz."""
    obs = observation_uri(pid, tag, idx)
    g.add((obs, RDF.type, RAMP.DeviationObservation))
    if metrics.get("lateral_rms_mm") is not None:
        g.add((obs, RAMP.lateralDeviationRmsMm, Literal(metrics["lateral_rms_mm"], datatype=XSD.double)))
    if metrics.get("lateral_max_mm") is not None:
        g.add((obs, RAMP.lateralDeviationMaxMm, Literal(metrics["lateral_max_mm"], datatype=XSD.double)))
    if metrics.get("orientation_max_deg") is not None:
        g.add((obs, RAMP.orientationDeviationMaxDeg, Literal(metrics["orientation_max_deg"], datatype=XSD.double)))
    if metrics.get("speed_dip_pct") is not None:
        g.add((obs, RAMP.speedDipPct, Literal(metrics["speed_dip_pct"], datatype=XSD.double)))
    if metrics.get("speed_dip_duration_ms") is not None:
        g.add((obs, RAMP.speedDipDurationMs, Literal(metrics["speed_dip_duration_ms"], datatype=XSD.double)))
    if metrics.get("exit_ringing_rms_mm") is not None:
        g.add((obs, RAMP.exitRingingRmsMm, Literal(metrics["exit_ringing_rms_mm"], datatype=XSD.double)))
    if metrics.get("sampling_rate_hz") is not None:
        g.add((obs, RAMP.samplingRateHz, Literal(metrics["sampling_rate_hz"], datatype=XSD.double)))
    g.add((obs, RAMP.hasProvenance, RAMP.RsiMeasured))
    g.add((obs, RAMP.observedInRegion, region_u))
    g.add((execrecord_uri(pid), RAMP.hasObservation, obs))
    return obs
