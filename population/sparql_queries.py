"""
Stage 5 -- SPARQL flagging / competency questions (plan.md Section 4 Stage 5,
Section 1 CQ1-CQ5). Real SPARQL queries executed against the populated
ramp_abox.ttl (+ ontology.rdf for class/property context), not pandas
shortcuts, per plan.md's explicit requirement.
"""
from rdflib import Graph, Namespace
from config import RAMP_NS, THETA

RAMP = Namespace(RAMP_NS)

PREFIXES = f"""
PREFIX ramp: <{RAMP_NS}>
PREFIX xsd: <http://www.w3.org/2001/XMLSchema#>
"""


def load_populated_graph(abox_path, ontology_path=None) -> Graph:
    g = Graph()
    g.parse(abox_path, format="turtle")
    if ontology_path is not None:
        g.parse(ontology_path, format="xml")
    return g


# ---------------------------------------------------------------------------
# CQ1: regions combining high angular twist with low wrist manipulability
# ---------------------------------------------------------------------------
CQ1_QUERY_TEMPLATE = PREFIXES + """
SELECT ?region ?program ?twist ?wristManip WHERE {{
  ?program ramp:hasRegion ?region .
  ?region a ramp:SharpReorientationRegion ;
          ramp:angularTwistRateDegPerMm ?twist ;
          ramp:hasKinematicProfile ?kp .
  ?kp ramp:wristManipulabilityMin ?wristManip .
  FILTER(?twist > {twist_threshold})
  FILTER(?wristManip < {wrist_manip_threshold})
}}
ORDER BY DESC(?twist)
"""


def cq1_sharp_reorientation_low_wrist_manip(g: Graph, twist_threshold=None, wrist_manip_threshold=None):
    twist_threshold = THETA["cq1_twist_deg_per_mm"] if twist_threshold is None else twist_threshold
    wrist_manip_threshold = THETA["cq1_wrist_manip_w"] if wrist_manip_threshold is None else wrist_manip_threshold
    q = CQ1_QUERY_TEMPLATE.format(twist_threshold=twist_threshold, wrist_manip_threshold=wrist_manip_threshold)
    return list(g.query(q))


# ---------------------------------------------------------------------------
# CQ2: which remedies apply to a region
# ---------------------------------------------------------------------------
CQ2_QUERY = PREFIXES + """
SELECT ?region ?riskType ?remedy WHERE {
  ?region ramp:hasRiskFlag ?flag .
  ?flag ramp:hasRiskType ?riskType ;
        ramp:suggestsRemedy ?remedy .
}
ORDER BY ?region
"""


def cq2_remedies_per_region(g: Graph):
    return list(g.query(CQ2_QUERY))


# ---------------------------------------------------------------------------
# CQ3: join flagged regions <-> measured infidelity from an execution
# (also: infidelity with no flag)
# ---------------------------------------------------------------------------
CQ3_FLAGGED_WITH_OBSERVATION_QUERY = PREFIXES + """
SELECT ?region ?riskType ?obs ?lateralRms ?speedDip WHERE {
  ?region ramp:hasRiskFlag ?flag .
  ?flag ramp:hasRiskType ?riskType .
  ?obs ramp:observedInRegion ?region ;
       ramp:lateralDeviationRmsMm ?lateralRms ;
       ramp:speedDipPct ?speedDip .
}
ORDER BY ?region
"""

CQ3_OBSERVATION_WITHOUT_FLAG_QUERY = PREFIXES + """
SELECT ?region ?obs ?lateralRms ?speedDip WHERE {
  ?obs ramp:observedInRegion ?region ;
       ramp:lateralDeviationRmsMm ?lateralRms ;
       ramp:speedDipPct ?speedDip .
  FILTER NOT EXISTS { ?region ramp:hasRiskFlag ?anyFlag }
}
ORDER BY DESC(?lateralRms)
"""


def cq3_flagged_with_observation(g: Graph):
    return list(g.query(CQ3_FLAGGED_WITH_OBSERVATION_QUERY))


def cq3_observation_without_flag(g: Graph):
    return list(g.query(CQ3_OBSERVATION_WITHOUT_FLAG_QUERY))


# ---------------------------------------------------------------------------
# CQ4: programs printing onto a given substrate with continuous tilt AND
# near-singular regions. No HybridPrintingContext/Substrate individuals exist
# in this data package (plan.md caveat: not fabricated), so this query is
# provided for completeness and is expected to return an empty result set.
# ---------------------------------------------------------------------------
CQ4_QUERY = PREFIXES + """
SELECT DISTINCT ?program WHERE {
  ?program ramp:hasPrintingContext ?ctx ;
           ramp:hasRegion ?region .
  ?ctx ramp:usesReorientationStrategy ramp:ContinuousTilt .
  ?region a ramp:NearSingularityRegion .
}
"""


def cq4_hybrid_continuous_tilt_near_singularity(g: Graph):
    return list(g.query(CQ4_QUERY))


# ---------------------------------------------------------------------------
# CQ5: planned features infeasible for nozzle dn = 4 mm
# ---------------------------------------------------------------------------
CQ5_QUERY = PREFIXES + """
SELECT ?program ?region ?minRadius WHERE {
  ?program ramp:hasRegion ?region .
  ?region a ramp:SubNozzleFeatureRegion ;
          ramp:minPlannedRadiusMm ?minRadius .
}
ORDER BY ?minRadius
"""


def cq5_subnozzle_features(g: Graph):
    return list(g.query(CQ5_QUERY))


# ---------------------------------------------------------------------------
# Table 1 risk-rule counts, per program x RiskType (D3), via SPARQL directly
# ---------------------------------------------------------------------------
RISK_COUNT_QUERY = PREFIXES + """
SELECT ?program (COUNT(DISTINCT ?flag) AS ?n) WHERE {
  ?program ramp:hasRegion ?region .
  ?region ramp:hasRiskFlag ?flag .
  ?flag ramp:hasRiskType ramp:__RISK_TYPE__ .
}
GROUP BY ?program
ORDER BY ?program
"""


def risk_counts_per_program(g: Graph, risk_type_name: str):
    q = RISK_COUNT_QUERY.replace("__RISK_TYPE__", risk_type_name)
    return list(g.query(q))


# ---------------------------------------------------------------------------
# D1/D2 support: region and command counts per program
# ---------------------------------------------------------------------------
REGION_COUNT_BY_TAG_QUERY = PREFIXES + """
SELECT ?program ?tag (COUNT(DISTINCT ?region) AS ?n) WHERE {
  ?program ramp:hasRegion ?region .
  ?region a ?tag .
  FILTER(?tag != ramp:Region)
}
GROUP BY ?program ?tag
ORDER BY ?program ?tag
"""


def region_counts_by_tag(g: Graph):
    return list(g.query(REGION_COUNT_BY_TAG_QUERY))


COMMAND_COUNT_QUERY = PREFIXES + """
SELECT ?program (COUNT(DISTINCT ?cmd) AS ?n) WHERE {
  ?program ramp:hasCommand ?cmd .
}
GROUP BY ?program
ORDER BY ?program
"""


def command_counts(g: Graph):
    return list(g.query(COMMAND_COUNT_QUERY))
