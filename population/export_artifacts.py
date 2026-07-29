"""
Regenerate the released ontology-side artifacts from the source of truth:

  ontology/ramp.ttl              <- Turtle serialization of ontology/ramp.rdf
  ontology/thresholds.json       <- the released threshold set Theta (config.py)
  ontology/queries/cq{1..5}.rq   <- competency questions (sparql_queries.py)
  ontology/queries/risk_rules/<risktype>.rq
                                 <- per-RiskType retrieval queries

Run:  python population/export_artifacts.py
These files are committed so the repo is browsable/citable without running
anything; this script exists so they can be regenerated deterministically.
"""
import json
from pathlib import Path

from rdflib import Graph

import config
import sparql_queries as SQ

REPO = Path(__file__).resolve().parent.parent
ONT = REPO / "ontology"
Q = ONT / "queries"
RR = Q / "risk_rules"
for d in (ONT, Q, RR):
    d.mkdir(parents=True, exist_ok=True)


def export_turtle():
    g = Graph()
    g.parse(config.ONTOLOGY_PATH, format="xml")
    g.serialize(destination=str(ONT / "ramp.ttl"), format="turtle")
    print(f"ramp.ttl  ({len(g)} triples)")


def export_thresholds():
    payload = {
        "_note": "Released threshold set Theta for the RAMP a-priori risk taggers "
                 "and competency questions. Cell-calibrated; see README / paper. "
                 "THETA_SENSITIVITY holds the paper's illustrative alternatives.",
        "namespace": config.RAMP_NS,
        "theta": config.THETA,
        "theta_sensitivity": config.THETA_SENSITIVITY,
        "joint_velocity_max_deg_per_s": config.JOINT_VEL_MAX_DEG_S,
        "rsi_sample_period_s": config.RSI_SAMPLE_PERIOD_S,
    }
    (ONT / "thresholds.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print("thresholds.json")


HEADER = ("# RAMP competency question -- released query.\n"
          "# Threshold literals are the released Theta (ontology/thresholds.json);\n"
          "# run against a graph populated per population/kg_population.py.\n")


def export_queries():
    cq1 = SQ.CQ1_QUERY_TEMPLATE.format(
        twist_threshold=config.THETA["cq1_twist_deg_per_mm"],
        wrist_manip_threshold=config.THETA["cq1_wrist_manip_w"],
    )
    files = {
        "cq1.rq": cq1,
        "cq2.rq": SQ.CQ2_QUERY,
        "cq3.rq": SQ.CQ3_FLAGGED_WITH_OBSERVATION_QUERY,
        "cq3_unflagged.rq": SQ.CQ3_OBSERVATION_WITHOUT_FLAG_QUERY,
        "cq4.rq": SQ.CQ4_QUERY,
        "cq5.rq": SQ.CQ5_QUERY,
    }
    for name, q in files.items():
        (Q / name).write_text(HEADER + q.strip() + "\n", encoding="utf-8")
    print(f"queries/*.rq ({len(files)} files)")


RISK_TYPES = {
    "blending_distortion": "BlendingDistortion",
    "wrist_joint_loading": "WristJointLoading",
    "kinematic_velocity_limitation": "KinematicVelocityLimitation",
    "singularity_proximity": "SingularityProximity",
    "geometric_infeasibility": "GeometricInfeasibility",
}

RR_TEMPLATE = SQ.PREFIXES + """
SELECT ?program ?region ?remedy WHERE {{
  ?program ramp:hasRegion ?region .
  ?region ramp:hasRiskFlag ?flag .
  ?flag ramp:hasRiskType ramp:{risk_type} .
  OPTIONAL {{ ?flag ramp:suggestsRemedy ?remedy . }}
}}
ORDER BY ?program ?region
"""

RR_HEADER = ("# RAMP a-priori risk rule (Table 1) -- retrieval query.\n"
             "# Returns every region carrying this RiskType flag and its suggested\n"
             "# remedies. The thresholded DETECTION logic that assigns the flags\n"
             "# lives in population/risk_rules.py; this query reads the result off\n"
             "# the populated graph.\n")


def export_risk_rule_queries():
    for fname, rtype in RISK_TYPES.items():
        q = RR_TEMPLATE.format(risk_type=rtype)
        (RR / f"{fname}.rq").write_text(RR_HEADER + q.strip() + "\n", encoding="utf-8")
    print(f"queries/risk_rules/*.rq ({len(RISK_TYPES)} files)")


if __name__ == "__main__":
    export_turtle()
    export_thresholds()
    export_queries()
    export_risk_rule_queries()
    print("done.")
