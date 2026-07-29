
# RAMP — Robotic Additive Manufacturing Planned-toolpath ontology

A minimal OWL ontology and reproducible population pipeline that represent
**planned 6-DOF robotic additive-manufacturing toolpaths as semantically
enriched objects**, so that design-stage trajectory-fidelity risks can be
identified *a priori* (before, or without, execution) by SPARQL queries over a
populated knowledge graph — and then corroborated against measured robot
telemetry.

This repository is the code-and-ontology artifact accompanying the paper:

> P. Kamble and N. Unger, *"Semantic Representation of Planned Toolpaths for
> 6-DOF Robotic Additive Manufacturing"*, DLR Institute of Vehicle Concepts.
> *(Add journal / DOI on publication.)*

---

## What's here

```
ramp-ontology/
├── README.md
├── LICENSE                     Apache-2.0
├── EXPLAINER.md                plain-language method + concept walkthrough
├── dhm_parameters.txt          KUKA KR 30 HA nominal Modified-DH parameters
├── ontology/
│   ├── ramp.rdf                the ontology (RDF/XML)   ── source of truth
│   ├── ramp.ttl                the ontology (Turtle)    ── generated mirror
│   ├── thresholds.json         released threshold set Θ (+ sensitivity set)
│   └── queries/
│       ├── cq1.rq … cq5.rq     competency questions (SPARQL)
│       ├── cq3_unflagged.rq    "measured deviation with no a-priori flag"
│       └── risk_rules/*.rq     per-RiskType retrieval queries (Table 1)
└── population/
    ├── *.py                    the pipeline (see mapping below)
    ├── export_artifacts.py     regenerates ramp.ttl / thresholds.json / *.rq
    └── requirements.txt
```

The `.ttl`, `.json`, and `.rq` files under `ontology/` are **generated from the
Python source** (`python population/export_artifacts.py`) so they cannot drift
from the implementation; they are committed so the ontology and queries can be
browsed, cited, and run without executing anything.

### Pipeline modules (`population/`)

The code is kept modular rather than collapsed into a few files, so each stage
is independently readable and testable.

| Module | Role | (sketch name) |
|---|---|---|
| `config.py` | all thresholds Θ, paths, constants — single source of truth | |
| `krl_parser.py` | **Stage 1** parse KUKA `.src` (KRL) → motion blocks + header | `extract_krl.py` |
| `kinematics.py` | KR 30 HA forward kinematics + geometric Jacobian (from DH) | |
| `planned_path.py` | build planned path `a` (piecewise-linear pos, SO(3) geodesic ori) | |
| `twin_association.py` | **Stage 2** associate RoboDK digital-twin joint samples; κ, manipulability, wrist manipulability, joint reserves | `robodk_twin.py` |
| `regions.py` | **Stage 3** region taggers (corner, sharp reorientation, near-singularity, velocity-limited, sub-nozzle, blend zone) | `taggers.py` |
| `risk_rules.py` | the six a-priori risk rules (Table 1) → RiskFlags | |
| `kg_population.py` | **Stage 4** emit RDF ABox conforming to `ramp.rdf`, provenance on every quantity | |
| `sparql_queries.py` | **Stage 5** CQ1–CQ5 + risk counts as real SPARQL | |
| `rsi_reconstruction.py` | physical lane: align/trim RSI telemetry, monotone closest-point projection onto `a` | |
| `deviation_metrics.py` | e⊥, eθ, δv, exit-ringing from executed vs planned | |
| `pipeline.py` | driver: runs all stages, merges graphs, writes the ABox | |
| `gen_tables.py`, `gen_d4_d6.py`, `gen_figures*.py` | reproduce the paper's tables and figures | |

---

## The idea in one paragraph

By the time a toolpath reaches the controller it is a flat list of `LIN`/`SLIN`
commands that has lost the engineering meaning of *why* a move is risky. RAMP
re-attaches that meaning as a knowledge graph: every motion command, geometric
region, predicted risk, and measured deviation becomes a typed node with
relationships. The pipeline runs **two strictly separate lanes** — an
*a-priori lane* (plan + kinematic digital twin only) that predicts risk before
the robot runs, and a *physical lane* (RSI telemetry) that measures what
actually happened — which meet only in the CQ3 join. Every quantitative
assertion carries a `hasProvenance` link marking it digital-twin-simulated or
RSI-measured; the two are never mixed within a metric. See `EXPLAINER.md`.

---

## Reproducing the results

The pipeline is deterministic — no learning, no manual annotation. Every
reported number is a function of the raw data + `config.THETA`.

```bash
cd population
pip install -r requirements.txt

# 1) regenerate the ontology-side artifacts (ttl / thresholds / queries)
python export_artifacts.py

# 2) run the full pipeline + build figures/tables  (needs the data package)
python gen_figures_driver.py
```

Running SPARQL alone against a populated graph, e.g. with `rdflib`:

```python
from rdflib import Graph
g = Graph(); g.parse("outputs/kg/ramp_abox.ttl", format="turtle")
print(len(list(g.query(open("ontology/queries/cq1.rq").read()))))
```

### Data availability

The raw experimental **data package is not part of this repository** (KRL
programs, RSI telemetry, and RoboDK digital-twin exports). The pipeline expects
it under `population/Data_Package/` (or set `RAMP_DATA_ROOT`); layout is
documented in `krl_parser.py` / `config.py`. Data may be available from the
authors on reasonable request. `dhm_parameters.txt` (nominal KR 30 HA DH
parameters) *is* included because the kinematics module needs it and it is a
commercial-robot nominal specification, not experimental data.

---

## The threshold set Θ

All region-tagging and query thresholds live in `config.py` and are exported to
`ontology/thresholds.json`. Values are **cell-calibrated** (tuned to this
manufacturing cell and corpus) and stated as such; `theta_sensitivity` holds the
paper's illustrative alternatives so results at both settings are traceable.
Kinematic condition number / manipulability use a metre-scaled linear Jacobian
block so they are not artifacts of a length-unit choice.

---

## Citing

Please cite the paper (above). To also cite the software/ontology directly, a
`CITATION.cff` can be added, or archive a release via Zenodo for a DOI.

## License

Apache-2.0 — see [LICENSE](LICENSE). Ontology: CC-BY-4.0 as declared inside
`ramp.rdf`.
=======
# ramp-ontology
RAMP ontology and population pipeline for a priori risk detection in robotic additive manufacturing.

