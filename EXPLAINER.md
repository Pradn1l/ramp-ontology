# RAMP Pipeline & Results — Explainer

A plain-language companion to the code in `src/` and the numbers in `results_draft.md`.
Everything below was cross-checked against a fresh pipeline run (33,336 triples, 4 programs, all RSI lanes active).

---

## PART 1 — TL;DR code explainer

The pipeline has **two lanes** that stay strictly separate and meet in exactly one place (the CQ3 join). Think of it as: *predict risks from the plan alone (a priori), then measure what actually happened (RSI), then compare.*

```
                 ┌─────────────────────── A PRIORI LANE (never touches RSI) ───────────────────────┐
 .src (KRL) ─▶ Stage1 parse ─▶ Stage2 twin ─▶ Stage3 tag ─▶ Stage4 KG ─▶ Stage5 SPARQL
 *_dt.csv ────────────────────▲ (joint angles)                 │                    │
 dhm_parameters.txt ─▶ kinematics (Jacobian)                   ▼                    ▼
                                                          ramp_abox.ttl         risk flags + CQ1,CQ2,CQ4,CQ5
                 └────────────────────────────────────────────┬───────────────────────────────────┘
                                                               │  CQ3 = the ONLY meeting point
                 ┌──────────── PHYSICAL LANE (RSI only) ───────┴──────┐
 rsi_data.dat ─▶ align+trim ─▶ monotone projection ─▶ deviation metrics ─▶ DeviationObservation (RSI)
```

### File-by-file (in `src/`)

| File | One-line job |
|---|---|
| `config.py` | Every threshold, path, constant in one place (the "Θ" set). Nothing is hardcoded elsewhere. |
| `kinematics.py` | KUKA KR 30 HA forward kinematics + Jacobian from the DH table → condition number, manipulability, wrist manipulability. Validated to 1e-7 vs a numerical Jacobian. |
| `krl_parser.py` | **Stage 1.** Reads `.src` files → ordered list of motion blocks (LIN/SLIN/PTP) with target pose + speed + blend switch; parses the header ($APO, $VEL, $ACC, $ADVANCE…); keeps only the first layer. |
| `planned_path.py` | Builds **path `a`**: the planned trajectory. Piecewise-linear position, SO(3)-geodesic orientation, arc-length parameterized. This is the reference everything is measured against. |
| `twin_association.py` | **Stage 2.** Loads `*_dt.csv` (digital-twin joint angles along `a`), truncates to the first layer, computes per-sample kinematic descriptors + joint reserves. **All DT-provenance.** |
| `regions.py` | **Stage 3.** Applies Θ to tag arc-length intervals: Corner, SharpReorientation, NearSingularity, VelocityLimited, SubNozzleFeature, BlendZone. Regions are **not** mutually exclusive. |
| `risk_rules.py` | The 6 a priori risk rules (Table 1). Turns tagged regions → RiskFlags with a RiskType + suggested Remedies. |
| `kg_population.py` | **Stage 4.** Emits RDF triples to build the knowledge graph, using *only* the ontology's real IRIs. Attaches provenance to every quantity. |
| `sparql_queries.py` | **Stage 5.** The competency questions (CQ1–CQ5) and risk counts as **real SPARQL** run against the populated graph. |
| `rsi_reconstruction.py` | **Physical lane.** Loads `rsi_data.dat`, finds where the print starts/ends, projects each executed sample onto path `a` (monotone closest-point). |
| `deviation_metrics.py` | Computes `e⊥` (lateral), `eθ` (orientation), `δv` (speed dip), `rW` (ringing) of executed vs planned. |
| `pipeline.py` | The driver: runs all stages for all 4 programs, merges the graphs, writes `ramp_abox.ttl`. |
| `gen_tables.py` / `gen_d4_d6.py` / `gen_figures*.py` | Turn the results into the D1–D8 tables and figures. |

### To reproduce everything
```bash
python src/gen_figures_driver.py    # rebuilds KG + all figures + numbers from raw data
```
No manual steps, no hand-entered numbers. Every value in `results_draft.md` traces to a function (see its Numbers Ledger).

---

## PART 2 — Concept explainer

### The core idea in one paragraph
A robot toolpath, by the time it reaches the controller, is a flat list of `LIN`/`SLIN` commands that has *lost* the engineering meaning of *why* each move is risky. RAMP re-attaches that meaning as a **knowledge graph**: each motion command, each geometric region, each predicted risk, and each measured deviation becomes a node with typed relationships. Once it's a graph, you can *ask questions* of it in SPARQL instead of writing bespoke analysis code — and crucially, you can predict risk **before** running the robot, then check those predictions against telemetry.

### The three trajectories (this is the mental model for everything)
- **`a` = planned path.** The sharp, exact geometry commanded by the KRL program. *We build this.* (`planned_path.py`)
- **`b` = controller's blended approximation.** What the KRC4 controller *actually aims for* after applying blending. **Never measured, never reconstructed** — it only exists as a set of knob values (β).
- **`c` = executed path.** What the robot's TCP actually did, measured by RSI at 4 ms. *We measure this.* (`rsi_reconstruction.py`)

The gap `a→b` is **designed** (you asked for blending). The gap `b→c` is **unintended infidelity** — that's the thing worth measuring. We measure `c` relative to `a`.

---

## PART 3 — Your specific questions

### Q1: "How exactly is the KG used?"

The KG is used as a **queryable store of facts + relationships**, not as a database of pre-computed answers. Concretely:

1. **Population (Stage 4):** the pipeline writes ~33k triples. Example, for one flagged region, the graph literally contains:
   ```turtle
   ramp:program_P04 ramp:hasRegion ramp:program_P04_region_SharpReorientationRegion_49 .
   ramp:program_P04_region_SharpReorientationRegion_49
       a ramp:SharpReorientationRegion ;
       ramp:angularTwistRateDegPerMm 3.286 ;
       ramp:hasKinematicProfile ramp:...kp... ;
       ramp:hasRiskFlag ramp:...flag... .
   ramp:...kp... ramp:wristManipulabilityMin 2.4365 ;
                 ramp:hasProvenance ramp:DigitalTwinSimulated .
   ramp:...flag... ramp:hasRiskType ramp:WristJointLoading ;
                   ramp:suggestsRemedy ramp:ReorientationRedistribution .
   ```
2. **Querying (Stage 5):** the competency questions are SPARQL patterns that *traverse* those relationships. CQ1, for example, says "find any Region typed SharpReorientationRegion whose twist rate is high AND whose linked KinematicProfile has low wrist manipulability." The KG does the join between region geometry and kinematic descriptors for you.

**Why bother with a graph instead of pandas?** Three reasons that matter for the paper:
- **Portability.** The risk logic is expressed in the ontology's vocabulary, not tied to our CSV column names. Another lab's toolpath, populated into the same TBox, answers the same queries.
- **Provenance.** Every quantitative node carries `hasProvenance` (DT vs RSI). Verified: 839/839 KinematicProfiles are DT, 815/815 DeviationObservations are RSI, zero mixing. A pandas dataframe wouldn't enforce that separation.
- **Composability.** CQ3 joins the a priori lane and the physical lane *through the graph* (a flagged Region ↔ a DeviationObservation `observedInRegion` that same Region) — no custom merge code.

### Q2: "Can I just write anything to the KG and get an answer per our SPARQL rule?"

**Yes — and I tested exactly this to be sure.** The SPARQL rules are pure graph queries; they respond to whatever triples are present, not to our specific data. Proof (from the cross-check run):

> I injected one synthetic region into a copy of the graph:
> `SharpReorientationRegion` with `angularTwistRateDegPerMm = 99.0` and a linked profile with `wristManipulabilityMin = 0.001`.
> **CQ1 went from 15 hits → 16 hits.** The query found the fake region purely from its structure.

So the important caveats:
- **You must write triples that conform to the ontology's shape.** CQ1 only "sees" a region if it has (a) `rdf:type ramp:SharpReorientationRegion`, (b) an `angularTwistRateDegPerMm` value, and (c) a `hasKinematicProfile` link to a node with `wristManipulabilityMin`. Miss any of those predicates and the query silently won't match — not because the rule is wrong, but because the pattern isn't satisfied.
- **The query has no idea whether your numbers are "real."** It will happily flag a fabricated region. The *discipline* that makes RAMP's numbers trustworthy lives in the **population** step (deterministic, provenance-tagged, no hand-entry), not in the query step. This is exactly why the pipeline forbids manual annotation.
- **Thresholds live in the query, not the graph.** e.g. CQ1's "twist > 0.75, wrist < 2.4496" are filter constants in the SPARQL string (sourced from `config.THETA`). Change Θ → re-run the query → different hits, same graph.

Short version: **the KG is a faithful mirror of whatever you populate; SPARQL answers questions about that mirror. Garbage in, garbage flagged.** The trust comes from the deterministic population pipeline, not from the query.

### Q3: "What are the things in the plots — planned or RSI?"

Verified directly against the plotting code. Here's every plot, colour by colour:

| Figure | Element | Source | Lane |
|---|---|---|---|
| **Fig 4 top** (X-Z) | **black line** "planned path a" | `position_at_s` on KRL targets | **PLANNED** |
| Fig 4 top | **blue line** "executed path c" | `run_xyz` from `rsi_data.dat` | **RSI** |
| Fig 4 top | grey shading (blend zones) | planned-path arc-length intervals | PLANNED |
| **Fig 4 middle** | `e⊥` (blue), `eθ` (red) | executed-vs-planned deviation | **RSI vs PLANNED** |
| **Fig 4 bottom** | **black dotted** "commanded v" | KRL `$VEL.CP` | PLANNED |
| Fig 4 bottom | **blue** "executed TCP speed" | differentiated RSI positions | **RSI** |
| **Risk maps** (2D & 3D) | **black line** (the whole path) | `position_at_s` on KRL targets | **PLANNED** |
| Risk maps | coloured overlays (Corner/Sharp/…) | planned-path region intervals | **PLANNED** |
| **Twist/wrist-manip** | ρ(s), w_wr(s) | planned geometry + DT joint angles | **PLANNED + DT** |
| **Joint-reserve heatmap** | reserves | DT joint angles along `a` | **DT (planned lane)** |
| **Speed-dip profile** | δv(s) | executed vs commanded | **RSI vs PLANNED** |

**The rule of thumb:**
- **Risk maps, twist/wrist, joint reserves = 100% a-priori** (planned geometry + digital twin). These are what you know *before* touching the robot. The coloured risk regions are drawn on the **planned** path.
- **Anything blue/"executed", and all deviation/speed-dip panels = RSI** (measured).
- **Figure 4 is the only plot that shows both at once** — that's the whole point of it: the flag (planned/a-priori) was raised *before* the blue RSI trace existed, and the trace confirms it.

So when you look at a risk map and see a corner flagged: that's a **prediction from the plan**, not something read off the executed motion. The executed motion only enters in Figure 4 and the deviation/speed panels.

---

## One-sentence summary
*We rebuild the planned path from the KRL program, predict where it's risky using the digital twin (a-priori lane), store all of it as a provenance-tagged knowledge graph you can query in SPARQL, then measure the real executed path from RSI (physical lane) and check the predictions — and every plot is one lane or the other, except Figure 4 which deliberately overlays both.*
