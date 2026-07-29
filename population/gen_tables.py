"""
D1-D3 table generation (plan.md Section 5). Consumes the in-memory
ProgramResult bundle from pipeline.run_all_programs() plus SPARQL queries
against the merged graph, and writes Markdown tables to outputs/tables/.
"""
import json
from collections import Counter, defaultdict

import config
import sparql_queries as SQ

RISK_TYPES = ["BlendingDistortion", "WristJointLoading", "KinematicVelocityLimitation",
              "SingularityProximity", "GeometricInfeasibility"]
RISK_LABELS = {
    "BlendingDistortion": "Blending distortion",
    "WristJointLoading": "Wrist-joint loading",
    "KinematicVelocityLimitation": "Kinematic velocity limitation",
    "SingularityProximity": "Singularity proximity",
    "GeometricInfeasibility": "Geometric infeasibility",
}


def md_table(headers, rows):
    lines = ["| " + " | ".join(headers) + " |", "|" + "|".join(["---"] * len(headers)) + "|"]
    for row in rows:
        lines.append("| " + " | ".join(str(c) for c in row) + " |")
    return "\n".join(lines)


def d1_program_corpus_table(results):
    rows = []
    for pid in config.PID_ORDER:
        key = config.PID_TO_KEY[pid]
        r = results[key]
        n_blocks = len(r.blocks)
        n_lin = sum(1 for b in r.blocks if b.motion_type == "LIN")
        n_slin = sum(1 for b in r.blocks if b.motion_type == "SLIN")
        n_corner = sum(1 for _, tag, _ in r.region_entries if tag == "CornerRegion")
        n_sharp = sum(1 for _, tag, _ in r.region_entries if tag == "SharpReorientationRegion")
        n_nearsing = sum(1 for _, tag, _ in r.region_entries if tag == "NearSingularityRegion")
        rows.append([pid, r.ptype, n_blocks, n_lin, n_slin, n_corner, n_sharp, n_nearsing])
    headers = ["Program", "Type", "Motion blocks", "LIN", "SLIN", "Corners",
               "Sharp reorientations", "Near-singularity regions"]
    return md_table(headers, rows)


def d2_kg_statistics(results, merged_graph):
    total_commands = sum(len(r.blocks) for r in results.values())
    total_regions = sum(len(r.regions) for r in results.values())
    total_triples = len(merged_graph)
    lines = [
        f"- Total motion commands across P01-P04: **{total_commands}**",
        f"- Total tagged regions across P01-P04: **{total_regions}**",
        f"- Total RDF triples in ramp_abox.ttl: **{total_triples}**",
    ]
    return "\n".join(lines)


def d3_flag_counts_table(g):
    counts = defaultdict(lambda: defaultdict(int))
    for risk_type in RISK_TYPES:
        for row in SQ.risk_counts_per_program(g, risk_type):
            program_uri = str(row[0])
            pid = program_uri.rsplit("_", 1)[-1]
            counts[pid][risk_type] = int(row[1])

    rows = []
    totals = defaultdict(int)
    for pid in config.PID_ORDER:
        row = [pid]
        row_total = 0
        for risk_type in RISK_TYPES:
            n = counts[pid].get(risk_type, 0)
            row.append(n)
            row_total += n
            totals[risk_type] += n
        row.append(row_total)
        rows.append(row)
    grand_total = sum(totals.values())
    totals_row = ["Total"] + [totals[rt] for rt in RISK_TYPES] + [grand_total]
    rows.append(totals_row)

    headers = ["Program"] + [RISK_LABELS[rt] for rt in RISK_TYPES] + ["Total"]
    return md_table(headers, rows)


def theta_table():
    from config import THETA, THETA_SENSITIVITY
    rows = []
    for k, v in THETA.items():
        sens = THETA_SENSITIVITY.get(k, "")
        rows.append([k, v, sens])
    return md_table(["Parameter", "Primary value (used)", "Paper-illustrative / sensitivity value"], rows)


def generate_all_tables(results, merged_graph, g_populated):
    out = {}
    out["d1_program_corpus.md"] = d1_program_corpus_table(results)
    out["d2_kg_statistics.md"] = d2_kg_statistics(results, merged_graph)
    out["d3_flag_counts.md"] = d3_flag_counts_table(g_populated)
    out["theta_table.md"] = theta_table()
    for fname, content in out.items():
        (config.TABLE_DIR / fname).parent.mkdir(parents=True, exist_ok=True)
        (config.TABLE_DIR / fname).write_text(content, encoding="utf-8")
    return out


if __name__ == "__main__":
    import pipeline
    results, merged_graph = pipeline.run_all_programs()
    pipeline.save_kg(merged_graph, config.KG_DIR / "ramp_abox.ttl")
    g_populated = SQ.load_populated_graph(config.KG_DIR / "ramp_abox.ttl")
    tables = generate_all_tables(results, merged_graph, g_populated)
    for name, content in tables.items():
        print(f"=== {name} ===")
        print(content)
        print()
