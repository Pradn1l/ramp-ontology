"""
D4 (CQ results) and D6 (telemetry corroboration numbers, Section 3.2)
generation. Consumes pipeline results + sparql_queries against the populated
graph.
"""
import numpy as np
import config
import sparql_queries as SQ
from gen_tables import md_table


def d4_cq1(g):
    rows = SQ.cq1_sharp_reorientation_low_wrist_manip(g)
    lines = [f"**CQ1** ({len(rows)} regions): twist > {config.THETA['cq1_twist_deg_per_mm']} deg/mm "
             f"AND wrist manipulability < {config.THETA['cq1_wrist_manip_w']:.4f} "
             f"(primary, calibrated; paper-illustrative values were 10.0 deg/mm and 0.05 -- see Theta table)."]
    table_rows = []
    for region, program, twist, wrist in rows:
        pid = str(program).rsplit("_", 1)[-1]
        table_rows.append([pid, str(region).rsplit("#", 1)[-1], f"{float(twist):.3f}", f"{float(wrist):.4f}"])
    lines.append(md_table(["Program", "Region", "twist [deg/mm]", "wrist manip"], table_rows))
    return "\n\n".join(lines)


def d4_cq5(g):
    rows = SQ.cq5_subnozzle_features(g)
    lines = [f"**CQ5** ({len(rows)} features): planned features with local contour radius below "
             f"the nozzle-feasible minimum for dn = {config.THETA['nozzle_diameter_mm']} mm "
             f"(min feasible radius = {config.THETA['min_feasible_radius_mm']} mm)."]
    table_rows = []
    for program, region, radius in rows:
        pid = str(program).rsplit("_", 1)[-1]
        table_rows.append([pid, str(region).rsplit("#", 1)[-1], f"{float(radius):.3f}"])
    lines.append(md_table(["Program", "Region", "min planned radius [mm]"], table_rows))
    return "\n\n".join(lines)


def d4_cq3(g):
    flagged = SQ.cq3_flagged_with_observation(g)
    unflagged = SQ.cq3_observation_without_flag(g)
    lines = [
        f"**CQ3 join** -- flagged regions with a corresponding measured DeviationObservation: **{len(flagged)}**. "
        f"Observations whose region carries NO a priori risk flag (\"infidelity with no flag\"): **{len(unflagged)}**.",
    ]
    if unflagged:
        table_rows = []
        for region, obs, lateral_rms, speed_dip in sorted(unflagged, key=lambda r: -float(r[2]))[:15]:
            table_rows.append([str(region).rsplit("#", 1)[-1], f"{float(lateral_rms):.3f}", f"{float(speed_dip):.1f}"])
        lines.append("Top unflagged-but-measured-deviation regions (by lateral RMS):")
        lines.append(md_table(["Region", "lateral RMS [mm]", "speed dip [%]"], table_rows))
    return "\n\n".join(lines)


def d6_aggregate_over_nonplanar(results):
    """
    Aggregate over non-planar programs (P03, P04) via CQ3: of N flagged
    reorientation regions, how many showed measured deviation/speed dip
    beyond designed tolerance, how many stayed within tolerance, and how many
    deviation events occurred outside any flagged region.
    """
    n_flagged_reorientation = 0
    n_outside_tolerance = 0
    n_within_tolerance = 0
    outside_events = []

    for key in ["cladding", "npsrf"]:
        r = results[key]
        if not r.rsi_available:
            continue
        for ru, tag, reg in r.region_entries:
            if tag != "SharpReorientationRegion":
                continue
            obs = r.region_observations.get(str(ru))
            if obs is None:
                continue
            n_flagged_reorientation += 1
            if obs["metrics"]["outside_designed_tolerance"]:
                n_outside_tolerance += 1
            else:
                n_within_tolerance += 1

        # deviation events outside any flagged region
        flagged_spans = [(reg.start_s_mm, reg.end_s_mm) for _, tag, reg in r.region_entries if tag != "BlendZone"]

        def in_any_flagged(s):
            return any(a <= s <= b for a, b in flagged_spans)

        # sample-level: find contiguous excursions of e_perp beyond designed tolerance mask that fall entirely outside flagged spans
        exceed = ~r.designed_tolerance_mask
        s = r.s_proj
        i = 0
        n = len(s)
        while i < n:
            if exceed[i] and not in_any_flagged(s[i]):
                j = i
                while j + 1 < n and exceed[j + 1] and not in_any_flagged(s[j + 1]):
                    j += 1
                seg_e = r.e_perp[i:j + 1]
                outside_events.append(dict(
                    program=r.pid, s0=float(s[i]), s1=float(s[j]),
                    lateral_max_mm=float(seg_e.max()),
                    kappa=float(np.interp(s[i], r.twin.s_mm, r.twin.kappa)),
                    wrist_manip=float(np.interp(s[i], r.twin.s_mm, r.twin.wrist_manip)),
                ))
                i = j + 1
            else:
                i += 1

    lines = [
        f"Of **{n_flagged_reorientation}** flagged SharpReorientationRegion regions in non-planar programs (P03+P04):",
        f"- **{n_outside_tolerance}** showed measured deviation/speed dip beyond designed tolerance",
        f"- **{n_within_tolerance}** stayed within designed tolerance (conservative flags)",
        f"- **{len(outside_events)}** deviation excursions occurred outside any flagged region",
    ]
    if outside_events:
        table_rows = []
        for e in sorted(outside_events, key=lambda x: -x["lateral_max_mm"])[:10]:
            table_rows.append([e["program"], f"{e['s0']:.1f}-{e['s1']:.1f}", f"{e['lateral_max_mm']:.3f}",
                                f"{e['kappa']:.2f}", f"{e['wrist_manip']:.4f}"])
        lines.append("Top unflagged deviation excursions (by max lateral deviation), with twin descriptors:")
        lines.append(md_table(["Program", "arc span [mm]", "lateral max [mm]", "kappa_DT", "wrist manip_DT"], table_rows))
    return "\n".join(lines), dict(n_flagged=n_flagged_reorientation, n_outside=n_outside_tolerance,
                                    n_within=n_within_tolerance, n_outside_events=len(outside_events))


def d6_figure4_region_numbers(result, region_uri_str):
    obs = result.region_observations[region_uri_str]
    m = obs["metrics"]
    lines = [
        f"Figure 4 region ({result.pid}, {obs['tag']}, span [{obs['region'].start_s_mm:.1f}, "
        f"{obs['region'].end_s_mm:.1f}] mm):",
        f"- $e_\\perp$ RMS: **{m['lateral_rms_mm']:.3f} mm**, max: **{m['lateral_max_mm']:.3f} mm**",
        f"- $e_\\theta$ max: **{m['orientation_max_deg']:.3f} deg**",
        f"- speed dip: **{m['speed_dip_pct']:.1f}%**",
        f"- outside designed blend tolerance: **{m['outside_designed_tolerance']}**",
        f"- exit-ringing RMS ($r_W$, {config.THETA['ringing_window_mm']} mm window): **{m['exit_ringing_rms_mm']:.3f} mm**",
    ]
    return "\n".join(lines)


if __name__ == "__main__":
    import pipeline
    results, merged_graph = pipeline.run_all_programs()
    pipeline.save_kg(merged_graph, config.KG_DIR / "ramp_abox.ttl")
    g = SQ.load_populated_graph(config.KG_DIR / "ramp_abox.ttl")

    print(d4_cq1(g))
    print()
    print(d4_cq5(g))
    print()
    print(d4_cq3(g))
    print()
    txt, stats = d6_aggregate_over_nonplanar(results)
    print(txt)
    print()
    fig4_region_uri = "https://w3id.org/ramp#program_P03_region_SharpReorientationRegion_5"
    print(d6_figure4_region_numbers(results["cladding"], fig4_region_uri))
