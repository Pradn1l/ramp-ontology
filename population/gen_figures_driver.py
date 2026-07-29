"""Driver: generate D5 (Figure 4) and D7 (supporting figures) for all
programs, using the validated pipeline results."""
import config
import pipeline
import gen_figures as GF


FIG4_CANDIDATE = dict(program_key="cladding", tag="SharpReorientationRegion", region_index=5)


def find_region(result, tag, index):
    matches = [(ru, t, r) for ru, t, r in result.region_entries if t == tag]
    return matches[index]


def main():
    results, merged_graph = pipeline.run_all_programs()
    pipeline.save_kg(merged_graph, config.KG_DIR / "ramp_abox.ttl")

    # D5 -- Figure 4
    r = results[FIG4_CANDIDATE["program_key"]]
    ru, tag, region = find_region(r, FIG4_CANDIDATE["tag"], FIG4_CANDIDATE["region_index"])
    blend_zones = [reg for _, t, reg in r.region_entries if t == "BlendZone"]
    out_path = config.FIG_DIR / f"fig4_wristload_anatomy_{r.pid}"
    GF.fig4_flagged_region_anatomy(
        r, region, r.s_proj, r.e_perp, r.e_theta, r.v_exec_mm_s, r.v_cmd_mm_s,
        r.rsi_xyz, blend_zones, out_path,
    )
    print(f"Figure 4 written to {out_path}.pdf/.png")

    # D5 (alt) -- two-panel Figure 4 (no path panel), publication layout
    out_path_2p = config.FIG_DIR / f"fig4_wristload_deviation_{r.pid}"
    GF.fig4_two_panel_anatomy(
        r, region, r.s_proj, r.e_perp, r.e_theta, r.v_exec_mm_s, r.v_cmd_mm_s,
        blend_zones, out_path_2p,
    )
    print(f"Figure 4 (two-panel) written to {out_path_2p}.pdf/.png")

    # D7 -- supporting figures
    for key, res in results.items():
        GF.fig_risk_map(res, config.FIG_DIR / f"fig7_riskmap_{res.pid}")
        if res.ptype == "non-planar":
            GF.fig_twist_and_wristmanip(res, config.FIG_DIR / f"fig7_twist_wristmanip_{res.pid}")
            GF.fig_risk_map_3d(res, config.FIG_DIR / f"fig7_riskmap3d_{res.pid}")
        GF.fig_speed_dip_profile(res, config.FIG_DIR / f"fig7_speeddip_{res.pid}")
        GF.fig_joint_reserve_heatmap(res, config.FIG_DIR / f"fig7_jointreserve_{res.pid}")
    print("D7 supporting figures written to", config.FIG_DIR)

    return results, merged_graph


if __name__ == "__main__":
    main()
