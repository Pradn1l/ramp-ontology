"""
Full pipeline driver (plan.md Section 4). Runs Stages 1-5 (a priori lane) and
the physical lane for all four programs, merges per-program graphs into one
ramp_abox.ttl, and returns an in-memory results bundle consumed by the
deliverables/figures/report generation scripts (D1-D8).
"""
import json
import numpy as np
from dataclasses import dataclass, asdict
from pathlib import Path

import config
import krl_parser as KP
import planned_path as PP
import twin_association as TA
import regions as RG
import kg_population as KG
import risk_rules as RR
import rsi_reconstruction as RC
import deviation_metrics as DM


@dataclass
class ProgramResult:
    key: str
    pid: str
    ptype: str
    header: object
    blocks: list
    path: object
    twin: object
    regions: list
    region_entries: list
    risk_flags: list
    accel_limited_block_indices: list
    rsi_available: bool
    rsi_start_idx: int = None
    rsi_end_idx: int = None
    s_proj: np.ndarray = None
    e_perp: np.ndarray = None
    e_theta: np.ndarray = None
    v_exec_mm_s: np.ndarray = None
    v_cmd_mm_s: np.ndarray = None
    dv_pct: np.ndarray = None
    designed_tolerance_mask: np.ndarray = None
    region_observations: dict = None  # region_uri (str) -> metrics dict
    rsi_xyz: np.ndarray = None       # executed Cartesian trace, same length/order as s_proj
    rsi_abc: np.ndarray = None       # executed orientation trace


def run_program(key: str) -> ProgramResult:
    cfg = config.PROGRAMS[key]
    header, blocks = KP.load_program_blocks(key, config.PROGRAMS)
    path = PP.build_planned_path(blocks)
    twin = TA.load_and_associate(cfg["run_dir"] / cfg["dt_csv"], path.total_length_mm)
    regs = RG.tag_all_regions(path, header, twin)

    g = KG.new_graph()
    KG.add_program(g, cfg["pid"], header)
    KG.add_commands(g, cfg["pid"], blocks)
    KG.fill_blend_specs(g, cfg["pid"], header, blocks)
    region_entries = KG.add_regions(g, cfg["pid"], regs)
    KG.link_spans_command(g, cfg["pid"], region_entries, path)

    risk_flags, accel_idx = RR.apply_all_rules(region_entries, blocks, path, header)
    KG.add_risk_flags(g, cfg["pid"], risk_flags)
    KG.add_execution_record(g, cfg["pid"], header, run_label=str(cfg["run_dir"]))

    result = ProgramResult(
        key=key, pid=cfg["pid"], ptype=cfg["ptype"], header=header, blocks=blocks,
        path=path, twin=twin, regions=regs, region_entries=region_entries,
        risk_flags=risk_flags, accel_limited_block_indices=accel_idx,
        rsi_available=False,
    )

    rsi_path = cfg["run_dir"] / cfg["rsi_dat"]
    if rsi_path.exists():
        rsi_rows = RC.load_rsi_dat(rsi_path)
        start_idx, end_idx = RC.find_layer_window(rsi_rows, path)
        run = RC.build_rsi_run(rsi_rows, start_idx, end_idx)
        s_proj, e_perp = RC.monotone_closest_point_projection(run, path)
        e_theta = DM.orientation_deviation_deg(run.abc, path, s_proj)
        v_exec = DM.tcp_speed_mm_s(run.xyz)
        v_cmd = PP.commanded_speed_at_s(path, s_proj) * 1000.0
        dv_pct = DM.speed_dip_pct(v_exec, v_cmd)
        blend_zones = [r for _, tag, r in region_entries if tag == "BlendZone"]
        tol_mask = DM.in_blend_zone_within_tolerance(s_proj, e_perp, blend_zones)

        region_observations = {}
        for ru, tag, r in region_entries:
            agg = DM.aggregate_region_deviation(r, s_proj, e_perp, e_theta, dv_pct, tol_mask)
            if agg is not None:
                region_observations[str(ru)] = dict(region_uri=ru, tag=tag, region=r, metrics=agg)
                obs = KG.add_deviation_observation(g, cfg["pid"], tag, len(region_observations), ru, agg)

        result.rsi_available = True
        result.rsi_start_idx = start_idx
        result.rsi_end_idx = end_idx
        result.s_proj = s_proj
        result.e_perp = e_perp
        result.e_theta = e_theta
        result.v_exec_mm_s = v_exec
        result.v_cmd_mm_s = v_cmd
        result.dv_pct = dv_pct
        result.designed_tolerance_mask = tol_mask
        result.rsi_xyz = run.xyz
        result.rsi_abc = run.abc
        result.region_observations = region_observations

    result.graph = g
    return result


def run_all_programs():
    results = {}
    merged_graph = KG.new_graph()
    for key in config.PROGRAMS:
        r = run_program(key)
        results[key] = r
        merged_graph += r.graph
    return results, merged_graph


def save_kg(merged_graph, out_path: Path):
    out_path.parent.mkdir(parents=True, exist_ok=True)
    merged_graph.serialize(destination=str(out_path), format="turtle")


if __name__ == "__main__":
    results, merged_graph = run_all_programs()
    save_kg(merged_graph, config.KG_DIR / "ramp_abox.ttl")
    for key, r in results.items():
        print(f"{r.pid} ({key}): {len(r.blocks)} blocks, {len(r.regions)} regions, "
              f"{len(r.risk_flags)} risk flags, RSI available: {r.rsi_available}")
    print(f"Total triples in merged graph: {len(merged_graph)}")
