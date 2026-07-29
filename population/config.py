"""
Central configuration for the RAMP pipeline.
All thresholds, constants, and file-layout rules live in one place so that
every reported number traces back here.
"""
import os
from pathlib import Path

# ---------------------------------------------------------------------------
# Paths (resolved relative to the repository root = this file's parent's parent)
#
# DATA_ROOT is NOT shipped with this repository -- the raw KRL / RSI /
# digital-twin data package is not public. Point DATA_ROOT at your own data,
# laid out as documented in README.md, or set the RAMP_DATA_ROOT env var.
# outputs/ is created at run time and is git-ignored.
# ---------------------------------------------------------------------------
PROJECT_ROOT = Path(__file__).resolve().parent.parent
DATA_ROOT = Path(os.environ.get("RAMP_DATA_ROOT", str(PROJECT_ROOT / "Data_Package")))
ONTOLOGY_PATH = PROJECT_ROOT / "ontology" / "ramp.rdf"
DHM_PATH = PROJECT_ROOT / "dhm_parameters.txt"
OUT_DIR = PROJECT_ROOT / "outputs"
FIG_DIR = OUT_DIR / "figures"
TABLE_DIR = OUT_DIR / "tables"
KG_DIR = OUT_DIR / "kg"

RAMP_NS = "https://w3id.org/ramp#"

# ---------------------------------------------------------------------------
# Program corpus: folder name -> (Program ID, family, type)
# Primary run selected per user decision: ignore any "Extra"-named folder at
# any depth; keep exactly one primary run per program.
# ---------------------------------------------------------------------------
PROGRAMS = {
    "cube": dict(
        pid="P01", family="cube", ptype="planar",
        folder=DATA_ROOT / "cube_base21",
        main_src="cube_s20cm_h10cm.src",
        motion_srcs=["cube_s20cm_h10cm_0.src"],
        is_multilayer_file=True,   # motion file stacks many 1mm planar layers
        run_dir=DATA_ROOT / "cube_base21" / "base21_v100" / "OR1_30L",
        dt_csv="cube_b21_OR1_l1_v100_dt.csv",
        rsi_dat="rsi_data.dat",
        commanded_speed_mm_s=100,
        orientation_type="OR1",
    ),
    "custom": dict(
        pid="P02", family="custom", ptype="planar",
        folder=DATA_ROOT / "custom_base26",
        main_src="custom_l1_v200_s1.src",
        motion_srcs=["custom_l1_v200_s1_00000.src", "custom_l1_v200_s1_00002.src"],
        is_multilayer_file=True,   # 00000 alone stacks 11 layers (Z 7.2..17.2)
        run_dir=DATA_ROOT / "custom_base26" / "base26_lh1mm_v100" / "OR1_first10last10",
        dt_csv="custom_b26_OR1_l1_s1_v200_dt.csv",
        rsi_dat="rsi_data.dat",
        commanded_speed_mm_s=100,  # v100 = actual speed after override, per description.txt
        orientation_type="OR1",
    ),
    "cladding": dict(
        pid="P03", family="cladding", ptype="non-planar",
        folder=DATA_ROOT / "cladding_base21",
        main_src="cladding.src",
        motion_srcs=None,  # single file = header + motion combined
        is_multilayer_file=False,  # whole file IS one non-planar layer (continuous Z)
        run_dir=DATA_ROOT / "cladding_base21" / "base21_v100" / "OR1",
        dt_csv="cladding_base21_v100_layers1_dt.csv",
        rsi_dat="rsi_data.dat",
        commanded_speed_mm_s=100,
        orientation_type="OR1",
    ),
    "npsrf": dict(
        pid="P04", family="npsrf", ptype="non-planar",
        folder=DATA_ROOT / "npsrf_base21",
        main_src="npsrf.src",
        motion_srcs=None,
        is_multilayer_file=False,  # whole file IS one non-planar layer (continuous Z)
        run_dir=DATA_ROOT / "npsrf_base21" / "base21_v100" / "OR1",
        dt_csv="xxnpsrf_base21_v100_layers1_dt.csv",
        rsi_dat="rsi_data.dat",
        commanded_speed_mm_s=100,
        orientation_type="OR1",
    ),
}

# Program ID ordering used throughout tables/figures (planar first)
PID_ORDER = ["P01", "P02", "P03", "P04"]
PID_TO_KEY = {v["pid"]: k for k, v in PROGRAMS.items()}

EXCLUDE_DIR_NAMES = {"extra"}  # case-insensitive match at any path depth

# ---------------------------------------------------------------------------
# KR 30 HA joint limits (KUKA published datasheet, nameplate values).
# Not derived from the data package; used only as the denominator for the
# per-joint velocity reserve r_i = 1 - |qdot_i|/qdot_i_max.
# Source: KUKA KR 30 HA specification (max axis speeds), disclosed here as an
# external reference, not fitted or fabricated.
# ---------------------------------------------------------------------------
JOINT_VEL_MAX_DEG_S = {
    1: 156.0,
    2: 156.0,
    3: 156.0,
    4: 343.0,
    5: 362.0,
    6: 659.0,
}

# ---------------------------------------------------------------------------
# Region-tagging thresholds (Theta) -- Stage 3. Cell-calibrated, stated as such.
#
# CALIBRATION NOTE: the paper's illustrative CQ1 threshold (twist > 10 deg/mm)
# and a naive joint-reserve floor of 0.15 were checked first against the pooled
# vertex-level twist-rate and DT reserve distributions across all four
# programs (see scratch analysis referenced in results_draft.md numbers
# ledger). At those values SharpReorientationRegion / VelocityLimitedRegion
# never fire on this corpus (max twist rate observed ~4.5 deg/mm, min reserve
# ~0.31-0.51). Primary thresholds below are therefore lowered to the ~90th
# /~10th percentile of the pooled distributions so the taggers produce a
# non-empty, still-meaningful set of flagged regions to carry into the RSI
# corroboration lane (Section 3.2 / CQ3). The paper's original illustrative
# values are kept as THETA_SENSITIVITY below and reported alongside as a
# sensitivity table, so both are traceable and neither is hidden.
# ---------------------------------------------------------------------------
THETA = dict(
    # arc-length sampling step for twin association (Stage 2)
    arc_length_step_mm=1.0,

    # CornerRegion: sharp translational direction change, low orientation change
    corner_angle_deg_c=20.0,          # direction-change threshold at a vertex
    # Upper bound on orientation change to still call it "low". Raised from
    # an initial 5.0 deg: on non-planar programs (P03/P04), genuine ~90 deg
    # raster/turnaround corners carry real orientation drift from the
    # surface-following tilt even at a pure translational reversal --
    # measured median ~5.0 deg, p95 ~14.6 deg, max ~36.2 deg at P03's true
    # direction-change vertices (dir_change > 60 deg). A 5 deg ceiling
    # rejected about half of all genuine corners on P03 for this reason.
    # 15 deg covers the p95 case while staying well below the twist-rate
    # regime that drives SharpReorientationRegion; a vertex can still carry
    # both tags when it genuinely qualifies for each (Region subclasses are
    # intentionally not disjoint).
    corner_max_orient_change_deg=15.0,

    # SharpReorientationRegion: angular twist rate rho(s) = |omega|/|pdot| [deg/mm]
    # lowered from the paper's illustrative 10.0 to the pooled ~90th percentile
    # (0.75 deg/mm) observed across P01-P04; see calibration note above.
    twist_rate_c_deg_per_mm=0.75,

    # NearSingularityRegion
    # q5 never drops below ~54 deg and kappa never exceeds ~5.1 anywhere in
    # this corpus (KR30HA stays in a comfortable mid-workspace posture
    # throughout) -- these thresholds are kept at physically meaningful
    # values rather than loosened to force hits; NearSingularityRegion is
    # expected to be empty and that is reported as a real finding.
    q5_c_deg=5.0,                      # |q5| below this -> near wrist singularity
    kappa_c=50.0,                      # Jacobian condition number ceiling

    # VelocityLimitedRegion
    # lowered from an initial 0.15 floor to the pooled ~10th percentile
    # (~0.87) of DT-derived per-joint reserves; see calibration note above.
    joint_reserve_rc=0.87,

    # SubNozzleFeatureRegion
    nozzle_diameter_mm=4.0,
    # minimum feasible contour radius = nozzle radius (a nozzle of diameter d
    # cannot deposit a convex external contour of radius < d/2 without
    # over/under-extrusion at the inside edge); stated explicitly, not derived.
    min_feasible_radius_mm=2.0,

    # BlendZone: designed approximation interval extent, taken as the active
    # $APO.CDIS at the corner (mm), symmetric around the commanded vertex.

    # CQ1 thresholds: same calibration issue as SharpReorientationRegion above.
    # Paper's illustrative values (twist > 10.0 deg/mm, w_wr < 0.05) never
    # fire on this corpus -- wrist manipulability itself only ranges 2.44-2.51
    # across all four programs (low dynamic range at this workspace region).
    # Primary values lowered to pooled p90 twist / p25 wrist-manipulability;
    # illustrative originals kept in THETA_SENSITIVITY.
    cq1_twist_deg_per_mm=0.75,
    cq1_wrist_manip_w=2.4496,

    # Risk-rule thresholds (Table 1)
    blend_overlap_ratio_c=0.5,          # blend-zone extent / adjacent segment length
    accel_limited_margin=0.0,           # commanded VEL.CP infeasible under ACC.CP margin
    wrist_axis_share_c=0.6,             # fraction of reorientation attributable to A4-A6
    manipulability_c=0.08,              # SingularityProximityRisk ceiling on w_DT
    hybrid_surface_angle_c_deg=15.0,    # substrate surface angle threshold

    # exit-ringing window (rW) length following a flagged feature
    ringing_window_mm=15.0,
)

# ---------------------------------------------------------------------------
# Sensitivity set: the paper's original illustrative thresholds, reported
# alongside THETA (primary) in a sensitivity table (D3/D4 supplement) so the
# lowering documented above is fully transparent and reversible.
# ---------------------------------------------------------------------------
THETA_SENSITIVITY = dict(
    twist_rate_c_deg_per_mm=10.0,
    joint_reserve_rc=0.15,
    cq1_twist_deg_per_mm=10.0,
    cq1_wrist_manip_w=0.05,
)

# Deviation-metric window / smoothing constants
DEVIATION = dict(
    projection_search_window_mm=50.0,   # local search band for monotone projection
    speed_smoothing_window_samples=5,   # simple moving average for RSI speed dip detection
)

RSI_SAMPLE_PERIOD_S = 0.004  # 4 ms
RSI_SAMPLING_RATE_HZ = 250.0
