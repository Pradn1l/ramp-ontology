"""
Stage 1 -- KRL block extraction (plan.md Section 3a, Section 4 Stage 1).

Parses the header of a program's main .src file for blending/dynamics
parameters ($APO.*, $VEL.*, $ACC.*, $ADVANCE, $ORI_TYPE, $JERK.*, BAS tool/base),
and parses the motion file(s) for an ordered list of motion blocks
(LIN / SLIN / PTP) with target pose and commanded speed.

Only the first layer is kept: the first block whose Z target differs from the
previous motion block's Z is treated as the start of the next layer, and
everything from that block onward is discarded.
"""
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional


HEADER_PATTERNS = {
    "apo_cdis_mm": re.compile(r"\$APO\.CDIS\s*=\s*([\-\d.]+)"),
    "apo_cvel_pct": re.compile(r"\$APO\.CVEL\s*=?\s*([\-\d.]+)"),
    "apo_cori_deg": re.compile(r"\$APO\.CORI\s*=\s*([\-\d.]+)"),
    "vel_cp": re.compile(r"\$VEL\.CP\s*=\s*([\-\d.]+)"),
    "acc_cp_pct": re.compile(r"\$ACC\.CP\s*=\s*([\-\d.]+)"),
    "advance_blocks": re.compile(r"\$ADVANCE\s*=\s*([\-\d.]+)"),
    "ori_type": re.compile(r"\$ORI_TYPE\s*=\s*#(\w+)"),
    "jerk_cp_pct": re.compile(r"\$JERK\.CP\s*=\s*([\-\d.]+)"),
    "jerk_ori_pct": re.compile(r"\$JERK\.ORI\s*=\s*([\-\d.]+)"),
    "tool_id": re.compile(r"BAS\s*\(\s*#TOOL\s*,\s*(\d+)\s*\)"),
    "base_id": re.compile(r"BAS\s*\(\s*#BASE\s*,\s*(\d+)\s*\)"),
}

MOTION_LINE_RE = re.compile(
    r"^\s*(LIN|SLIN|PTP)\s*\{([^}]*)\}(?:\s*WITH\s*\$VEL\s*=\s*\{([^}]*)\})?\s*(C_VEL|C_SPL|C_PTP|C_DIS|C_ORI)?\s*$"
)
INLINE_VEL_CP_RE = re.compile(r"\$VEL\.CP\s*=\s*([\-\d.]+)")
POSE_FIELD_RE = re.compile(r"([A-Z][A-Za-z0-9]*)\s+(-?[\d.]+)")


@dataclass
class HeaderParams:
    apo_cdis_mm: Optional[float] = None
    apo_cvel_pct: Optional[float] = None
    apo_cori_deg: Optional[float] = None
    vel_cp: Optional[float] = None
    acc_cp_pct: Optional[float] = None
    advance_blocks: Optional[int] = None
    ori_type: Optional[str] = None
    jerk_cp_pct: Optional[float] = None
    jerk_ori_pct: Optional[float] = None
    tool_id: Optional[int] = None
    base_id: Optional[int] = None


@dataclass
class MotionBlock:
    block_index: int
    source_file: str
    source_line: int
    motion_type: str  # LIN | SLIN | PTP
    x: float
    y: float
    z: float
    a: float
    b: float
    c: float
    commanded_vel_cp: float  # m/s as written in KRL ($VEL.CP), in force at this block
    blend_switch: Optional[str]  # C_VEL | C_SPL | C_PTP | None


def parse_header(main_src_path: Path) -> HeaderParams:
    text = main_src_path.read_text(encoding="utf-8", errors="replace")
    hp = HeaderParams()
    for field_name, pattern in HEADER_PATTERNS.items():
        m = pattern.search(text)
        if m:
            val = m.group(1)
            if field_name in ("advance_blocks", "tool_id", "base_id"):
                setattr(hp, field_name, int(float(val)))
            elif field_name == "ori_type":
                setattr(hp, field_name, val)
            else:
                setattr(hp, field_name, float(val))
    return hp


def _parse_pose_fields(body: str) -> dict:
    fields = {}
    for name, val in POSE_FIELD_RE.findall(body):
        fields[name] = float(val)
    return fields


def parse_motion_file(src_path: Path, start_index: int = 0, current_vel_cp: float = None):
    """
    Parse one motion .src file into an ordered list of MotionBlock, tracking
    the in-force $VEL.CP as it changes line-by-line (KRL semantics: a bare
    $VEL.CP=... line sets the commanded speed for all following blocks until
    changed again).

    Returns (blocks, ending_vel_cp) so multi-file programs (custom) can chain.
    """
    blocks = []
    vel_cp = current_vel_cp
    idx = start_index
    with open(src_path, "r", encoding="utf-8", errors="replace") as f:
        for lineno, raw_line in enumerate(f, start=1):
            line = raw_line.strip()
            if not line:
                continue
            m_vel = INLINE_VEL_CP_RE.match(line)
            if m_vel:
                vel_cp = float(m_vel.group(1))
                continue
            m = MOTION_LINE_RE.match(line)
            if not m:
                continue
            motion_type, body, _with_vel, blend_switch = m.groups()
            pf = _parse_pose_fields(body)
            if not all(k in pf for k in ("X", "Y", "Z")):
                continue
            block = MotionBlock(
                block_index=idx,
                source_file=src_path.name,
                source_line=lineno,
                motion_type=motion_type,
                x=pf.get("X"), y=pf.get("Y"), z=pf.get("Z"),
                a=pf.get("A", 0.0), b=pf.get("B", 0.0), c=pf.get("C", 0.0),
                commanded_vel_cp=vel_cp if vel_cp is not None else float("nan"),
                blend_switch=blend_switch,
            )
            blocks.append(block)
            idx += 1
    return blocks, vel_cp


def truncate_to_first_layer(blocks):
    """
    Discard all blocks from the first Z-target change onward (plan.md: 'detect
    the layer change as the first change in Z target value'). The block whose
    Z first differs from the immediately preceding block's Z is the start of
    the discarded region; everything from it onward is dropped.
    Comparison is on raw Z target values (deposition height changes are >=
    layer-height scale, so exact float compare on parsed values is safe here).

    Applies only to families whose main motion file stacks multiple planar
    layers back to back (cube, custom). npsrf and cladding are non-planar:
    their single .src file already IS one layer, with Z varying continuously
    as the tool follows a curved surface -- there is no discrete layer jump
    to detect, and applying this rule there would truncate after 1 block.
    Callers select the right behavior via `is_multilayer_file`.
    """
    if not blocks:
        return blocks
    z0 = blocks[0].z
    for i, b in enumerate(blocks):
        if b.z != z0:
            return blocks[:i]
    return blocks


def load_program_blocks(program_key: str, programs_config: dict):
    """
    Top-level Stage-1 entry point for one program key from config.PROGRAMS.
    Returns (header_params, first_layer_blocks_reindexed).
    """
    cfg = programs_config[program_key]
    header = parse_header(cfg["folder"] / cfg["main_src"])

    if cfg["motion_srcs"] is None:
        # single file = header + motion combined (cladding, npsrf)
        all_blocks, _ = parse_motion_file(cfg["folder"] / cfg["main_src"])
    else:
        all_blocks = []
        vel_cp = header.vel_cp
        idx = 0
        for motion_src in cfg["motion_srcs"]:
            blocks, vel_cp = parse_motion_file(
                cfg["folder"] / motion_src, start_index=idx, current_vel_cp=vel_cp
            )
            all_blocks.extend(blocks)
            idx += len(blocks)

    if cfg.get("is_multilayer_file", True):
        first_layer = truncate_to_first_layer(all_blocks)
    else:
        first_layer = all_blocks
    # reindex block_index to be contiguous 0..N-1 within the retained layer
    for new_idx, b in enumerate(first_layer):
        b.block_index = new_idx
    return header, first_layer
