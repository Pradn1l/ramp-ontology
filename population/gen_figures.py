"""
D5/D7 figure generation (plan.md Section 5 D5/D7, Section 6 style rules).
Matplotlib, vector PDF primary + 300dpi PNG copy, single/double-column widths,
8-9pt serif, thin axes, consistent color scheme:
  planned = black solid, executed = colored (blue), blend zones = light gray
  shading, flags/features = warm accent (orange/red).
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

import config
import planned_path as PP

COL_PLANNED = "black"
COL_EXECUTED = "#1f6fb2"      # blue
COL_BLENDZONE = "#d9d9d9"     # light gray shading
COL_FLAG = "#d95f02"          # warm accent (orange)
COL_FLAG2 = "#a6191e"         # secondary warm accent (red), if two flag kinds needed

RC_STYLE = {
    "font.family": "serif",
    "font.size": 8.5,
    "axes.linewidth": 0.6,
    "xtick.major.width": 0.6,
    "ytick.major.width": 0.6,
    "axes.grid": False,
    "legend.frameon": False,
    "savefig.dpi": 300,
    "pdf.fonttype": 42,
}


def _apply_style():
    plt.rcParams.update(RC_STYLE)


def _save(fig, path_no_ext):
    fig.savefig(str(path_no_ext) + ".pdf")
    fig.savefig(str(path_no_ext) + ".png", dpi=300)
    plt.close(fig)


TOP_PANEL_AXIS_INDEX = dict(X=0, Y=1, Z=2)


def fig4_flagged_region_anatomy(result, region, s_proj, e_perp, e_theta, v_exec, v_cmd,
                                 run_xyz, blend_zones, out_path, pad_mm=20.0, dpi_col_width_in=7.0,
                                 top_panel_axes=("X", "Z")):
    """
    D5: three stacked panels sharing arc-length axis, for one flagged region.
      1. Top: planned path a vs executed path c, 2D projection onto
         top_panel_axes (default X-Z: for a non-planar/surface-following
         program the tool-height variation that makes the path non-planar
         lives in Z, and an X-Y projection compresses it away; X-Z shows the
         actual surface-following tilt/undulation the region sits on).
         Blend zone shaded.
      2. Middle: e_perp and e_theta vs arc length.
      3. Bottom: executed TCP speed vs commanded constant.
    Caption fact preserved outside the figure (in results_draft.md): the flag
    was raised a priori, before this trace existed.
    """
    _apply_style()
    path = result.path
    s0, s1 = region.start_s_mm - pad_mm, region.end_s_mm + pad_mm

    window_mask = (s_proj >= s0) & (s_proj <= s1)
    s_win = s_proj[window_mask]

    s_dense = np.linspace(max(s0, 0), min(s1, path.total_length_mm), 400)
    xyz_planned = PP.position_at_s(path, s_dense)

    i0, i1 = TOP_PANEL_AXIS_INDEX[top_panel_axes[0]], TOP_PANEL_AXIS_INDEX[top_panel_axes[1]]

    fig, axes = plt.subplots(3, 1, figsize=(dpi_col_width_in, 6.2), sharex=False,
                              gridspec_kw=dict(height_ratios=[1.3, 1, 1], hspace=0.45))

    ax0 = axes[0]
    for bz in blend_zones:
        if bz.end_s_mm >= s0 and bz.start_s_mm <= s1:
            s_bz = np.linspace(max(bz.start_s_mm, s0), min(bz.end_s_mm, s1), 10)
            xyz_bz = PP.position_at_s(path, s_bz)
            ax0.plot(xyz_bz[:, i0], xyz_bz[:, i1], color=COL_BLENDZONE, lw=4.0, solid_capstyle="round", zorder=0)
    ax0.plot(xyz_planned[:, i0], xyz_planned[:, i1], color=COL_PLANNED, lw=1.0, label="planned path $a$", zorder=2)
    ax0.plot(run_xyz[window_mask, i0], run_xyz[window_mask, i1], color=COL_EXECUTED, lw=0.8,
             label="executed path $c$ (RSI)", zorder=3)
    ax0.set_xlabel(f"{top_panel_axes[0]} [mm]")
    ax0.set_ylabel(f"{top_panel_axes[1]} [mm]")
    ax0.set_title(f"{result.pid} -- region {region.tag} [{region.start_s_mm:.1f}, {region.end_s_mm:.1f}] mm", fontsize=8.5)
    ax0.legend(loc="best", fontsize=7)

    ax1 = axes[1]
    for bz in blend_zones:
        if bz.end_s_mm >= s0 and bz.start_s_mm <= s1:
            ax1.axvspan(max(bz.start_s_mm, s0), min(bz.end_s_mm, s1), color=COL_BLENDZONE, lw=0)
    ax1.axvspan(region.start_s_mm, region.end_s_mm, color=COL_FLAG, alpha=0.12, lw=0)
    ax1b = ax1.twinx()
    ax1.plot(s_win, e_perp[window_mask], color=COL_EXECUTED, lw=0.9, label=r"$e_\perp$ [mm]")
    ax1b.plot(s_win, e_theta[window_mask], color=COL_FLAG2, lw=0.9, ls="--", label=r"$e_\theta$ [deg]")
    ax1.set_xlim(s0, s1)
    ax1.set_xlabel("arc length $s$ [mm]")
    ax1.set_ylabel(r"$e_\perp$ [mm]", color=COL_EXECUTED)
    ax1b.set_ylabel(r"$e_\theta$ [deg]", color=COL_FLAG2)
    ax1.tick_params(axis="y", colors=COL_EXECUTED)
    ax1b.tick_params(axis="y", colors=COL_FLAG2)

    ax2 = axes[2]
    for bz in blend_zones:
        if bz.end_s_mm >= s0 and bz.start_s_mm <= s1:
            ax2.axvspan(max(bz.start_s_mm, s0), min(bz.end_s_mm, s1), color=COL_BLENDZONE, lw=0)
    ax2.axvspan(region.start_s_mm, region.end_s_mm, color=COL_FLAG, alpha=0.12, lw=0)
    ax2.plot(s_win, v_cmd[window_mask], color=COL_PLANNED, lw=0.9, ls=":", label="commanded $v_{cmd}$")
    ax2.plot(s_win, v_exec[window_mask], color=COL_EXECUTED, lw=0.9, label="executed TCP speed")
    ringing_end = region.end_s_mm + config.THETA["ringing_window_mm"]
    ax2.axvspan(region.end_s_mm, min(ringing_end, s1), color=COL_FLAG2, alpha=0.08, lw=0)
    ax2.set_xlim(s0, s1)
    ax2.set_xlabel("arc length $s$ [mm]")
    ax2.set_ylabel("$v$ [mm/s]")
    ax2.legend(loc="best", fontsize=7)

    fig.align_ylabels([ax1, ax2])
    _save(fig, out_path)


def fig4_two_panel_anatomy(result, region, s_proj, e_perp, e_theta, v_exec, v_cmd,
                            blend_zones, out_path, pad_mm=20.0, fig_width_in=5.5,
                            show_blend_zones=True):
    """
    Publication two-panel variant of Figure 4 (no path panel):
      (a) lateral e_perp and orientation e_theta deviation vs arc length.
      (b) executed TCP speed vs commanded constant, with the exit-ringing window.
    Both panels share the arc-length axis (labelled once, at the bottom). The
    flagged region is highlighted; blend zones give designed-tolerance context.
    Vector PDF + 300 dpi PNG, serif, thin recessive spines -- MDPI-ready.
    """
    _apply_style()
    plt.rcParams.update({"font.size": 9.0})

    s0, s1 = region.start_s_mm - pad_mm, region.end_s_mm + pad_mm
    window_mask = (s_proj >= s0) & (s_proj <= s1)
    s_win = s_proj[window_mask]

    fig, (ax_a, ax_b) = plt.subplots(
        2, 1, sharex=True, figsize=(fig_width_in, 3.9),
        gridspec_kw=dict(height_ratios=[1, 1], hspace=0.12),
    )

    def _shade_context(ax):
        # designed blend zones: light-gray context bands (behind everything)
        if show_blend_zones:
            for bz in blend_zones:
                if bz.end_s_mm >= s0 and bz.start_s_mm <= s1:
                    ax.axvspan(max(bz.start_s_mm, s0), min(bz.end_s_mm, s1),
                               color=COL_BLENDZONE, alpha=0.5, lw=0, zorder=0)
        # flagged region: light warm fill + crisp dashed boundaries so it stays
        # legible even where it overlaps a gray blend band (a heavier fill
        # muddies to brown over the gray).
        ax.axvspan(region.start_s_mm, region.end_s_mm, color=COL_FLAG, alpha=0.10,
                   lw=0, zorder=0.5)
        for s_edge in (region.start_s_mm, region.end_s_mm):
            ax.axvline(s_edge, color=COL_FLAG, lw=0.9, ls=(0, (4, 2)), zorder=0.6)

    # --- Panel (a): deviation, dual axis (units differ: mm vs deg) ---
    _shade_context(ax_a)
    ax_a2 = ax_a.twinx()
    l1, = ax_a.plot(s_win, e_perp[window_mask], color=COL_EXECUTED, lw=1.1,
                    label=r"$e_\perp$ (lateral)")
    l2, = ax_a2.plot(s_win, e_theta[window_mask], color=COL_FLAG2, lw=1.1, ls="--",
                     label=r"$e_\theta$ (orientation)")
    ax_a.set_ylabel(r"$e_\perp$ [mm]", color=COL_EXECUTED)
    ax_a2.set_ylabel(r"$e_\theta$ [$^\circ$]", color=COL_FLAG2)
    ax_a.tick_params(axis="y", colors=COL_EXECUTED)
    ax_a2.tick_params(axis="y", colors=COL_FLAG2)
    ax_a.spines["left"].set_color(COL_EXECUTED)
    ax_a2.spines["right"].set_color(COL_FLAG2)
    ax_a2.spines["top"].set_visible(False)
    ax_a.legend(handles=[l1, l2], loc="upper left", bbox_to_anchor=(0.0, 0.86),
                fontsize=7.5, framealpha=0.9, edgecolor="none")
    ax_a.set_ylim(0, 1.15 * max(e_perp[window_mask].max(), 1e-6))
    ax_a2.set_ylim(bottom=0)

    # --- Panel (b): TCP speed ---
    _shade_context(ax_b)
    ax_b.plot(s_win, v_cmd[window_mask], color=COL_PLANNED, lw=1.0, ls=":",
              label=r"commanded $v_\mathrm{cmd}$")
    ax_b.plot(s_win, v_exec[window_mask], color=COL_EXECUTED, lw=1.1,
              label="executed TCP speed")
    ax_b.set_ylabel(r"$v$ [mm/s]")
    ax_b.set_xlabel(r"arc length $s$ [mm]")
    ax_b.set_xlim(s0, s1)
    ax_b.set_ylim(0, max(v_cmd[window_mask].max(), v_exec[window_mask].max()) * 1.18)
    ax_b.legend(loc="lower right", fontsize=7.5, framealpha=0.9, edgecolor="none")

    # exit-ringing window: a subtle bracket above the traces (not a bold fill
    # -- a full hatch here overwhelms the speed-dip feature it sits next to).
    ringing_end = min(region.end_s_mm + config.THETA["ringing_window_mm"], s1)
    y_br = ax_b.get_ylim()[1] * 0.93
    ax_b.annotate("", xy=(ringing_end, y_br), xytext=(region.end_s_mm, y_br),
                  arrowprops=dict(arrowstyle="|-|,widthA=0.4,widthB=0.4",
                                  color=COL_FLAG2, lw=0.9))
    ax_b.text(0.5 * (region.end_s_mm + ringing_end), y_br * 0.985,
              r"exit-ringing window ($r_W$)", color=COL_FLAG2, fontsize=6.8,
              ha="center", va="top")

    # panel tags
    for ax, tag in ((ax_a, "(a)"), (ax_b, "(b)")):
        ax.annotate(tag, xy=(0.015, 0.965), xycoords="axes fraction",
                    fontsize=9, fontweight="bold", va="top", ha="left")

    # recessive spines
    for ax in (ax_a, ax_a2, ax_b):
        for s in ax.spines.values():
            s.set_linewidth(0.6)

    fig.align_ylabels([ax_a, ax_b])
    fig.subplots_adjust(left=0.11, right=0.89, top=0.98, bottom=0.11)
    _save(fig, out_path)
    plt.rcParams.update({"font.size": 8.5})  # restore module default


def fig_risk_map(result, out_path, dpi_col_width_in=3.5):
    """D7: per-program overhead plot of the planned layer, regions
    color-coded by tag."""
    _apply_style()
    path = result.path
    tag_colors = {
        "CornerRegion": "#7570b3",
        "SharpReorientationRegion": COL_FLAG,
        "NearSingularityRegion": COL_FLAG2,
        "VelocityLimitedRegion": "#1b9e77",
        "SubNozzleFeatureRegion": "#e7298a",
        "BlendZone": COL_BLENDZONE,
    }
    fig, ax = plt.subplots(figsize=(dpi_col_width_in * 1.55, dpi_col_width_in))
    s_dense = np.linspace(0, path.total_length_mm, 2000)
    xyz = PP.position_at_s(path, s_dense)
    ax.plot(xyz[:, 0], xyz[:, 1], color=COL_PLANNED, lw=0.8, zorder=1)

    seen_tags = set()
    for _, tag, r in result.region_entries:
        if tag == "BlendZone":
            continue  # too dense to usefully overlay on the risk map
        s_seg = np.linspace(r.start_s_mm, r.end_s_mm, 20)
        seg_xyz = PP.position_at_s(path, s_seg)
        label = tag if tag not in seen_tags else None
        seen_tags.add(tag)
        ax.plot(seg_xyz[:, 0], seg_xyz[:, 1], color=tag_colors.get(tag, "gray"), lw=2.2,
                solid_capstyle="round", label=label, zorder=2)

    ax.set_xlabel("X [mm]")
    ax.set_ylabel("Y [mm]")
    ax.set_title(f"{result.pid} risk map", fontsize=8.5)
    ax.set_aspect("equal", adjustable="datalim")
    if seen_tags:
        ax.legend(loc="center left", bbox_to_anchor=(1.02, 0.5), fontsize=6.5, borderaxespad=0.0)
    fig.tight_layout()
    _save(fig, out_path)


RISK_MAP_TAG_COLORS = {
    "CornerRegion": "#7570b3",
    "SharpReorientationRegion": COL_FLAG,
    "NearSingularityRegion": COL_FLAG2,
    "VelocityLimitedRegion": "#1b9e77",
    "SubNozzleFeatureRegion": "#e7298a",
    "BlendZone": COL_BLENDZONE,
}


def fig_risk_map_3d(result, out_path, dpi_col_width_in=5.0, elev=22, azim=-60):
    """
    D7 (non-planar programs): 3D risk map showing the true X-Y-Z surface the
    toolpath follows, regions color-coded by tag. A 2D X-Y overhead
    projection (fig_risk_map) flattens away exactly the Z variation that
    makes these programs non-planar; this view keeps it.
    """
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (registers 3D projection)

    _apply_style()
    path = result.path
    fig = plt.figure(figsize=(dpi_col_width_in * 1.5, dpi_col_width_in))
    ax = fig.add_subplot(111, projection="3d")

    s_dense = np.linspace(0, path.total_length_mm, 3000)
    xyz = PP.position_at_s(path, s_dense)
    ax.plot(xyz[:, 0], xyz[:, 1], xyz[:, 2], color=COL_PLANNED, lw=0.7, zorder=1)

    seen_tags = set()
    for _, tag, r in result.region_entries:
        if tag == "BlendZone":
            continue  # too dense to usefully overlay on the risk map
        s_seg = np.linspace(r.start_s_mm, r.end_s_mm, 20)
        seg_xyz = PP.position_at_s(path, s_seg)
        label = tag if tag not in seen_tags else None
        seen_tags.add(tag)
        ax.plot(seg_xyz[:, 0], seg_xyz[:, 1], seg_xyz[:, 2],
                color=RISK_MAP_TAG_COLORS.get(tag, "gray"), lw=2.4,
                solid_capstyle="round", label=label, zorder=2)

    ax.set_xlabel("X [mm]", labelpad=6)
    ax.set_ylabel("Y [mm]", labelpad=6)
    ax.set_zlabel("Z [mm]", labelpad=2)
    ax.set_title(f"{result.pid} risk map (3D)", fontsize=8.5)
    ax.view_init(elev=elev, azim=azim)

    # Tight per-axis limits (a small pad, not forced to a common range) plus
    # set_box_aspect with the true X:Y:Z extents, so mm-per-inch is the same
    # on every axis (no artificial stretching) without wasting plot area on
    # an oversized box when one axis (e.g. Z on a thin non-planar layer) spans
    # much less than the others.
    ranges = xyz.max(axis=0) - xyz.min(axis=0)
    pad = ranges.max() * 0.05
    mins, maxs = xyz.min(axis=0) - pad, xyz.max(axis=0) + pad
    ax.set_xlim(mins[0], maxs[0])
    ax.set_ylim(mins[1], maxs[1])
    ax.set_zlim(mins[2], maxs[2])
    ax.set_box_aspect(maxs - mins)

    if seen_tags:
        ax.legend(loc="center left", bbox_to_anchor=(1.05, 0.5), fontsize=6.5, borderaxespad=0.0)
    fig.tight_layout()
    _save(fig, out_path)


def fig_twist_and_wristmanip(result, out_path, dpi_col_width_in=7.0):
    """D7: rho(s) and w_wr_DT(s) along arc length, with thresholds as
    horizontal lines."""
    _apply_style()
    twin = result.twin
    fig, axes = plt.subplots(2, 1, figsize=(dpi_col_width_in, 4.0), sharex=True)
    ax0, ax1 = axes

    path = result.path
    n = len(path.xyz_targets)
    s_vertex = []
    twist_vertex = []
    import regions as RG
    for i in range(1, n - 1):
        oc = RG._vertex_orientation_change_deg(path, i)
        translation = np.linalg.norm(path.xyz_targets[i + 1] - path.xyz_targets[i - 1])
        tw = oc / translation if translation > 1e-6 else 0.0
        s_vertex.append(path.s_targets[i])
        twist_vertex.append(tw)

    ax0.plot(s_vertex, twist_vertex, color=COL_EXECUTED, lw=0.9)
    ax0.axhline(config.THETA["twist_rate_c_deg_per_mm"], color=COL_FLAG, lw=0.8, ls="--",
                label=r"$\rho_c$ (primary)")
    ax0.axhline(config.THETA_SENSITIVITY["twist_rate_c_deg_per_mm"], color=COL_FLAG2, lw=0.8, ls=":",
                label=r"$\rho_c$ (paper-illustrative)")
    ax0.set_ylabel(r"$\rho(s)$ [deg/mm]")
    ax0.legend(loc="best", fontsize=6.5)

    ax1.plot(twin.s_mm, twin.wrist_manip, color=COL_EXECUTED, lw=0.9)
    ax1.axhline(config.THETA["cq1_wrist_manip_w"], color=COL_FLAG, lw=0.8, ls="--", label=r"$w_{wr,c}$ (primary)")
    ax1.set_xlabel("arc length $s$ [mm]")
    ax1.set_ylabel(r"$w_{wr,DT}(s)$")
    ax1.legend(loc="best", fontsize=6.5)

    fig.suptitle(f"{result.pid}: angular twist rate and wrist manipulability", fontsize=8.5)
    _save(fig, out_path)


def fig_speed_dip_profile(result, out_path, dpi_col_width_in=7.0):
    """D7: delta_v profile along arc length with flagged regions shaded."""
    if not result.rsi_available:
        return
    _apply_style()
    fig, ax = plt.subplots(figsize=(dpi_col_width_in, 2.6))
    for _, tag, r in result.region_entries:
        if tag == "BlendZone":
            continue
        ax.axvspan(r.start_s_mm, r.end_s_mm, color=COL_FLAG, alpha=0.10, lw=0)
    ax.plot(result.s_proj, result.dv_pct, color=COL_EXECUTED, lw=0.7)
    ax.axhline(0, color=COL_PLANNED, lw=0.5)
    ax.set_xlabel("arc length $s$ [mm]")
    ax.set_ylabel(r"$\delta v$ [%]")
    ax.set_title(f"{result.pid}: speed-dip profile", fontsize=8.5)
    _save(fig, out_path)


def fig_joint_reserve_heatmap(result, out_path, dpi_col_width_in=7.0):
    """D7 (optional): joint-reserve heatmap (joints x arc length) from DT
    data."""
    _apply_style()
    twin = result.twin
    fig, ax = plt.subplots(figsize=(dpi_col_width_in, 2.6))
    im = ax.imshow(twin.joint_reserves.T, aspect="auto", cmap="RdYlGn", vmin=0, vmax=1,
                    extent=[twin.s_mm.min(), twin.s_mm.max(), 6.5, 0.5])
    ax.set_yticks(range(1, 7))
    ax.set_xlabel("arc length $s$ [mm]")
    ax.set_ylabel("joint")
    cbar = fig.colorbar(im, ax=ax, pad=0.02)
    cbar.set_label(r"reserve $r_i$")
    ax.set_title(f"{result.pid}: joint velocity reserve", fontsize=8.5)
    _save(fig, out_path)
