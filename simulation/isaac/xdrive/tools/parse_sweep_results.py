#!/usr/bin/env python3
"""Parse and compare FORTIS arm pose sweep results.

Plain Python + matplotlib — no Isaac Sim dependency.
Run after sweep scripts complete:
    python tools/parse_sweep_results.py
"""
import os
import sys
import json
import glob

import numpy as np

XDRIVE_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
RESULTS_DIR = os.path.join(XDRIVE_ROOT, "results", "arm_pose_sweep")
COMBINED_DIR = os.path.join(RESULTS_DIR, "combined")

# Motor / gearbox reference lines
GB_RATED = 11.0   # Cricket MK II rated torque (Nm)
MOT_MAX  = 20.0   # Motor x ratio absolute max (Nm)

# Colour and style maps for configs
ARM_COLORS = {
    "36": {"unloaded": "#ef5350", "loaded": "#b71c1c"},   # red family
    "30": {"unloaded": "#42a5f5", "loaded": "#0d47a1"},   # blue family
    "24": {"unloaded": "#66bb6a", "loaded": "#1b5e20"},   # green family
}

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _parse_config_name(name):
    """Extract arm length, load state, and environment from a directory name.

    Expected patterns:
        36arm_unloaded_step
        30arm_loaded_reactor
    Returns dict with keys: arm_in, load, env  (or None values if unparseable).
    """
    parts = name.split("_")
    arm_in = None
    load = None
    env = None
    if len(parts) >= 3:
        # First token: e.g. "36arm"
        token = parts[0]
        if token.endswith("arm"):
            try:
                arm_in = int(token[:-3])
            except ValueError:
                pass
        load = parts[1] if parts[1] in ("loaded", "unloaded") else None
        env = parts[2] if parts[2] in ("step", "reactor") else None
    return {"arm_in": arm_in, "load": load, "env": env}


def _load_summaries():
    """Scan RESULTS_DIR for summary.json files.

    Returns ordered list of (config_name, summary_dict, parsed_meta) tuples.
    """
    pattern = os.path.join(RESULTS_DIR, "*", "summary.json")
    paths = sorted(glob.glob(pattern))
    results = []
    for p in paths:
        config_name = os.path.basename(os.path.dirname(p))
        if config_name == "combined":
            continue
        try:
            with open(p, "r") as f:
                data = json.load(f)
            meta = _parse_config_name(config_name)
            results.append((config_name, data, meta))
            print(f"  [OK]  {config_name}")
        except Exception as exc:
            print(f"  [ERR] {config_name}: {exc}")
    return results


def _safe_get(d, *keys, default=None):
    """Nested dict access that never raises."""
    cur = d
    for k in keys:
        if isinstance(cur, dict):
            cur = cur.get(k)
        else:
            return default
        if cur is None:
            return default
    return cur


def _fmt_nm(val):
    """Format a torque value or return '—' if missing."""
    if val is None:
        return "     —"
    return f"{val:6.2f}Nm"


def _tip_pct(summary):
    """Compute aggregate tipping percentage across all J1 angles."""
    tipping = summary.get("tipping")
    if not tipping:
        return None
    total_tip = sum(t.get("n_tip", 0) for t in tipping)
    total_ok  = sum(t.get("n_ok", 0) for t in tipping)
    if total_ok == 0:
        return None
    return 100.0 * total_tip / total_ok


def _contact_pct(summary):
    """Compute reactor contact percentage if available."""
    contact = summary.get("reactor_contact")
    if not contact:
        return None
    n_contact = contact.get("n_contact", 0)
    n_total   = contact.get("n_total", 0)
    if n_total == 0:
        return None
    return 100.0 * n_contact / n_total


def _n_poses(summary):
    """Total valid pose count."""
    grid = summary.get("grid", {})
    return grid.get("n_valid", grid.get("n_total", None))


# ---------------------------------------------------------------------------
# Table
# ---------------------------------------------------------------------------

def print_comparison_table(configs):
    """Print formatted comparison table to stdout."""
    if not configs:
        print("\nNo configs to compare.\n")
        return

    hdr = (f"{'Config':<26}| {'J2 peak':>8} | {'J3 peak':>8} | "
           f"{'J4 peak':>8} | {'Tip%':>5} | {'Contact%':>9} | {'Poses':>6}")
    sep = "-" * len(hdr)

    print(f"\n{'=' * len(hdr)}")
    print("FORTIS Arm Pose Sweep — Comparison")
    print(f"{'=' * len(hdr)}\n")
    print(hdr)
    print(sep)

    for name, data, _meta in configs:
        j2 = _safe_get(data, "peak_torques", "J2", "peak_Nm")
        j3 = _safe_get(data, "peak_torques", "J3", "peak_Nm")
        j4 = _safe_get(data, "peak_torques", "J4", "peak_Nm")
        tip = _tip_pct(data)
        contact = _contact_pct(data)
        poses = _n_poses(data)

        tip_str = f"{tip:5.1f}%" if tip is not None else "    —"
        con_str = f"{contact:8.1f}%" if contact is not None else "       —"
        pos_str = f"{poses:6d}" if poses is not None else "     —"

        print(f"{name:<26}| {_fmt_nm(j2)} | {_fmt_nm(j3)} | "
              f"{_fmt_nm(j4)} | {tip_str} | {con_str} | {pos_str}")

    print(sep)
    print()


# ---------------------------------------------------------------------------
# Charts
# ---------------------------------------------------------------------------

def _import_matplotlib():
    """Import matplotlib with Agg backend; return (plt, True) or (None, False)."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        return plt, True
    except ImportError:
        print("\n[WARN] matplotlib not available — skipping chart generation.")
        print("       Install with:  pip install matplotlib\n")
        return None, False


def plot_torque_comparison(configs, out_dir, plt):
    """Grouped bar chart: peak torque per joint, one bar per config."""
    joints = ["J1", "J2", "J3", "J4"]
    n_joints = len(joints)
    n_configs = len(configs)

    if n_configs == 0:
        return

    # Gather data
    peaks = {}  # config_name -> [J1, J2, J3, J4]
    for name, data, _meta in configs:
        row = []
        for j in joints:
            val = _safe_get(data, "peak_torques", j, "peak_Nm")
            row.append(val if val is not None else 0.0)
        peaks[name] = row

    # Bar positions
    x = np.arange(n_joints)
    width = 0.8 / max(n_configs, 1)

    fig, ax = plt.subplots(figsize=(max(10, n_configs * 1.2), 7))

    for i, (name, data, meta) in enumerate(configs):
        arm_str = str(meta.get("arm_in", "??"))
        load = meta.get("load", "unloaded")
        env  = meta.get("env", "step")

        # Pick colour
        arm_colors = ARM_COLORS.get(arm_str, {"unloaded": "#888888", "loaded": "#444444"})
        color = arm_colors.get(load, "#666666")

        # Hatch for reactor
        hatch = "//" if env == "reactor" else None

        offset = (i - n_configs / 2.0 + 0.5) * width
        vals = peaks[name]
        bars = ax.bar(x + offset, vals, width, label=name, color=color,
                      hatch=hatch, edgecolor="black", linewidth=0.6, alpha=0.85)

    # Reference lines
    ax.axhline(GB_RATED, color="orange", ls="--", lw=2, label=f"Cricket MK II rated ({GB_RATED} Nm)")
    ax.axhline(MOT_MAX,  color="red",    ls=":",  lw=1.5, label=f"Absolute max ({MOT_MAX} Nm)")

    ax.set_xticks(x)
    ax.set_xticklabels(joints, fontsize=12)
    ax.set_ylabel("Peak |torque| (Nm)", fontsize=11)
    ax.set_title("FORTIS Arm Pose Sweep — Peak Torque Comparison", fontsize=13)
    ax.legend(fontsize=7, loc="upper left", ncol=2, framealpha=0.9)
    ax.grid(True, alpha=0.3, axis="y")
    ax.set_ylim(bottom=0)

    plt.tight_layout()
    path = os.path.join(out_dir, "torque_comparison.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {os.path.relpath(path, XDRIVE_ROOT)}")


def plot_tipping_comparison(configs, out_dir, plt):
    """Horizontal stacked bar chart: stable vs tipping percentage per config."""
    names = []
    stable_pcts = []
    tip_pcts = []
    tip_counts = []
    total_counts = []

    for name, data, _meta in configs:
        tipping = data.get("tipping")
        if not tipping:
            continue
        total_tip = sum(t.get("n_tip", 0) for t in tipping)
        total_ok  = sum(t.get("n_ok", 0)  for t in tipping)
        if total_ok == 0:
            continue
        pct = 100.0 * total_tip / total_ok
        names.append(name)
        tip_pcts.append(pct)
        stable_pcts.append(100.0 - pct)
        tip_counts.append(total_tip)
        total_counts.append(total_ok)

    if not names:
        return

    n = len(names)
    y = np.arange(n)
    height = 0.6

    fig, ax = plt.subplots(figsize=(10, max(4, n * 0.6 + 1)))

    bars_stable = ax.barh(y, stable_pcts, height, color="#4caf50", label="Stable")
    bars_tip    = ax.barh(y, tip_pcts, height, left=stable_pcts, color="#f44336",
                          label="Tipping")

    # Annotate counts
    for i in range(n):
        # Stable count
        stable_count = total_counts[i] - tip_counts[i]
        if stable_pcts[i] > 8:
            ax.text(stable_pcts[i] / 2, y[i], f"{stable_count:,}",
                    ha="center", va="center", fontsize=8, color="white",
                    fontweight="bold")
        # Tip count
        if tip_pcts[i] > 3:
            ax.text(stable_pcts[i] + tip_pcts[i] / 2, y[i],
                    f"{tip_counts[i]:,}",
                    ha="center", va="center", fontsize=8, color="white",
                    fontweight="bold")
        # Percentage annotation on right
        ax.text(101, y[i], f"{tip_pcts[i]:.1f}% tipped", va="center",
                fontsize=8, color="#b71c1c")

    ax.set_yticks(y)
    ax.set_yticklabels(names, fontsize=9)
    ax.set_xlim(0, 115)
    ax.set_xlabel("Percentage of poses (%)", fontsize=11)
    ax.set_title("FORTIS Arm Pose Sweep — Stability Comparison", fontsize=13)
    ax.legend(loc="lower right", fontsize=9)
    ax.grid(True, alpha=0.3, axis="x")

    plt.tight_layout()
    path = os.path.join(out_dir, "tipping_comparison.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {os.path.relpath(path, XDRIVE_ROOT)}")


def _load_sweep_csv(config_dir):
    """Load sweep.csv from a config directory.

    Returns a numpy structured array or None on failure.
    """
    csv_path = os.path.join(config_dir, "sweep.csv")
    if not os.path.isfile(csv_path):
        return None
    try:
        # Read header to detect columns
        with open(csv_path, "r") as f:
            header = f.readline().strip()
        col_names = [c.strip() for c in header.split(",")]
        data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None,
                             encoding="utf-8")
        return data
    except Exception as exc:
        print(f"  [WARN] Could not load {csv_path}: {exc}")
        return None


def plot_contact_heatmap(configs, out_dir, plt):
    """2x3 heatmap grid: J2 vs J3, colour = reactor contact %.

    Only processes configs where environment == "reactor".
    """
    reactor_configs = [(n, d, m) for n, d, m in configs if m.get("env") == "reactor"]
    if not reactor_configs:
        print("  No reactor configs found — skipping contact heatmap.")
        return

    # Sort by arm length then load state for consistent subplot order
    def _sort_key(item):
        name, _data, meta = item
        arm = meta.get("arm_in", 0)
        load_order = 0 if meta.get("load") == "unloaded" else 1
        return (arm, load_order)

    reactor_configs.sort(key=_sort_key)

    # Determine grid layout
    n = len(reactor_configs)
    ncols = min(3, n)
    nrows = (n + ncols - 1) // ncols
    # Prefer 2x3 as specified
    if n <= 6:
        nrows, ncols = max(nrows, 2), 3

    fig, axes = plt.subplots(nrows, ncols, figsize=(5 * ncols, 4 * nrows),
                             squeeze=False)

    for idx, (name, data, meta) in enumerate(reactor_configs):
        row = idx // ncols
        col = idx % ncols
        ax = axes[row][col]

        config_dir = os.path.join(RESULTS_DIR, name)
        csv_data = _load_sweep_csv(config_dir)

        if csv_data is None or len(csv_data) == 0:
            ax.set_title(f"{name}\n(no CSV data)", fontsize=9)
            ax.set_visible(True)
            ax.text(0.5, 0.5, "No data", ha="center", va="center",
                    transform=ax.transAxes, fontsize=12, color="gray")
            continue

        # Extract columns — try various naming conventions
        j2_col = None
        j3_col = None
        contact_col = None
        col_names = csv_data.dtype.names if csv_data.dtype.names else []

        for cn in col_names:
            cl = cn.lower().strip()
            if cl in ("j2", "j2_deg", "theta2", "t2_deg", "t2"):
                j2_col = cn
            elif cl in ("j3", "j3_deg", "theta3", "t3_deg", "t3"):
                j3_col = cn
            elif cl in ("reactor_contact", "contact", "has_contact"):
                contact_col = cn

        if j2_col is None or j3_col is None or contact_col is None:
            ax.set_title(f"{name}\n(missing columns)", fontsize=9)
            ax.text(0.5, 0.5, f"Cols: {list(col_names)}", ha="center",
                    va="center", transform=ax.transAxes, fontsize=7,
                    color="gray", wrap=True)
            continue

        j2_vals = csv_data[j2_col].astype(float)
        j3_vals = csv_data[j3_col].astype(float)
        contact_vals = csv_data[contact_col].astype(float)

        # Unique sorted grid values
        j2_unique = np.sort(np.unique(j2_vals))
        j3_unique = np.sort(np.unique(j3_vals))

        # Build heatmap: for each (j2, j3), fraction of rows with contact==1
        heatmap = np.full((len(j3_unique), len(j2_unique)), np.nan)
        j2_idx_map = {v: i for i, v in enumerate(j2_unique)}
        j3_idx_map = {v: i for i, v in enumerate(j3_unique)}

        for j2v in j2_unique:
            for j3v in j3_unique:
                mask = (j2_vals == j2v) & (j3_vals == j3v)
                total = mask.sum()
                if total > 0:
                    n_contact = (contact_vals[mask] == 1).sum()
                    heatmap[j3_idx_map[j3v], j2_idx_map[j2v]] = 100.0 * n_contact / total

        im = ax.imshow(heatmap, aspect="auto", origin="lower",
                       cmap="RdYlGn", vmin=0, vmax=100,
                       extent=[j2_unique[0], j2_unique[-1],
                               j3_unique[0], j3_unique[-1]])

        ax.set_title(name, fontsize=10, fontweight="bold")
        ax.set_xlabel("J2 (deg)", fontsize=9)
        ax.set_ylabel("J3 (deg)", fontsize=9)
        fig.colorbar(im, ax=ax, label="Contact %", shrink=0.8)

    # Turn off unused subplots
    for idx in range(len(reactor_configs), nrows * ncols):
        row = idx // ncols
        col = idx % ncols
        axes[row][col].set_visible(False)

    fig.suptitle("FORTIS Arm Pose Sweep — Reactor Contact Heatmap\n"
                 "(% of J4 poses with reactor contact at each J2/J3 pair)",
                 fontsize=13, fontweight="bold")
    plt.tight_layout(rect=[0, 0, 1, 0.93])
    path = os.path.join(out_dir, "contact_heatmap.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {os.path.relpath(path, XDRIVE_ROOT)}")


# ---------------------------------------------------------------------------
# Combined JSON
# ---------------------------------------------------------------------------

def write_combined_json(configs, out_dir):
    """Write all_configs_summary.json mapping config_name -> summary contents."""
    combined = {}
    for name, data, _meta in configs:
        combined[name] = data

    path = os.path.join(out_dir, "all_configs_summary.json")

    def _json_default(o):
        if isinstance(o, (np.integer,)):
            return int(o)
        if isinstance(o, (np.floating,)):
            return float(o)
        if isinstance(o, np.ndarray):
            return o.tolist()
        return str(o)

    with open(path, "w") as f:
        json.dump(combined, f, indent=2, default=_json_default)
    print(f"  Saved: {os.path.relpath(path, XDRIVE_ROOT)}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print(f"\n{'=' * 60}")
    print("FORTIS Arm Pose Sweep — Results Aggregator")
    print(f"{'=' * 60}")
    print(f"\nScanning: {RESULTS_DIR}")

    if not os.path.isdir(RESULTS_DIR):
        print(f"\n[ERROR] Results directory not found:\n  {RESULTS_DIR}")
        print("Run the arm pose sweep scripts first, then re-run this parser.")
        sys.exit(1)

    # Discover configs
    print("\nLoading summaries:")
    configs = _load_summaries()

    if not configs:
        print("\n[ERROR] No summary.json files found in any subdirectory.")
        print(f"Expected structure: {RESULTS_DIR}/<config_name>/summary.json")
        sys.exit(1)

    print(f"\nFound {len(configs)} config(s).\n")

    # Comparison table
    print_comparison_table(configs)

    # Output directory
    os.makedirs(COMBINED_DIR, exist_ok=True)

    # Combined JSON
    print("Writing combined JSON...")
    write_combined_json(configs, COMBINED_DIR)

    # Charts
    plt, has_mpl = _import_matplotlib()
    if has_mpl:
        print("\nGenerating charts...")

        print("\n  [1/3] Torque comparison...")
        try:
            plot_torque_comparison(configs, COMBINED_DIR, plt)
        except Exception as exc:
            print(f"  [ERR] torque_comparison.png: {exc}")

        print("  [2/3] Tipping comparison...")
        try:
            plot_tipping_comparison(configs, COMBINED_DIR, plt)
        except Exception as exc:
            print(f"  [ERR] tipping_comparison.png: {exc}")

        print("  [3/3] Contact heatmap...")
        try:
            plot_contact_heatmap(configs, COMBINED_DIR, plt)
        except Exception as exc:
            print(f"  [ERR] contact_heatmap.png: {exc}")

    # Summary
    print(f"\n{'=' * 60}")
    print("Done.")
    print(f"  Configs loaded : {len(configs)}")
    print(f"  Output dir     : {COMBINED_DIR}")
    print(f"{'=' * 60}\n")


if __name__ == "__main__":
    main()
