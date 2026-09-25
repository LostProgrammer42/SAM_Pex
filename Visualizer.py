#!/usr/bin/env python3
"""
Visualizer.py

Debug visualizer for .sampex files produced by Pex_Test.cpp.

Three views:
  1. Static layout  -- all segments drawn as rectangles, colored by layer.
                        Good for sanity-checking parsing.
  2. Sweep animation -- steps through the sweep log, drawing a vertical
                        sweep-line at the event's x, and highlighting the
                        currently-active segment set. Good for debugging
                        the IntervalTree insert/erase sequence itself.
  3. Net view        -- given a net name, shows only that net's segments
                        (across all layers -- net_id is shared across layers
                        by vias/contacts, so this doesn't need separate via
                        data, it just filters by net_id). Each segment is
                        labeled with its area, so R/C values in the .sampex
                        can be sanity-checked at a glance against geometry.

Usage:
    python3 Visualizer.py <file.sampex>                 # static layout
    python3 Visualizer.py <file.sampex> --animate        # sweep animation
    python3 Visualizer.py <file.sampex> --net Y          # net view for net "Y"
"""

import argparse
import json
import sys

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation

Use_Gray_Nets = False

# Fixed layer -> color mapping so colors stay consistent across runs/files.
LAYER_COLORS = {
    "ndiff":       "#8dd35f",
    "pdiff":       "#e8a33d",
    "ntransistor": "#4f9d3d",
    "ptransistor": "#b4741f",
    "polysilicon": "#d9455f",
    "li":          "#3d8fe8",
    "m1":          "#e0c93d",
    "m2":          "#9d3de8",
    "polycon":     "#555555",
    "librdrcon":   "#777777",
    "mcon":        "#999999",
    "ndc":         "#333333",
    "pdc":         "#333333",
    "none":        "#cccccc",
}


def load_sampex(path):
    with open(path, "r") as f:
        data = json.load(f)
    segments = data.get("segments", [])
    sweep = data.get("sweep", [])
    nets = data.get("nets", [])
    return segments, sweep, nets


def layer_color(layer):
    return LAYER_COLORS.get(layer, "#cccccc")


def compute_bounds(segments):
    if not segments:
        return (0, 1, 0, 1)
    x_lo = min(s["x_lo"] for s in segments)
    x_hi = max(s["x_hi"] for s in segments)
    y_lo = min(s["y_lo"] for s in segments)
    y_hi = max(s["y_hi"] for s in segments)
    pad_x = max(1, (x_hi - x_lo) * 0.05)
    pad_y = max(1, (y_hi - y_lo) * 0.05)
    return (x_lo - pad_x, x_hi + pad_x, y_lo - pad_y, y_hi + pad_y)


def segment_area(seg):
    return (seg["x_hi"] - seg["x_lo"]) * (seg["y_hi"] - seg["y_lo"])


def draw_segment(ax, seg, facecolor, edgecolor="black", alpha=0.7, linewidth=0.8):
    w = seg["x_hi"] - seg["x_lo"]
    h = seg["y_hi"] - seg["y_lo"]
    rect = patches.Rectangle(
        (seg["x_lo"], seg["y_lo"]), w, h,
        facecolor=facecolor, edgecolor=edgecolor,
        alpha=alpha, linewidth=linewidth,
    )
    ax.add_patch(rect)


def plot_static(segments, title="PEX Layout"):
    fig, ax = plt.subplots(figsize=(10, 10))
    x_lo, x_hi, y_lo, y_hi = compute_bounds(segments)

    for seg in segments:
        draw_segment(ax, seg, layer_color(seg["layer"]))

    # Legend: one entry per layer actually present
    present_layers = sorted({seg["layer"] for seg in segments})
    handles = [
        patches.Patch(facecolor=layer_color(l), edgecolor="black", label=l)
        for l in present_layers
    ]
    ax.legend(handles=handles, loc="upper right", fontsize=8, framealpha=0.9)

    ax.set_xlim(x_lo, x_hi)
    ax.set_ylim(y_lo, y_hi)
    ax.set_aspect("equal")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title(title)
    plt.tight_layout()
    return fig, ax


def plot_sweep_animation(segments, sweep, interval_ms=300):
    if not sweep:
        print("No sweep events found in this .sampex file -- nothing to animate.")
        return

    seg_by_id = {s["id"]: s for s in segments}
    x_lo, x_hi, y_lo, y_hi = compute_bounds(segments)

    fig, ax = plt.subplots(figsize=(10, 10))

    def draw_frame(step_idx):
        ax.clear()
        step = sweep[step_idx]

        for seg in segments:
            if Use_Gray_Nets:
                draw_segment(ax, seg, "#dddddd", alpha=1.0, edgecolor="#bbbbbb", linewidth=0.5)
            else:
                draw_segment(ax, seg, layer_color(seg["layer"]), alpha=0.15, edgecolor="none")

        # Highlight the currently active set in full, correct layer color
        for seg_id in step["active"]:
            seg = seg_by_id.get(seg_id)
            if seg is None:
                continue
            draw_segment(ax, seg, layer_color(seg["layer"]), alpha=0.9, edgecolor="black", linewidth=1.2)

        # Draw the sweep line itself
        ax.axvline(x=step["x"], color="red", linewidth=1.5, linestyle="--")

        ax.set_xlim(x_lo, x_hi)
        ax.set_ylim(y_lo, y_hi)
        ax.set_aspect("equal")
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_title(
            f"Sweep step {step_idx + 1}/{len(sweep)}  |  x={step['x']}  "
            f"{step['event']} seg {step['seg_id']}  |  active={step['active']}"
        )

    anim = animation.FuncAnimation(
        fig, draw_frame, frames=len(sweep), interval=interval_ms, repeat=True
    )
    plt.tight_layout()
    plt.show()


def plot_net(segments, net_name, nets_summary=None):
    net_segments = [s for s in segments if str(s["net"]) == net_name]

    if not net_segments:
        # net_name might be a name string but "net" field in segments is a
        # numeric id -- try resolving via the nets summary if present.
        if nets_summary:
            match = next((n for n in nets_summary if n["net_name"] == net_name), None)
            if match:
                net_segments = [s for s in segments if s["net"] == match["net_id"]]

    if not net_segments:
        print(f"No segments found for net '{net_name}'.", file=sys.stderr)
        print("Available nets:", sorted({s["net"] for s in segments}), file=sys.stderr)
        return

    fig, ax = plt.subplots(figsize=(10, 10))

    for seg in segments:
        if Use_Gray_Nets:
            draw_segment(ax, seg, "#dddddd", alpha=1.0, edgecolor="#bbbbbb", linewidth=0.5)
        else:
            draw_segment(ax, seg, layer_color(seg["layer"]), alpha=0.15, edgecolor="none")

    total_area = 0
    for seg in net_segments:
        area = segment_area(seg)
        total_area += area
        draw_segment(ax, seg, layer_color(seg["layer"]), alpha=0.85, edgecolor="black", linewidth=1.2)

        cx = (seg["x_lo"] + seg["x_hi"]) / 2
        cy = (seg["y_lo"] + seg["y_hi"]) / 2
        label = f"A={area}"
        if "resistance" in seg:
            label += f"\nR={seg['resistance']:.3g}"
        if "cap_ground" in seg:
            label += f"\nC={seg['cap_ground']:.3g}"
        ax.text(cx, cy, label, ha="center", va="center", fontsize=6.5,
                 bbox=dict(boxstyle="round,pad=0.15", facecolor="white", alpha=0.75, edgecolor="none"))

    x_lo, x_hi, y_lo, y_hi = compute_bounds(segments)
    ax.set_xlim(x_lo, x_hi)
    ax.set_ylim(y_lo, y_hi)
    ax.set_aspect("equal")
    ax.set_xlabel("x")
    ax.set_ylabel("y")

    present_layers = sorted({seg["layer"] for seg in net_segments})
    handles = [
        patches.Patch(facecolor=layer_color(l), edgecolor="black", label=l)
        for l in present_layers
    ]
    ax.legend(handles=handles, loc="upper right", fontsize=8, framealpha=0.9)

    title = f"Net '{net_name}'  |  {len(net_segments)} segments  |  total area={total_area}"
    if nets_summary:
        match = next((n for n in nets_summary if str(n["net_id"]) == net_name or n["net_name"] == net_name), None)
        if match:
            title += f"\nR_total={match['resistance_total']:.4g}  C_ground_total={match['cap_ground_total']:.4g}"
    ax.set_title(title, fontsize=10)

    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Visualize a .sampex PEX debug file.")
    parser.add_argument("sampex_file", help="Path to the .sampex file")
    parser.add_argument("--animate", action="store_true", help="Show the sweep-line animation")
    parser.add_argument("--interval", type=int, default=300, help="Animation frame interval in ms (default: 300)")
    parser.add_argument("--net", metavar="NET_NAME_OR_ID", help="Show only the segments belonging to this net (across all layers), with per-segment area/R/C labels")
    args = parser.parse_args()

    segments, sweep, nets = load_sampex(args.sampex_file)
    if not segments:
        print(f"Warning: no segments found in {args.sampex_file}", file=sys.stderr)

    if args.net is not None:
        plot_net(segments, args.net, nets_summary=nets)
    elif args.animate:
        plot_sweep_animation(segments, sweep, interval_ms=args.interval)
    else:
        plot_static(segments, title=args.sampex_file)
        plt.show()


if __name__ == "__main__":
    main()