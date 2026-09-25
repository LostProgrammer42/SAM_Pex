#!/usr/bin/env python3
"""
sampex_visualizer.py

Debug visualizer for .sampex files produced by PexTest.cpp.

Two views:
  1. Static layout  -- all segments drawn as rectangles, colored by layer,
                        labeled with net id. Good for sanity-checking parsing.
  2. Sweep animation -- steps through the sweep log, drawing a vertical
                        sweep-line at the event's x, and highlighting the
                        currently-active segment set. Good for debugging
                        the IntervalTree insert/erase sequence itself.

Usage:
    python3 sampex_visualizer.py <file.sampex>                 # static layout only
    python3 sampex_visualizer.py <file.sampex> --animate        # sweep animation
    python3 sampex_visualizer.py <file.sampex> --animate --save out.gif
"""

import argparse
import json
import sys

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation


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
    return segments, sweep


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


def plot_sweep_animation(segments, sweep, save_path=None, interval_ms=300):
    if not sweep:
        print("No sweep events found in this .sampex file -- nothing to animate.")
        return

    seg_by_id = {s["id"]: s for s in segments}
    x_lo, x_hi, y_lo, y_hi = compute_bounds(segments)

    fig, ax = plt.subplots(figsize=(10, 10))

    def draw_frame(step_idx):
        ax.clear()
        step = sweep[step_idx]

        # Draw all segments faded out as context
        for seg in segments:
            draw_segment(ax, seg, layer_color(seg["layer"]), alpha=0.15, edgecolor="none")

        # Highlight the currently active set
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

    if save_path:
        print(f"Saving animation to {save_path} ...")
        anim.save(save_path, writer="pillow")
        print("Done.")
    else:
        plt.tight_layout()
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="Visualize a .sampex PEX debug file.")
    parser.add_argument("sampex_file", help="Path to the .sampex file")
    parser.add_argument("--animate", action="store_true", help="Show the sweep-line animation instead of/after the static layout")
    parser.add_argument("--save", metavar="OUT.gif", help="Save the animation to a file instead of showing it interactively")
    parser.add_argument("--interval", type=int, default=300, help="Animation frame interval in ms (default: 300)")
    args = parser.parse_args()

    segments, sweep = load_sampex(args.sampex_file)
    if not segments:
        print(f"Warning: no segments found in {args.sampex_file}", file=sys.stderr)

    if args.animate:
        plot_sweep_animation(segments, sweep, save_path=args.save, interval_ms=args.interval)
    else:
        plot_static(segments, title=args.sampex_file)
        plt.show()


if __name__ == "__main__":
    main()