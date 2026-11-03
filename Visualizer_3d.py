#!/usr/bin/env python3
"""Interactive 3D viewer for .sampex files written by Pex_Test.

Shows every wire/via shape as a 3D box and, for every contributing pair, the slab of
dielectric between the two facing walls (that slab IS the capacitor the code computed).

    python visualizer_3d.py out.sampex                  # interactive window
    python visualizer_3d.py out.sampex --pair 3         # start with pair #3 highlighted
    python visualizer_3d.py out.sampex --save view.png  # headless: write a PNG and exit

Mouse: left-drag rotates, right-drag zooms (the toolbar's pan/zoom tools also work).
Keys : [ and ]  step through contributing pairs (largest capacitance first).
"""

# ---------- CONFIG ----------
COLOR_BY = "net"        # "net" or "layer": what colours the wire boxes
Z_SCALE = 3.0           # visual exaggeration of the vertical axis (tick labels stay true)
Wire_alpha = 0.28
Slab_alpha = 0.55
Show_tree = False       # draw quad tree leaves (can be toggled in the GUI)
Show_labels = False     # write "net#id" on every shape (can be toggled in the GUI)
Max_slab_groups = 10    # net pairs that get their own colour/checkbox; the rest are lumped as "other"
Layer_filter = None     # e.g. ["li1"] to draw only some layers
View_window = None      # [x1, y1, x2, y2] in um: only shapes overlapping it are drawn
Initial_pair = 0        # 0 = none highlighted

import argparse
import json

ap = argparse.ArgumentParser()
ap.add_argument("sampex", nargs="?", default="out.sampex")
ap.add_argument("--save", help="write a PNG and exit instead of opening a window")
ap.add_argument("--pair", type=int, default=Initial_pair, help="pair number to highlight (1 = largest)")
ap.add_argument("--elev", type=float, default=28)
ap.add_argument("--azim", type=float, default=-58)
args = ap.parse_args()

import numpy as np
import matplotlib

if args.save:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import to_rgba
from matplotlib.patches import Patch
from matplotlib.widgets import CheckButtons, Slider
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection

# ---------- LOAD ----------
with open(args.sampex) as f:
    data = json.load(f)

layers = data["layers"]
layer_z = data["layer_z"]          # [z_bottom, thickness] per layer
nets = data["nets"]
segs = {s["id"]: s for s in data["segments"]}
trees = {t["layer"]: t["nodes"] for t in data["trees"]}

LAYER_FACE = ["#ECBA6F", "#8e44ad", "#2e86c1", "#27ae60", "#c0392b"]
LAYER_EDGE = ["#7a5a1e", "#4a235a", "#1b4f72", "#145a32", "#7b241c"]
SLAB_COLORS = ["#e41a1c", "#ff7f00", "#984ea3", "#a65628", "#f781bf", "#377eb8", "#17becf", "#bcbd22",
               "#d62728", "#8c564b"]


def seg_box(s):
    z0, t = layer_z[s["layer"]]
    return (s["llx"], s["lly"], z0, s["llx"] + s["wx"], s["lly"] + s["wy"], z0 + t)


def in_window(s):
    if View_window is None:
        return True
    x1, y1, x2, y2 = View_window
    return not (s["llx"] + s["wx"] < x1 or s["llx"] > x2 or s["lly"] + s["wy"] < y1 or s["lly"] > y2)


shown = {i for i, s in segs.items()
         if (Layer_filter is None or layers[s["layer"]] in Layer_filter) and in_window(s)}
if not shown:
    raise SystemExit("nothing to draw (check Layer_filter / View_window)")

pairs = [p for p in data["pairs"] if p["a"] in shown and p["b"] in shown]
# counted pairs first (largest C first), then pairs that slab blocking removed entirely (largest raw C first)
pairs.sort(key=lambda p: (-p["fF"], -p["raw_fF"]))
N = len(pairs)

FACES = [(0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]


def box_faces(x1, y1, z1, x2, y2, z2):
    v = np.array([[x1, y1, z1], [x2, y1, z1], [x2, y2, z1], [x1, y2, z1],
                  [x1, y1, z2], [x2, y1, z2], [x2, y2, z2], [x1, y2, z2]])
    return [v[list(f)] for f in FACES]


def net_name(i):
    return nets[i]["name"]


# ---------- COLOURS ----------
cmap = plt.get_cmap("Set2")
net_color, k = {}, 0
for i, n in enumerate(nets):
    if n["supply"]:
        net_color[i] = to_rgba("#9a9a9a")
    else:
        net_color[i] = cmap(k % 8)
        k += 1


def seg_face_color(s):
    c = net_color[s["net"]] if COLOR_BY == "net" else to_rgba(LAYER_FACE[s["layer"] % len(LAYER_FACE)])
    return to_rgba(c, Wire_alpha)


# ---------- FIGURE ----------
fig = plt.figure(figsize=(14.5, 8.5))
ax = fig.add_axes([0.0, 0.0, 0.66, 0.93], projection="3d")
# Draw artists in a fixed order (wires < slabs < selection) instead of per-collection depth sorting,
# so the selected pair is never hidden behind other translucent boxes.
ax.computed_zorder = False

# wires: one artist for all shapes
w_faces, w_fc, w_ec = [], [], []
for i in sorted(shown):
    s = segs[i]
    fcs = box_faces(*seg_box(s))
    w_faces += fcs
    w_fc += [seg_face_color(s)] * 6
    w_ec += [to_rgba(LAYER_EDGE[s["layer"] % len(LAYER_EDGE)], 0.9)] * 6
wires = Poly3DCollection(w_faces, facecolors=w_fc, edgecolors=w_ec, linewidths=0.7)
ax.add_collection3d(wires)
wires.set_zorder(1)

# coupling slabs, grouped by net pair (largest groups get their own colour + checkbox)
group_total = {}
for p in pairs:
    if p["fF"] <= 0:
        continue  # fully blocked pairs get their own grey group below
    key = tuple(sorted((segs[p["a"]]["net"], segs[p["b"]]["net"])))
    group_total[key] = group_total.get(key, 0.0) + p["fF"]
ordered_groups = sorted(group_total, key=lambda g: -group_total[g])
named = ordered_groups[:Max_slab_groups]
group_of = {g: g for g in named}


def group_key(p):
    if p["fF"] <= 0:
        return "blocked"
    key = tuple(sorted((segs[p["a"]]["net"], segs[p["b"]]["net"])))
    return key if key in group_of else "other"


def gap_line(p):
    x1, y1, z1, x2, y2, z2 = p["box"]
    cx, cy, cz = (x1 + x2) / 2, (y1 + y2) / 2, (z1 + z2) / 2
    if p["axis"] == "x":
        return [(x1, cy, cz), (x2, cy, cz)]
    if p["axis"] == "y":
        return [(cx, y1, cz), (cx, y2, cz)]
    return [(cx, cy, z1), (cx, cy, z2)]


toggles = {"wires": [wires]}
by_group = {}
for p in pairs:
    by_group.setdefault(group_key(p), []).append(p)

labels_order = ["wires"]
group_labels = {}
extra_groups = [k for k in ("other", "blocked") if k in by_group]
for gi, key in enumerate(named + extra_groups):
    if key == "other":
        color, lab = "#444444", "slabs: other pairs"
    elif key == "blocked":
        color, lab = "#808080", f"blocked pairs (C removed): {len(by_group['blocked'])}"
    else:
        sup = nets[key[0]]["supply"] or nets[key[1]]["supply"]
        color = SLAB_COLORS[gi % len(SLAB_COLORS)]
        lab = f"{net_name(key[0])} <-> {net_name(key[1])}  {group_total[key]:.4g} fF{' *' if sup else ''}"
    faces = []
    for p in by_group[key]:
        faces += box_faces(*p["box"])
    poly = Poly3DCollection(faces, facecolors=to_rgba(color, 0.2 if key == "blocked" else Slab_alpha),
                            edgecolors=to_rgba(color, 1.0), linewidths=1.0,
                            linestyles="--" if key == "blocked" else "-")
    ax.add_collection3d(poly)
    poly.set_zorder(2)
    lines = Line3DCollection([gap_line(p) for p in by_group[key]], colors=[to_rgba(color, 1.0)], linewidths=2.2)
    ax.add_collection3d(lines)
    lines.set_zorder(3)
    toggles[lab] = [poly, lines]
    labels_order.append(lab)
    group_labels[lab] = color

# highlight artists (selected pair): created hidden with a dummy face
dummy = [np.zeros((4, 3))]
hl_slab = Poly3DCollection(dummy, facecolors=to_rgba("#ffee00", 0.8), edgecolors="black", linewidths=2.2)
hl_segs = Poly3DCollection(dummy, facecolors=to_rgba("#ff0000", 0.15), edgecolors=to_rgba("#d00000", 1.0),
                           linewidths=2.4)
# shapes sitting inside the selected pair's slab (the ones that block it)
hl_blk = Poly3DCollection(dummy, facecolors=to_rgba("#ff9900", 0.35), edgecolors=to_rgba("#e65c00", 1.0),
                          linewidths=2.2, linestyles="--")
for h, z in ((hl_slab, 5), (hl_blk, 5.5), (hl_segs, 6)):
    ax.add_collection3d(h)
    h.set_zorder(z)
    h.set_visible(False)

# quad tree leaves (drawn on top face of each layer), hidden unless Show_tree
for li, nodes in trees.items():
    ztop = layer_z[li][0] + layer_z[li][1] + 0.003
    segs3d = []
    for n in nodes:
        if not n["leaf"]:
            continue
        x1, y1, x2, y2 = n["llx"], n["lly"], n["llx"] + n["wx"], n["lly"] + n["wy"]
        c = [(x1, y1, ztop), (x2, y1, ztop), (x2, y2, ztop), (x1, y2, ztop)]
        segs3d += [[c[i], c[(i + 1) % 4]] for i in range(4)]
    if not segs3d:
        continue
    tl = Line3DCollection(segs3d, colors=[to_rgba(LAYER_EDGE[li % len(LAYER_EDGE)], 0.8)], linewidths=0.8,
                          linestyles="--")
    ax.add_collection3d(tl)
    tl.set_zorder(4)
    tl.set_visible(Show_tree)
    lab = f"quad tree: {layers[li]}"
    toggles[lab] = [tl]
    labels_order.append(lab)

# net#id labels
label_texts = []
if len(shown) <= 400:
    for i in sorted(shown):
        s = segs[i]
        x1, y1, z1, x2, y2, z2 = seg_box(s)
        t = ax.text((x1 + x2) / 2, (y1 + y2) / 2, z2, f"{net_name(s['net'])}#{i}", fontsize=6, ha="center",
                    va="bottom")
        t.set_visible(Show_labels)
        t.set_zorder(7)
        label_texts.append(t)
    toggles["labels: net#id"] = label_texts
    labels_order.append("labels: net#id")

# ---------- AXES ----------
xs = [segs[i]["llx"] for i in shown] + [segs[i]["llx"] + segs[i]["wx"] for i in shown]
ys = [segs[i]["lly"] for i in shown] + [segs[i]["lly"] + segs[i]["wy"] for i in shown]
zs = [seg_box(segs[i])[2] for i in shown] + [seg_box(segs[i])[5] for i in shown]
x0, x1_, y0, y1_, z0, z1_ = min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)
pad = 0.05 * max(x1_ - x0, y1_ - y0)
ax.set_xlim(x0 - pad, x1_ + pad)
ax.set_ylim(y0 - pad, y1_ + pad)
ax.set_zlim(z0 - 0.02, z1_ + 0.02)
ax.set_xlabel("x (µm)")
ax.set_ylabel("y (µm)")
ax.set_zlabel("z (µm)")
ax.view_init(elev=args.elev, azim=args.azim)
dx, dy, dz = (x1_ - x0 + 2 * pad), (y1_ - y0 + 2 * pad), (z1_ - z0 + 0.04)


def set_aspect(zscale):
    ax.set_box_aspect((dx, dy, dz * zscale))
    ax.set_title(f"{data['design']}: coupling slabs   (z axis stretched ×{zscale:g})", fontsize=11)


set_aspect(Z_SCALE)

# legend (wire colours)
handles = []
if COLOR_BY == "net":
    used = sorted({segs[i]["net"] for i in shown})
    handles += [Patch(facecolor=net_color[n], edgecolor="none", label=f"net {net_name(n)}") for n in used]
used_layers = sorted({segs[i]["layer"] for i in shown})
handles += [Patch(facecolor="none", edgecolor=LAYER_EDGE[l % len(LAYER_EDGE)], label=f"{layers[l]} (edge colour)")
            for l in used_layers]
handles += [Patch(facecolor=to_rgba("#ffee00", 0.8), edgecolor="black", label="selected slab"),
            Patch(facecolor=to_rgba("#ff9900", 0.35), edgecolor="#e65c00", label="blockers inside slab")]
ax.legend(handles=handles, loc="upper left", fontsize=8, frameon=True)

# ---------- WIDGETS ----------
cax = fig.add_axes([0.675, 0.50, 0.32, 0.46], frameon=False)
cax.set_title("Show / hide", loc="left", fontsize=10)
actives = [toggles[l][0].get_visible() for l in labels_order]
check = CheckButtons(cax, labels_order, actives)
for t in check.labels:
    t.set_fontsize(8.5)
    if t.get_text() in group_labels:
        t.set_color(group_labels[t.get_text()])


def on_check(label):
    for art in toggles[label]:
        art.set_visible(not art.get_visible())
    fig.canvas.draw_idle()


check.on_clicked(on_check)

detail = fig.text(0.68, 0.37, "", va="top", family="monospace", fontsize=8.8)
fig.text(0.68, 0.03,
         "left-drag: rotate   right-drag: zoom   [ / ]: step through pairs\n"
         "slab = dielectric between the two facing walls;  line = field direction\n"
         "orange dashed = shapes inside the slab that block it;  (*) = supply net",
         va="bottom", fontsize=8, color="#555555")


def describe(k):
    p = pairs[k - 1]
    a, b = segs[p["a"]], segs[p["b"]]
    la, lb = layers[a["layer"]], layers[b["layer"]]
    state = "  ** BLOCKED **" if p["fF"] <= 0 else ""
    head = f"Pair {k} of {N}{state}\n"
    who = (f"  {net_name(a['net'])} (seg {a['id']}, {la}{', via' if a['via'] else ''})\n"
           f"  {net_name(b['net'])} (seg {b['id']}, {lb}{', via' if b['via'] else ''})\n")
    if p["type"] == "side":
        body = (f"type          : same-layer sidewall ({la})\n"
                f"facing across : {p['axis']}\n"
                f"edge spacing  : {p['gap']:.4f} µm\n"
                f"parallel run  : {p['extent']:.4f} µm\n"
                f"  unblocked   : {p['clear']:.4f} µm\n"
                f"thickness     : {layer_z[a['layer']][1]:.4f} µm\n"
                f"C = eps*fringe*t*unblocked_run/spacing\n")
        if p["blockers"]:
            body += "blockers      : " + ", ".join(
                f"seg {i} ({net_name(segs[i]['net'])})" for i in p["blockers"]) + "\n"
        else:
            body += "blockers      : none\n"
    else:
        body = (f"type          : adjacent-layer overlap ({la}/{lb})\n"
                f"overlap area  : {p['extent']:.5f} µm²\n"
                f"dielectric gap: {p['gap']:.4f} µm\n"
                f"C = eps*area/gap\n")
    return head + who + body + f"raw C = {p['raw_fF']:.4e} fF   ->   counted C = {p['fF']:.4e} fF"


def show_pair(k):
    k = int(k)
    if k <= 0 or k > N:
        hl_slab.set_visible(False)
        hl_segs.set_visible(False)
        hl_blk.set_visible(False)
        detail.set_text("No pair selected.\nUse the slider or [ / ] keys\nto step through pairs." if N else
                        "No coupling pairs in this view.")
        fig.canvas.draw_idle()
        return
    p = pairs[k - 1]
    hl_slab.set_verts(box_faces(*p["box"]))
    hl_segs.set_verts(box_faces(*seg_box(segs[p["a"]])) + box_faces(*seg_box(segs[p["b"]])))
    hl_slab.set_visible(True)
    hl_segs.set_visible(True)
    blk = [i for i in p["blockers"] if i in segs]
    if blk:
        hl_blk.set_verts([f for i in blk for f in box_faces(*seg_box(segs[i]))])
    hl_blk.set_visible(bool(blk))
    detail.set_text(describe(k))
    fig.canvas.draw_idle()


pair_slider = None
if N:
    pair_ax = fig.add_axes([0.73, 0.44, 0.22, 0.022])
    pair_slider = Slider(pair_ax, "pair #", 0, N, valinit=0, valstep=1)
    pair_slider.on_changed(show_pair)

z_ax = fig.add_axes([0.73, 0.40, 0.22, 0.022])
z_slider = Slider(z_ax, "z stretch", 1, 10, valinit=Z_SCALE)
z_slider.on_changed(lambda v: (set_aspect(v), fig.canvas.draw_idle()))


def on_key(event):
    if pair_slider is None:
        return
    if event.key == "]":
        pair_slider.set_val(min(N, pair_slider.val + 1))
    elif event.key == "[":
        pair_slider.set_val(max(0, pair_slider.val - 1))


fig.canvas.mpl_connect("key_press_event", on_key)

if pair_slider is not None and args.pair:
    pair_slider.set_val(min(max(args.pair, 0), N))
else:
    show_pair(0)

if args.save:
    fig.savefig(args.save, dpi=110)
    print("wrote", args.save)
else:
    plt.show()