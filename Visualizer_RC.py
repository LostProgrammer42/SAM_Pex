#!/usr/bin/env python3
"""2D RC viewer. Layout in the background, resistor symbols (zigzag) on each wire stretch at its exact
position with the ohm value, vias as a boxed R, nodes as dots.

Interactive window: tick/untick LAYERS and NETS on the left, plus display options (layout shapes,
nodes, node ids, R values). Zoom / pan with the toolbar. Saves a picture only if a file name is given
(the saved picture shows what is ticked at start-up, i.e. after --net / --layer).

Usage: Visualizer_RC.py out.sampex [save_as.png|.pdf|.svg] [--net=NAME[,NAME]] [--layer=NAME[,NAME]]
       --net / --layer set what is ticked when the window opens (default: everything)."""
import json, sys
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.widgets import CheckButtons

args = [a for a in sys.argv[1:] if not a.startswith("--")]
opt = {a.split("=")[0]: (a.split("=")[1] if "=" in a else "") for a in sys.argv[1:] if a.startswith("--")}
src = args[0] if args else "out.sampex"
png = args[1] if len(args) > 1 else None  # save only when a file name is given
d = json.load(open(src)); rc = d["rc"]
nets = [n["name"] for n in d["nets"]]; layers = d["layers"]
LCOL = {0: "#ECBA6F", 1: "#b48ad6"}
NCOL = plt.get_cmap("tab10")
VIA = "via"  # pseudo-layer for the via resistors

def listopt(k, allv):
    v = opt.get(k)
    return set(allv) if not v else set(v.split(","))
init_nets = listopt("--net", nets)
init_layers = listopt("--layer", layers + [VIA])

def fmt(o): return f"{o:.3g} Ω" if o < 1000 else f"{o/1000:.3g} kΩ"

fig, ax = plt.subplots(figsize=(13, 7.5))
fig.subplots_adjust(left=0.2)
span = max(d["die"][2] - d["die"][0], d["die"][3] - d["die"][1])
amp = span * 0.006

def zigzag(x1, y1, x2, y2):
    L = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
    if L == 0: return [x1], [y1]
    ux, uy = (x2 - x1) / L, (y2 - y1) / L; nx, ny = -uy, ux
    lead = 0.2 * L
    pts = [(x1, y1), (x1 + ux * lead, y1 + uy * lead)]
    for k in range(6):
        f = lead + (L - 2 * lead) * (k + 0.5) / 6
        sg = 1 if k % 2 == 0 else -1
        pts.append((x1 + ux * f + nx * amp * sg, y1 + uy * f + ny * amp * sg))
    pts += [(x2 - ux * lead, y2 - uy * lead), (x2, y2)]
    return [p[0] for p in pts], [p[1] for p in pts]

# every artist is tagged: (net index, layer name, kind) -- visibility is recomputed from the checkboxes
tagged = []  # (artist, net, layername, kind)  kind in shape / res / val / node / nid
def add(artist, net, lay, kind):
    tagged.append((artist, net, lay, kind)); return artist

for s in d["segments"]:
    r = Rectangle((s["llx"], s["lly"]), s["wx"], s["wy"], facecolor=LCOL.get(s["layer"], "#ccc"),
                  edgecolor="none", alpha=0.35, zorder=1 + s["layer"])
    ax.add_patch(r); add(r, s["net"], layers[s["layer"]], "shape")

for r in rc["res"]:
    net = rc["nodes"][r["a"]]["net"]; col = NCOL(net % 10)
    if r["via"] >= 0:
        cx, cy = r["x1"], r["y1"]
        b = Rectangle((cx - amp * 2, cy - amp * 2), amp * 4, amp * 4, fill=False, edgecolor=col, lw=1.8, zorder=20)
        ax.add_patch(b); add(b, net, VIA, "res")
        t = ax.text(cx + amp * 2.5, cy - amp * 2.5, f'via {fmt(r["ohm"])}', fontsize=7, color="k", zorder=21,
                    bbox=dict(fc="w", ec="none", alpha=0.7, pad=0.5)); add(t, net, VIA, "val")
        continue
    lay = layers[r["layer"]]
    xs, ys = zigzag(r["x1"], r["y1"], r["x2"], r["y2"])
    ln, = ax.plot(xs, ys, "-", color=col, lw=1.4, zorder=10); add(ln, net, lay, "res")
    horiz = abs(r["y2"] - r["y1"]) < 1e-9
    mx, my = (r["x1"] + r["x2"]) / 2, (r["y1"] + r["y2"]) / 2
    t = ax.text(mx + (0 if horiz else amp * 2.5), my + (amp * 2.5 if horiz else 0), fmt(r["ohm"]), fontsize=7,
                ha="center" if horiz else "left", va="bottom" if horiz else "center", color=col, zorder=21,
                bbox=dict(fc="w", ec="none", alpha=0.75, pad=0.5)); add(t, net, lay, "val")

for n in rc["nodes"]:
    lay = layers[n["layer"]]
    p, = ax.plot(n["x"], n["y"], "o", ms=4, color="k", zorder=15); add(p, n["net"], lay, "node")
    t = ax.text(n["x"], n["y"], f' n{n["id"]}', fontsize=6, color="0.3", va="top", zorder=22); add(t, n["net"], lay, "nid")

# a node sits on a layer, but a via node joins two layers: show it if either adjacent resistor's layer is ticked
node_layers = {n["id"]: {layers[n["layer"]]} for n in rc["nodes"]}
for r in rc["res"]:
    for k in (r["a"], r["b"]):
        node_layers[k].add(VIA if r["via"] >= 0 else layers[r["layer"]])
node_tags = [(a, k) for (a, net, lay, kind) in tagged if kind in ("node", "nid")]
# tagged order for nodes follows rc["nodes"] (two artists per node)
node_artists = {}
i = 0
for n in rc["nodes"]:
    node_artists[n["id"]] = [node_tags[i][0], node_tags[i + 1][0]]; i += 2

ax.set_aspect("equal"); ax.autoscale_view()
ax.set_title(f'{d["design"]}: {len(rc["nodes"])} nodes, {len(rc["res"])} resistors (R only)')
ax.set_xlabel("um"); ax.set_ylabel("um")

# ---- widgets ----
layer_names = layers + [VIA]
a_l = fig.add_axes([0.01, 0.62, 0.16, 0.3]); a_l.set_title("Layers", fontsize=9, loc="left")
a_n = fig.add_axes([0.01, 0.30, 0.16, 0.3]); a_n.set_title("Nets", fontsize=9, loc="left")
a_o = fig.add_axes([0.01, 0.05, 0.16, 0.22]); a_o.set_title("Show", fontsize=9, loc="left")
for a in (a_l, a_n, a_o): a.set_frame_on(False)
w_l = CheckButtons(a_l, layer_names, [l in init_layers for l in layer_names])
w_n = CheckButtons(a_n, nets, [n in init_nets for n in nets])
OPTS = ["layout shapes", "resistors", "R values", "nodes", "node ids"]
w_o = CheckButtons(a_o, OPTS, [True, True, True, True, False])
for i, n in enumerate(nets):  # colour the net labels like their resistors
    w_n.labels[i].set_color(NCOL(i % 10)); w_n.labels[i].set_fontweight("bold")
for w in (w_l, w_n, w_o):
    for lab in w.labels: lab.set_fontsize(9)

def refresh(_=None):
    ls = dict(zip(layer_names, w_l.get_status()))
    ns = dict(zip(nets, w_n.get_status()))
    os_ = dict(zip(OPTS, w_o.get_status()))
    kindopt = {"shape": "layout shapes", "res": "resistors", "val": "R values", "node": "nodes", "nid": "node ids"}
    for art, net, lay, kind in tagged:
        if kind in ("node", "nid"): continue
        art.set_visible(ns[nets[net]] and ls[lay] and os_[kindopt[kind]])
    for n in rc["nodes"]:  # nodes: visible if the net is ticked and any layer they touch is ticked
        vis_l = any(ls[l] for l in node_layers[n["id"]])
        for art, kind in zip(node_artists[n["id"]], ("node", "nid")):
            art.set_visible(ns[nets[n["net"]]] and vis_l and os_[kindopt[kind]])
    fig.canvas.draw_idle()

for w in (w_l, w_n, w_o): w.on_clicked(refresh)
refresh()
if png:
    fig.savefig(png, dpi=130, bbox_inches="tight")
    print("wrote", png)
plt.show()  # window; with a non-interactive backend (no display) this does nothing