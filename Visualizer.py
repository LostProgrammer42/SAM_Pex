import json
import sys
from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle, Patch

# ---------- CONFIG ----------
SAMPEX_FILE = sys.argv[1] if len(sys.argv) > 1 else "out.sampex"
OUT_PDF = "layout_tree.pdf"

Show_segments = True
Show_tree = True
Show_internal_nodes = False     # also draw non-leaf quadrants
Shade_leaves_by_count = True    # leaf fill intensity ~ number of segment ids stored
Show_net_names = True
Layer_filter = None             # e.g. ["li1"] to show only some layers; None = all
Tree_layer_filter = None        # layers whose trees are drawn; None = same as Layer_filter
Highlight_net = None            # e.g. "out": dim every other net
Zoom_to_fit = True
Legend = True
Grid = False

# ---------- COLOR MAP ----------
def layer_color(layer):
    layer = layer.lower()
    if layer == "li1":
        return "#ECBA6F"
    if layer.startswith("met"):
        return "purple"
    if layer == "poly":
        return "red"
    return "gray"

# ---------- LOAD ----------
with open(SAMPEX_FILE) as f:
    data = json.load(f)

layers = data["layers"]
nets = data["nets"]
segs = data["segments"]
trees = {t["layer"]: t["nodes"] for t in data["trees"]}

def layer_shown(idx):
    return Layer_filter is None or layers[idx] in Layer_filter

def tree_shown(idx):
    flt = Tree_layer_filter if Tree_layer_filter is not None else Layer_filter
    return flt is None or layers[idx] in flt

# ---------- PLOT ----------
fig, ax = plt.subplots(figsize=(9, 9))

if Show_segments:
    for s in segs:
        if not layer_shown(s["layer"]):
            continue
        lname = layers[s["layer"]]
        net_name = nets[s["net"]]["name"]
        dim = Highlight_net is not None and net_name != Highlight_net
        ax.add_patch(Rectangle(
            (s["llx"], s["lly"]), s["wx"], s["wy"],
            facecolor=layer_color(lname),
            edgecolor=layer_color(lname),
            linewidth=0.5,
            alpha=0.15 if dim else (0.55 if s["layer"] == 0 else 0.45),
            hatch="xxxx" if s["via"] else None,
            zorder=10 + s["layer"],
        ))
        if Show_net_names and not dim:
            ax.text(s["llx"] + s["wx"] / 2, s["lly"] + s["wy"] / 2, net_name,
                    ha="center", va="center", fontsize=6, color="black", zorder=30)

if Show_tree:
    cmap = plt.get_cmap("Blues")
    for li, nodes in trees.items():
        if not tree_shown(li):
            continue
        max_n = max([n["n"] for n in nodes if n["leaf"]] + [1])
        for n in nodes:
            if not n["leaf"] and not Show_internal_nodes:
                continue
            face = "none"
            if Shade_leaves_by_count and n["leaf"] and n["n"] > 0:
                face = cmap(0.15 + 0.7 * n["n"] / max_n)
            ax.add_patch(Rectangle(
                (n["llx"], n["lly"]), n["wx"], n["wy"],
                facecolor=face,
                edgecolor="dimgray",
                linewidth=0.4,
                alpha=0.25 if face != "none" else 1.0,
                zorder=1,
            ))

# ---------- AXES ----------
die = data["die"]
ax.set_aspect("equal", adjustable="box")
ax.set_xlabel("x (um)")
ax.set_ylabel("y (um)")
ax.set_title(f"{data['design']}: segments + quad tree leaves")
if Grid:
    ax.grid(True, linestyle=":", linewidth=0.5)

if Zoom_to_fit and segs:
    xs = [s["llx"] for s in segs] + [s["llx"] + s["wx"] for s in segs]
    ys = [s["lly"] for s in segs] + [s["lly"] + s["wy"] for s in segs]
    pad = 0.1
    ax.set_xlim(min(xs) - pad, max(xs) + pad)
    ax.set_ylim(min(ys) - pad, max(ys) + pad)
else:
    ax.set_xlim(die[0] - 0.1, die[2] + 0.1)
    ax.set_ylim(die[1] - 0.1, die[3] + 0.1)

if Legend:
    handles = [Patch(facecolor=layer_color(l), alpha=0.5, label=l)
               for i, l in enumerate(layers) if layer_shown(i)]
    if Show_tree:
        handles.append(Patch(facecolor="none", edgecolor="dimgray", label="quad tree leaf"))
    ax.legend(handles=handles, title="Layers", loc="upper left",
              bbox_to_anchor=(1.02, 1), frameon=True)

plt.savefig(OUT_PDF, bbox_inches="tight", facecolor="white", format="pdf")
plt.show()