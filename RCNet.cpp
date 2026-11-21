// RCNet.cpp -- turn every connected piece into a resistor network (R only for now).
// Included by Pex_Test.cpp (unity build) after Connectivity.cpp.
//
// Model
//   * A wire is its centreline + width. R of a stretch = rsheet * length / width.
//   * A wire is cut at: both ends, every point where another shape of the same piece touches it
//     (the closest point of this wire's centreline to that shape), and every MAX_SEG_LEN.
//   * Each cut is a node; the resistor between two neighbouring cuts is one stretch of wire.
//   * Touching wires (same layer) share a node. A via pad shares a node with the wire it touches;
//     the via itself is a single resistor between its pads (CUT_R / ncuts).
//   * A turn is therefore just a junction node with one resistor on each leg, centre to centre
//     (the corner square is counted as a full square; a real corner is ~0.56).
#pragma once

struct RCNode {
    int id = -1;
    double x = 0, y = 0;  // um
    int layer = -1;
    int net = -1;
    int piece = -1;
};

struct RCRes {
    int a = -1, b = -1;                       // node ids
    double ohm = 0;
    int seg = -1;                             // wire segment this stretch belongs to, -1 for a via
    int via = -1;                             // via instance for a via resistor, else -1
    double x1 = 0, y1 = 0, x2 = 0, y2 = 0;    // where it sits on the layout (wire stretch ends / via pad centres)
    double len = 0, width = 0;                // um, wires only
    int layer = -1;
};

struct RCNet {
    vector<RCNode> nodes;
    vector<RCRes> res;
    int nZeroR = 0;  // wire stretches on layers with no RSHEET
};

namespace rcdetail {
// closest point on an axis-parallel centreline [(x1,y1)-(x2,y2)] to point (px,py)
inline void closestOnLine(const Segment& w, double px, double py, double& ox, double& oy) {
    ox = min(max(px, w.cx1), w.cx2);
    oy = min(max(py, w.cy1), w.cy2);
}
// closest point on wire a's centreline to wire b's centreline (both axis-parallel)
inline void closestBetween(const Segment& a, const Segment& b, double& ox, double& oy) {
    if (b.dir == 'N') { closestOnLine(a, b.cx1, b.cy1, ox, oy); return; }
    // the pair of closest points of two axis-parallel segments: clamp each coordinate range against the other
    auto clampRange = [](double lo1, double hi1, double lo2, double hi2) {
        // coordinate on segment 1 closest to segment 2's range
        double lo = max(lo1, lo2), hi = min(hi1, hi2);
        if (lo <= hi) return 0.5 * (lo + hi);          // ranges overlap: any point; take the middle of the overlap
        return hi1 < lo2 ? hi1 : lo1;                 // disjoint: nearest end
    };
    ox = clampRange(a.cx1, a.cx2, b.cx1, b.cx2);
    oy = clampRange(a.cy1, a.cy2, b.cy1, b.cy2);
}
}  // namespace rcdetail

inline RCNet buildRC(const Design& d, const Tech& t, const Connectivity& c) {
    RCNet rc;
    const double EPS = 1e-7;

    // ---- raw nodes: (segment, coordinate along it) ----
    struct Raw { int seg; double x, y; };
    vector<Raw> raw;
    vector<vector<int>> wireCuts(d.segs.size());  // per wire: raw node ids on it (sorted later)
    vector<int> padNode(d.segs.size(), -1);       // per via pad: its raw node
    vector<pair<int, int>> merges;

    auto newRaw = [&](int seg, double x, double y) {
        raw.push_back({seg, x, y});
        return (int)raw.size() - 1;
    };
    auto cutAt = [&](int w, double x, double y) {  // reuse an existing cut within EPS, else make one
        for (int r : wireCuts[w])
            if (fabs(raw[r].x - x) < EPS && fabs(raw[r].y - y) < EPS) return r;
        int r = newRaw(w, x, y);
        wireCuts[w].push_back(r);
        return r;
    };

    for (const Segment& s : d.segs) {
        if (s.isVia) { padNode[s.id] = newRaw(s.id, s.cx1, s.cy1); continue; }
        if (s.dir == 'N') continue;
        cutAt(s.id, s.cx1, s.cy1);
        cutAt(s.id, s.cx2, s.cy2);
    }

    // ---- junctions: touching shapes of the same piece ----
    for (int p = 0; p < c.nPieces; ++p) {
        const auto& m = c.pieceSegs[p];
        for (size_t i = 0; i < m.size(); ++i)
            for (size_t j = i + 1; j < m.size(); ++j) {
                const Segment& a = d.segs[m[i]];
                const Segment& b = d.segs[m[j]];
                if (a.layer != b.layer || !touchesWithLength(a.box, b.box)) continue;
                if (a.isVia && b.isVia) continue;  // pads of different vias: joined only if a wire links them
                int na = -1, nb = -1;
                double x, y;
                if (!a.isVia) { rcdetail::closestBetween(a, b, x, y); na = cutAt(a.id, x, y); } else na = padNode[a.id];
                if (!b.isVia) { rcdetail::closestBetween(b, a, x, y); nb = cutAt(b.id, x, y); } else nb = padNode[b.id];
                merges.push_back({na, nb});
            }
    }

    // ---- MAX_SEG_LEN: extra cuts ----
    if (t.maxSegLen > 0) {
        for (const Segment& s : d.segs) {
            if (s.isVia || s.dir == 'N') continue;
            vector<double> pos;  // coordinate along the wire of each cut
            for (int r : wireCuts[s.id]) pos.push_back(s.dir == 'H' ? raw[r].x : raw[r].y);
            sort(pos.begin(), pos.end());
            vector<double> extra;
            for (size_t k = 0; k + 1 < pos.size(); ++k) {
                double gap = pos[k + 1] - pos[k];
                int parts = (int)ceil(gap / t.maxSegLen - 1e-9);
                for (int q = 1; q < parts; ++q) extra.push_back(pos[k] + gap * q / parts);
            }
            for (double e : extra) {
                if (s.dir == 'H') cutAt(s.id, e, s.cy1); else cutAt(s.id, s.cx1, e);
            }
        }
    }

    // ---- via resistors, or a merge if the via has no cut resistance ----
    struct ViaEdge { int via, na, nb; double ohm; };
    vector<ViaEdge> viaEdges;
    for (const ViaInst& v : d.vias) {
        for (size_t k = 1; k < v.pads.size(); ++k) {
            int na = padNode[v.pads[k - 1]], nb = padNode[v.pads[k]];
            double ohm = (v.nCuts > 0 && t.cutR.count(v.cutLayer)) ? t.cutR.at(v.cutLayer) / v.nCuts : 0;
            if (ohm > 0) viaEdges.push_back({v.id, na, nb, ohm});
            else merges.push_back({na, nb});
        }
    }

    // ---- collapse merged raw nodes ----
    DSU u((int)raw.size());
    for (auto& mg : merges) u.join(mg.first, mg.second);
    vector<int> nodeOf(raw.size(), -1);
    for (int r = 0; r < (int)raw.size(); ++r) {
        int root = u.find(r);
        if (nodeOf[root] < 0) {
            RCNode n;
            n.id = (int)rc.nodes.size();
            n.x = raw[root].x;
            n.y = raw[root].y;
            n.layer = d.segs[raw[root].seg].layer;
            n.net = d.segs[raw[root].seg].net;
            n.piece = c.piece[raw[root].seg];
            rc.nodes.push_back(n);
            nodeOf[root] = n.id;
        }
        nodeOf[r] = nodeOf[root];
    }

    // ---- wire resistors ----
    for (const Segment& s : d.segs) {
        if (s.isVia || s.dir == 'N') continue;
        vector<int> cuts = wireCuts[s.id];
        auto key = [&](int r) { return s.dir == 'H' ? raw[r].x : raw[r].y; };
        sort(cuts.begin(), cuts.end(), [&](int a, int b) { return key(a) < key(b); });
        double rs = t.layers[s.layer].rsheet;
        if (rs <= 0) ++rc.nZeroR;
        for (size_t k = 0; k + 1 < cuts.size(); ++k) {
            RCRes r;
            r.a = nodeOf[cuts[k]];
            r.b = nodeOf[cuts[k + 1]];
            r.len = key(cuts[k + 1]) - key(cuts[k]);
            r.width = s.width;
            r.ohm = rs * r.len / s.width;
            r.seg = s.id;
            r.layer = s.layer;
            r.x1 = raw[cuts[k]].x; r.y1 = raw[cuts[k]].y;
            r.x2 = raw[cuts[k + 1]].x; r.y2 = raw[cuts[k + 1]].y;
            rc.res.push_back(r);
        }
    }
    for (auto& e : viaEdges) {
        RCRes r;
        r.a = nodeOf[e.na];
        r.b = nodeOf[e.nb];
        r.ohm = e.ohm;
        r.via = e.via;
        r.layer = d.segs[raw[e.na].seg].layer;
        r.x1 = raw[e.na].x; r.y1 = raw[e.na].y;
        r.x2 = raw[e.nb].x; r.y2 = raw[e.nb].y;
        rc.res.push_back(r);
    }
    return rc;
}

// The resistor graph must hold together exactly as the connectivity says: one component per piece.
inline bool checkRC(const Design& d, const Connectivity& c, const RCNet& rc) {
    DSU u((int)rc.nodes.size());
    for (auto& r : rc.res) u.join(r.a, r.b);
    vector<int> compOfPiece(c.nPieces, -1);
    bool ok = true;
    for (auto& n : rc.nodes) {
        int root = u.find(n.id);
        if (compOfPiece[n.piece] < 0) compOfPiece[n.piece] = root;
        else if (compOfPiece[n.piece] != root) ok = false;  // a piece fell apart
    }
    set<int> seen;
    for (int p = 0; p < c.nPieces; ++p)
        if (compOfPiece[p] >= 0 && !seen.insert(compOfPiece[p]).second) ok = false;  // two pieces merged
    (void)d;
    return ok;
}

inline void reportRC(const Design& d, const Tech& t, const RCNet& rc) {
    printf("\nResistor network (R only): %zu nodes, %zu resistors\n", rc.nodes.size(), rc.res.size());
    if (rc.nZeroR) printf("  warning: %d wire(s) on layers with no RSHEET have R = 0\n", rc.nZeroR);
    for (size_t net = 0; net < d.netNames.size(); ++net) {
        int nn = 0, nr = 0;
        double tot = 0;
        for (auto& n : rc.nodes) if (n.net == (int)net) ++nn;
        for (auto& r : rc.res) if (rc.nodes[r.a].net == (int)net) { ++nr; tot += r.ohm; }
        if (!nn) continue;
        printf("  net %-8s %3d nodes %3d resistors  sum of all R = %.4f ohm\n", d.netNames[net].c_str(), nn, nr, tot);
    }
    if (rc.res.size() <= 40) {
        for (auto& r : rc.res) {
            if (r.via >= 0) printf("    n%-3d -- n%-3d  %10.4f ohm   via %s\n", r.a, r.b, r.ohm, d.vias[r.via].name.c_str());
            else printf("    n%-3d -- n%-3d  %10.4f ohm   %s wire %d  L=%.3f W=%.3f\n", r.a, r.b, r.ohm,
                        t.layers[r.layer].name.c_str(), r.seg, r.len, r.width);
        }
    }
}