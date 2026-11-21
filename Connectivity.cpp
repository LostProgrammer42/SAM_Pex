// Connectivity.cpp -- which shapes of a net are electrically one piece?
// Included by Pex_Test.cpp (unity build, after Helpers.cpp and Quad_Trees.cpp).
//
// Rules
//   * same layer, same net, rectangles overlap or share an edge of positive length -> connected
//   * corner-only touch (shared edge length 0) -> NOT connected
//   * different layers connect only through a via instance (its pads are all joined)
//   * same layer, different net, overlap or edge contact -> reported as a SHORT
#pragma once
#include <algorithm>
#include <functional>

struct DSU {
    vector<int> p;
    explicit DSU(int n) : p(n) { for (int i = 0; i < n; ++i) p[i] = i; }
    int find(int x) { while (p[x] != x) { p[x] = p[p[x]]; x = p[x]; } return x; }
    void join(int a, int b) { a = find(a); b = find(b); if (a != b) p[max(a, b)] = min(a, b); }
};

// true if the rectangles share area, or share an edge segment of length > 0
inline bool touchesWithLength(const Rect& a, const Rect& b) {
    double ox = min(a.x2, b.x2) - max(a.x1, b.x1);  // overlap extent along x (negative = gap)
    double oy = min(a.y2, b.y2) - max(a.y1, b.y1);
    if (ox < -GEOM_EPS || oy < -GEOM_EPS) return false;  // gap
    return ox > GEOM_EPS || oy > GEOM_EPS;               // both ~0 = corner touch only
}

struct Short { int a, b; };

struct Connectivity {
    vector<int> piece;               // per segment: piece id (global, 0..nPieces-1)
    int nPieces = 0;
    vector<vector<int>> pieceSegs;   // piece id -> segment ids
    vector<int> pieceNet;            // piece id -> net
    vector<Short> shorts;            // touching shapes of different nets (same layer)
};

// pairs are found through the per-layer quad trees (trees[layer].query)
inline Connectivity buildConnectivity(const Design& d, vector<QuadTree>& trees) {
    int n = (int)d.segs.size();
    DSU u(n);
    Connectivity c;
    vector<pair<int, int>> cand;
    for (const Segment& s : d.segs) {
        trees[s.layer].query(s.box.expanded(GEOM_EPS), [&](int j) {
            if (j > s.id) cand.push_back({s.id, j});
        });
    }
    // evaluate after the queries finish (the trees are not re-entrant)
    for (auto& pr : cand) {
        const Segment& a = d.segs[pr.first];
        const Segment& b = d.segs[pr.second];
        if (!touchesWithLength(a.box, b.box)) continue;
        if (a.net == b.net) u.join(a.id, b.id);
        else c.shorts.push_back({a.id, b.id});
    }
    for (const ViaInst& v : d.vias)
        for (size_t k = 1; k < v.pads.size(); ++k) u.join(v.pads[0], v.pads[k]);

    vector<int> rootToPiece(n, -1);
    c.piece.assign(n, -1);
    for (int i = 0; i < n; ++i) {
        int r = u.find(i);
        if (rootToPiece[r] < 0) {
            rootToPiece[r] = c.nPieces++;
            c.pieceSegs.emplace_back();
            c.pieceNet.push_back(d.segs[i].net);
        }
        c.piece[i] = rootToPiece[r];
        c.pieceSegs[c.piece[i]].push_back(i);
    }
    return c;
}

// O(n^2) reference used to validate the tree-based version
inline Connectivity buildConnectivityBrute(const Design& d) {
    int n = (int)d.segs.size();
    DSU u(n);
    Connectivity c;
    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j) {
            const Segment& a = d.segs[i];
            const Segment& b = d.segs[j];
            if (a.layer != b.layer || !touchesWithLength(a.box, b.box)) continue;
            if (a.net == b.net) u.join(i, j);
            else c.shorts.push_back({i, j});
        }
    for (const ViaInst& v : d.vias)
        for (size_t k = 1; k < v.pads.size(); ++k) u.join(v.pads[0], v.pads[k]);
    vector<int> rootToPiece(n, -1);
    c.piece.assign(n, -1);
    for (int i = 0; i < n; ++i) {
        int r = u.find(i);
        if (rootToPiece[r] < 0) {
            rootToPiece[r] = c.nPieces++;
            c.pieceSegs.emplace_back();
            c.pieceNet.push_back(d.segs[i].net);
        }
        c.piece[i] = rootToPiece[r];
        c.pieceSegs[c.piece[i]].push_back(i);
    }
    return c;
}

inline bool sameConnectivity(const Connectivity& a, const Connectivity& b) {
    if (a.nPieces != b.nPieces || a.piece != b.piece) return false;
    auto norm = [](vector<Short> v) {
        for (auto& s : v) if (s.a > s.b) swap(s.a, s.b);
        sort(v.begin(), v.end(), [](const Short& x, const Short& y) { return x.a != y.a ? x.a < y.a : x.b < y.b; });
        return v;
    };
    auto sa = norm(a.shorts), sb = norm(b.shorts);
    if (sa.size() != sb.size()) return false;
    for (size_t i = 0; i < sa.size(); ++i) if (sa[i].a != sb[i].a || sa[i].b != sb[i].b) return false;
    return true;
}

inline void reportConnectivity(const Design& d, const Tech& t, const Connectivity& c) {
    printf("\nConnectivity: %d shapes, %d vias -> %d connected pieces, %zu shorts\n",
           (int)d.segs.size(), (int)d.vias.size(), c.nPieces, c.shorts.size());
    for (size_t net = 0; net < d.netNames.size(); ++net) {
        vector<int> mine;
        int nshapes = 0;
        for (int p = 0; p < c.nPieces; ++p)
            if (c.pieceNet[p] == (int)net) { mine.push_back(p); nshapes += (int)c.pieceSegs[p].size(); }
        if (mine.empty()) continue;
        printf("  net %-8s %2d shapes  %d piece%s%s\n", d.netNames[net].c_str(), nshapes, (int)mine.size(),
               mine.size() == 1 ? "" : "s", mine.size() > 1 ? "   <-- not fully connected (open or fragmented)" : "");
        for (int p : mine) {
            printf("    piece %-3d:", p);
            for (int id : c.pieceSegs[p]) {
                const Segment& s = d.segs[id];
                printf(" %s%d(%s)", s.isVia ? "v" : "w", id, t.layers[s.layer].name.c_str());
            }
            printf("\n");
        }
    }
    for (const Short& s : c.shorts)
        printf("  SHORT: shape %d (net %s) touches shape %d (net %s) on %s\n", s.a, d.netNames[d.segs[s.a].net].c_str(),
               s.b, d.netNames[d.segs[s.b].net].c_str(), t.layers[d.segs[s.a].layer].name.c_str());
    for (const ViaInst& v : d.vias)
        if (v.nCuts > 0)
            printf("  via %-18s net %-6s at (%.3f, %.3f)  %s x%d  R = %.3f ohm\n", v.name.c_str(),
                   d.netNames[v.net].c_str(), v.x, v.y, v.cutLayer.c_str(), v.nCuts,
                   t.cutR.at(v.cutLayer) / v.nCuts);
}