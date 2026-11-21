// Merge.cpp -- merge fragmented wires of the same net into proper rectangles.
// Included by Pex_Test.cpp (unity build) after Connectivity.cpp; runs right after the DEF is parsed.
//
// Why: a DEF often draws one physical wire as several stripes (a 0.29 um rail as 0.04 + 0.22 + 0.03 wide
// stripes, or one wire cut into end-to-end pieces). Treated as separate wires they give parallel resistors
// tied at one point, and double-counted shapes.
//
// What: within one net and layer, wires that touch form a cluster. If the cluster holds two touching wires of
// the SAME direction (side by side, end to end, or overlapping), the cluster is one fragmented shape: its union
// is cut into non-overlapping rectangles by a slab sweep (once with horizontal slabs, once with vertical ones,
// keeping the one with fewer rectangles, ties -> the one with the biggest single rectangle, then horizontal).
// Every resulting rectangle becomes one wire: its long side is the direction, its short side the width.
// Clusters with only perpendicular touches (plain L / T turns) are left exactly as they were.
// Via pads are never touched.
//
// Switch off with  MERGE_FRAGMENTS OFF  in the tech file or  --merge=off  on the command line.
#pragma once

struct MergeStats {
    int wiresBefore = 0, wiresAfter = 0;
    int clustersMerged = 0;
};

namespace mergedetail {
inline long long q(double v) { return llround(v * 1e6); }  // snap to 1e-6 um so comparisons are exact

struct IRect { long long x1, y1, x2, y2; };

// Cut the union of `in` into rectangles using slabs along y (horizontal slabs) and then fuse slabs that have
// the same x-interval. `transpose` swaps x and y first (and back at the end) to get vertical slabs.
inline vector<IRect> slabDecompose(const vector<IRect>& in, bool transpose) {
    vector<IRect> r = in;
    if (transpose) for (auto& a : r) { swap(a.x1, a.y1); swap(a.x2, a.y2); }
    vector<long long> ys;
    for (auto& a : r) { ys.push_back(a.y1); ys.push_back(a.y2); }
    sort(ys.begin(), ys.end());
    ys.erase(unique(ys.begin(), ys.end()), ys.end());
    vector<IRect> out;
    vector<IRect> open;  // rectangles still growing upward
    for (size_t i = 0; i + 1 < ys.size(); ++i) {
        long long lo = ys[i], hi = ys[i + 1];
        vector<pair<long long, long long>> iv;  // covered x-intervals in this slab
        for (auto& a : r) if (a.y1 <= lo && a.y2 >= hi) iv.push_back({a.x1, a.x2});
        sort(iv.begin(), iv.end());
        vector<pair<long long, long long>> m;
        for (auto& p : iv) {
            if (!m.empty() && p.first <= m.back().second) m.back().second = max(m.back().second, p.second);
            else m.push_back(p);
        }
        vector<IRect> nextOpen;
        for (auto& p : m) {
            bool fused = false;
            for (auto it = open.begin(); it != open.end(); ++it)
                if (it->x1 == p.first && it->x2 == p.second && it->y2 == lo) {
                    it->y2 = hi;
                    nextOpen.push_back(*it);
                    open.erase(it);
                    fused = true;
                    break;
                }
            if (!fused) nextOpen.push_back({p.first, lo, p.second, hi});
        }
        for (auto& o : open) out.push_back(o);  // not continued: finished
        open = nextOpen;
    }
    for (auto& o : open) out.push_back(o);
    if (transpose) for (auto& a : out) { swap(a.x1, a.y1); swap(a.x2, a.y2); }
    return out;
}

inline double maxArea(const vector<IRect>& v) {
    double m = 0;
    for (auto& a : v) m = max(m, (double)(a.x2 - a.x1) * (double)(a.y2 - a.y1));
    return m;
}
}  // namespace mergedetail

inline MergeStats mergeFragments(Design& d) {
    using namespace mergedetail;
    MergeStats st;
    int n = (int)d.segs.size();
    for (auto& s : d.segs) if (!s.isVia && s.dir != 'N') ++st.wiresBefore;

    // 1. clusters of touching wires per (net, layer)
    DSU u(n);
    map<pair<int, int>, vector<int>> groups;
    for (auto& s : d.segs) if (!s.isVia && s.dir != 'N') groups[{s.net, s.layer}].push_back(s.id);
    vector<char> sameDirTouch(n, 0);  // marks a wire that touches another wire of the same direction
    for (auto& g : groups)
        for (size_t i = 0; i < g.second.size(); ++i)
            for (size_t j = i + 1; j < g.second.size(); ++j) {
                const Segment& a = d.segs[g.second[i]];
                const Segment& b = d.segs[g.second[j]];
                if (!touchesWithLength(a.box, b.box)) continue;
                u.join(a.id, b.id);
                if (a.dir == b.dir) sameDirTouch[a.id] = sameDirTouch[b.id] = 1;
            }
    map<int, vector<int>> clusters;
    for (auto& g : groups) for (int id : g.second) clusters[u.find(id)].push_back(id);

    // 2. decide per cluster
    map<int, vector<IRect>> replacement;  // first member id -> new rectangles
    set<int> dropped;                     // ids replaced by a merge
    for (auto& cl : clusters) {
        const vector<int>& ids = cl.second;
        if (ids.size() < 2) continue;
        bool frag = false;
        for (int id : ids) if (sameDirTouch[id]) frag = true;
        if (!frag) continue;  // only perpendicular touches: a plain turn, leave it
        vector<IRect> in;
        for (int id : ids) {
            const Rect& b = d.segs[id].box;
            in.push_back({q(b.x1), q(b.y1), q(b.x2), q(b.y2)});
        }
        auto h = slabDecompose(in, false), v = slabDecompose(in, true);
        const vector<IRect>& best = (v.size() < h.size() || (v.size() == h.size() && maxArea(v) > maxArea(h))) ? v : h;
        {   // already clean? (same rectangles as before) then leave the wires, ids and all, untouched
            auto key = [](vector<IRect> v) {
                sort(v.begin(), v.end(), [](const IRect& a, const IRect& b) {
                    return tie(a.x1, a.y1, a.x2, a.y2) < tie(b.x1, b.y1, b.x2, b.y2);
                });
                return v;
            };
            auto a = key(in), b = key(best);
            bool same = a.size() == b.size();
            for (size_t k = 0; same && k < a.size(); ++k)
                same = a[k].x1 == b[k].x1 && a[k].y1 == b[k].y1 && a[k].x2 == b[k].x2 && a[k].y2 == b[k].y2;
            if (same) continue;
        }
        int first = *min_element(ids.begin(), ids.end());
        replacement[first] = best;
        for (int id : ids) dropped.insert(id);
        ++st.clustersMerged;
    }

    // 3. rebuild the segment list (ids are renumbered; via pads follow)
    vector<Segment> out;
    vector<int> oldToNew(n, -1);
    for (int i = 0; i < n; ++i) {
        const Segment& s = d.segs[i];
        auto rp = replacement.find(i);
        if (rp != replacement.end()) {
            for (const IRect& r : rp->second) {
                Segment w;
                w.id = (int)out.size();
                w.net = s.net;
                w.layer = s.layer;
                w.isVia = false;
                w.box = Rect{r.x1 * 1e-6, r.y1 * 1e-6, r.x2 * 1e-6, r.y2 * 1e-6};
                double bw = w.box.x2 - w.box.x1, bh = w.box.y2 - w.box.y1;
                if (bw >= bh) {
                    w.dir = 'H'; w.width = bh;
                    w.cx1 = w.box.x1; w.cx2 = w.box.x2; w.cy1 = w.cy2 = 0.5 * (w.box.y1 + w.box.y2);
                } else {
                    w.dir = 'V'; w.width = bw;
                    w.cy1 = w.box.y1; w.cy2 = w.box.y2; w.cx1 = w.cx2 = 0.5 * (w.box.x1 + w.box.x2);
                }
                out.push_back(w);
            }
        } else if (dropped.count(i)) {
            // absorbed into the replacement emitted at its cluster's first member
        } else {
            Segment w = s;
            w.id = (int)out.size();
            oldToNew[i] = w.id;
            out.push_back(w);
        }
    }
    for (auto& v : d.vias) for (int& p : v.pads) p = oldToNew[p];
    d.segs = out;
    for (auto& s : d.segs) if (!s.isVia && s.dir != 'N') ++st.wiresAfter;
    return st;
}