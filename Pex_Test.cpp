// Pex_Test.cpp -- parasitic extraction driver (coupling capacitance, analytic model).
// Unity build: compile ONLY this file.
//   g++ -O2 -std=c++17 Pex_Test.cpp -o pex
//   ./pex [design.def] [tech.cfg] [out.sampex]
#include "Helpers.cpp"
#include "Quad_Trees.cpp"

#include <chrono>
#include <functional>

// ---------------------------------------------------------------- coupling model
struct Coupling {
    double side = 0;     // same-layer sidewall coupling, fF
    double overlap = 0;  // adjacent-layer overlap coupling, fF
    double total() const { return side + overlap; }
};

// The dielectric slab between the two facing surfaces of one contributing pair (microns).
// Exported so the 3D visualizer can show exactly which walls a capacitance is measured across.
struct PairGeom {
    double x1 = 0, y1 = 0, z1 = 0, x2 = 0, y2 = 0, z2 = 0;  // slab box (x1<=x2 ...)
    double gap = 0;     // sidewall: edge-to-edge spacing; overlap: vertical dielectric gap
    double extent = 0;  // sidewall: parallel run length;  overlap: overlap area (um^2)
    char axis = 'z';    // direction across which the two walls face each other: 'x', 'y' or 'z'
    double clearRun = 0;  // sidewall: parallel run left after slab blocking (== extent if nothing blocks)
};

struct PairRec {
    int a = -1, b = -1;  // segment ids
    Coupling raw;        // before slab blocking (every facing pair counted)
    Coupling c;          // after slab blocking (what is actually accumulated)
    PairGeom g;
    vector<int> blockers;  // shapes sitting inside the slab
};

// Coupling between two shapes. Symmetric. Zero for same-net pairs.
//  same layer     : C = eps * fringe * thickness * parallel_run / spacing   (spacing <= HALO)
//  adjacent layer : C = eps * overlap_area / vertical_gap
// Different-net shapes that touch/overlap on one layer (a short) are ignored.
inline Coupling pairCoupling(const Tech& t, const Segment& a, const Segment& b, PairGeom* g = nullptr) {
    Coupling c;
    if (a.net == b.net) return c;
    if (a.layer == b.layer) {
        double sx = max(a.box.x1, b.box.x1) - min(a.box.x2, b.box.x2);  // >0: gap in x, <0: overlap in x
        double sy = max(a.box.y1, b.box.y1) - min(a.box.y2, b.box.y2);
        double spacing, prl;
        bool gapInX;
        if (sx > GEOM_EPS && sy < -GEOM_EPS) {
            spacing = sx;
            prl = -sy;
            gapInX = true;
        } else if (sy > GEOM_EPS && sx < -GEOM_EPS) {
            spacing = sy;
            prl = -sx;
            gapInX = false;
        } else {
            return c;  // diagonal neighbours, touching, or overlapping
        }
        if (spacing > t.halo) return c;
        const Layer& L = t.layers[a.layer];
        if (t.useTable)
            c.side = prl * t.curves[a.layer].at(spacing);  // captable value is fF per um of run
        else
            c.side = t.eps() * t.fringe * L.thickness * prl / spacing;
        if (g) {
            if (gapInX) {  // slab spans the x gap, and the y overlap
                g->x1 = min(a.box.x2, b.box.x2);
                g->x2 = max(a.box.x1, b.box.x1);
                g->y1 = max(a.box.y1, b.box.y1);
                g->y2 = min(a.box.y2, b.box.y2);
            } else {
                g->y1 = min(a.box.y2, b.box.y2);
                g->y2 = max(a.box.y1, b.box.y1);
                g->x1 = max(a.box.x1, b.box.x1);
                g->x2 = min(a.box.x2, b.box.x2);
            }
            g->z1 = L.zBottom;
            g->z2 = L.zBottom + L.thickness;
            g->gap = spacing;
            g->extent = prl;
            g->axis = gapInX ? 'x' : 'y';
        }
    } else if (abs(a.layer - b.layer) == 1) {
        double ox = min(a.box.x2, b.box.x2) - max(a.box.x1, b.box.x1);
        double oy = min(a.box.y2, b.box.y2) - max(a.box.y1, b.box.y1);
        if (ox > GEOM_EPS && oy > GEOM_EPS) {
            int lo = min(a.layer, b.layer);
            c.overlap = t.eps() * ox * oy / t.gapAbove(lo);
            if (g) {  // slab = overlap rectangle, between top of lower layer and bottom of upper layer
                g->x1 = max(a.box.x1, b.box.x1);
                g->x2 = min(a.box.x2, b.box.x2);
                g->y1 = max(a.box.y1, b.box.y1);
                g->y2 = min(a.box.y2, b.box.y2);
                g->z1 = t.layers[lo].zBottom + t.layers[lo].thickness;
                g->z2 = t.layers[lo + 1].zBottom;
                g->gap = t.gapAbove(lo);
                g->extent = ox * oy;
                g->axis = 'z';
            }
        }
    }
    return c;
}

using NetPair = pair<int, int>;
using CouplingMap = map<NetPair, Coupling>;

// Finds shapes on `layer` whose box overlaps r and calls cb(id) for each (duplicates are harmless).
// The tree extractor answers this with a quad tree query, the brute-force one with a linear scan.
using BlockerFinder = function<void(int layer, const Rect& r, const function<void(int)>& cb)>;

// Slab blocking. A same-layer shape sitting inside the slab between two facing walls (any net,
// including the plates' own nets) intercepts the field there. Its footprint along the run
// direction is removed from the parallel run, and C is scaled by the unblocked fraction.
// A fully covered run contributes nothing.
inline void applyShielding(const vector<Segment>& segs, const Segment& a, const Segment& b, PairGeom& g,
                           Coupling& c, vector<int>& blockers, const BlockerFinder& find) {
    g.clearRun = g.extent;
    if (c.side <= 0) return;  // only sidewall pairs: nothing sits between adjacent layers
    bool gapInX = (g.axis == 'x');
    double lo = gapInX ? g.y1 : g.x1, hi = gapInX ? g.y2 : g.x2;  // run interval along the facing walls
    Rect slab{g.x1, g.y1, g.x2, g.y2};
    vector<pair<double, double>> iv;
    find(a.layer, slab, [&](int id) {
        if (id == a.id || id == b.id) return;
        const Rect& r = segs[id].box;
        double ox = min(r.x2, slab.x2) - max(r.x1, slab.x1);
        double oy = min(r.y2, slab.y2) - max(r.y1, slab.y1);
        if (ox <= GEOM_EPS || oy <= GEOM_EPS) return;  // touching only, not inside the slab
        iv.push_back(gapInX ? make_pair(max(r.y1, lo), min(r.y2, hi)) : make_pair(max(r.x1, lo), min(r.x2, hi)));
        blockers.push_back(id);
    });
    if (iv.empty()) return;
    sort(blockers.begin(), blockers.end());
    blockers.erase(unique(blockers.begin(), blockers.end()), blockers.end());
    sort(iv.begin(), iv.end());
    double covered = 0, curLo = iv[0].first, curHi = iv[0].second;  // union of blocker footprints
    for (size_t k = 1; k < iv.size(); ++k) {
        if (iv[k].first > curHi) {
            covered += curHi - curLo;
            curLo = iv[k].first;
            curHi = iv[k].second;
        } else {
            curHi = max(curHi, iv[k].second);
        }
    }
    covered += curHi - curLo;
    double clear = (hi - lo) - covered;
    if (clear <= GEOM_EPS) {
        c.side = 0;
        g.clearRun = 0;
    } else {
        c.side *= clear / (hi - lo);  // C is proportional to the run length
        g.clearRun = clear;
    }
}

// Evaluates one candidate pair and adds it to the map. If sink is non-null, every facing pair is
// also recorded (including fully blocked ones, with C = 0) so the visualizer can show them.
inline void accumulate(CouplingMap& m, const Tech& t, const vector<Segment>& segs, int i, int j,
                       const BlockerFinder& find, vector<PairRec>* sink = nullptr) {
    PairGeom g;
    Coupling raw = pairCoupling(t, segs[i], segs[j], &g);
    if (raw.total() == 0) return;
    Coupling c = raw;
    vector<int> blockers;
    applyShielding(segs, segs[i], segs[j], g, c, blockers, find);
    if (c.total() > 0) {
        NetPair key(min(segs[i].net, segs[j].net), max(segs[i].net, segs[j].net));
        Coupling& acc = m[key];
        acc.side += c.side;
        acc.overlap += c.overlap;
    }
    if (sink) {
        PairRec r;
        r.a = i;
        r.b = j;
        r.raw = raw;
        r.c = c;
        r.g = g;
        r.blockers = blockers;
        sink->push_back(r);
    }
}

// ---------------------------------------------------------------- extraction
long gTreeCandidates = 0, gBruteCandidates = 0;

CouplingMap extractWithTrees(const Tech& t, const vector<Segment>& segs, vector<QuadTree>& trees,
                             vector<PairRec>* sink = nullptr) {
    CouplingMap m;
    int nl = (int)trees.size();
    BlockerFinder find = [&](int layer, const Rect& r, const function<void(int)>& cb) {
        trees[layer].query(r, cb);
    };
    vector<int> cand;
    for (const Segment& s : segs) {
        int i = s.id;
        cand.clear();
        // same layer: neighbours within the halo; count each pair once (j > i)
        trees[s.layer].query(s.box.expanded(t.halo + GEOM_EPS), [&](int j) {
            if (j > i) cand.push_back(j);
        });
        // layer above: shapes overlapping this box (each cross-layer pair seen once, from the lower shape)
        if (s.layer + 1 < nl) {
            trees[s.layer + 1].query(s.box, [&](int j) { cand.push_back(j); });
        }
        // Evaluate only after the queries above have finished: slab blocking runs its own tree
        // queries, and a tree's visit stamps are not reentrant.
        for (int j : cand) {
            ++gTreeCandidates;
            accumulate(m, t, segs, i, j, find, sink);
        }
    }
    return m;
}

CouplingMap extractBrute(const Tech& t, const vector<Segment>& segs) {
    CouplingMap m;
    BlockerFinder find = [&](int layer, const Rect& r, const function<void(int)>& cb) {
        for (const Segment& s : segs)
            if (s.layer == layer && overlapsClosed(s.box, r)) cb(s.id);
    };
    for (size_t i = 0; i < segs.size(); ++i)
        for (size_t j = i + 1; j < segs.size(); ++j) {
            if (abs(segs[i].layer - segs[j].layer) > 1) continue;
            ++gBruteCandidates;
            accumulate(m, t, segs, (int)i, (int)j, find);
        }
    return m;
}

bool sameResult(const CouplingMap& a, const CouplingMap& b) {
    if (a.size() != b.size()) return false;
    auto ia = a.begin();
    auto ib = b.begin();
    for (; ia != a.end(); ++ia, ++ib) {
        if (ia->first != ib->first) return false;
        double sa = ia->second.side, sb = ib->second.side;
        double oa = ia->second.overlap, ob = ib->second.overlap;
        if (fabs(sa - sb) > 1e-9 * max(1.0, fabs(sa))) return false;
        if (fabs(oa - ob) > 1e-9 * max(1.0, fabs(oa))) return false;
    }
    return true;
}

// ---------------------------------------------------------------- export
static string jsonEsc(const string& s) {
    string o;
    for (char c : s) {
        if (c == '"' || c == '\\') o += '\\';
        o += c;
    }
    return o;
}

bool writeSampex(const string& path, const Design& d, const Tech& t, const vector<QuadTree>& trees,
                 const CouplingMap& cm, const vector<PairRec>& pairs) {
    FILE* f = fopen(path.c_str(), "w");
    if (!f) {
        fprintf(stderr, "error: cannot write '%s'\n", path.c_str());
        return false;
    }
    fprintf(f, "{\n\"design\":\"%s\",\n", jsonEsc(d.name).c_str());
    fprintf(f, "\"die\":[%.6f,%.6f,%.6f,%.6f],\n", d.die.x1, d.die.y1, d.die.x2, d.die.y2);
    fprintf(f, "\"layers\":[");
    for (size_t i = 0; i < t.layers.size(); ++i)
        fprintf(f, "%s\"%s\"", i ? "," : "", jsonEsc(t.layers[i].name).c_str());
    fprintf(f, "],\n\"layer_z\":[");  // [z_bottom, thickness] per layer, same order as "layers"
    for (size_t i = 0; i < t.layers.size(); ++i)
        fprintf(f, "%s[%.6f,%.6f]", i ? "," : "", t.layers[i].zBottom, t.layers[i].thickness);
    fprintf(f, "],\n\"halo\":%.6f,\n\"coupling_model\":\"%s\",\n\"nets\":[", t.halo,
            t.useTable ? "table" : "analytic");
    for (size_t i = 0; i < d.netNames.size(); ++i)
        fprintf(f, "%s{\"name\":\"%s\",\"supply\":%s}", i ? "," : "", jsonEsc(d.netNames[i]).c_str(),
                d.netIsSupply[i] ? "true" : "false");
    fprintf(f, "],\n\"segments\":[\n");
    for (size_t i = 0; i < d.segs.size(); ++i) {
        const Segment& s = d.segs[i];
        fprintf(f, "%s{\"id\":%d,\"net\":%d,\"layer\":%d,\"via\":%s,\"llx\":%.6f,\"lly\":%.6f,\"wx\":%.6f,\"wy\":%.6f}",
                i ? ",\n" : "", s.id, s.net, s.layer, s.isVia ? "true" : "false", s.box.x1, s.box.y1,
                s.box.w(), s.box.h());
    }
    fprintf(f, "\n],\n\"trees\":[\n");
    for (size_t i = 0; i < trees.size(); ++i) {
        fprintf(f, "%s{\"layer\":%zu,\"nodes\":", i ? ",\n" : "", i);
        trees[i].writeNodesJson(f);
        fprintf(f, "}");
    }
    fprintf(f, "\n],\n\"coupling\":[");
    bool first = true;
    for (auto& kv : cm) {
        fprintf(f, "%s{\"a\":%d,\"b\":%d,\"side_fF\":%.9g,\"overlap_fF\":%.9g}", first ? "" : ",",
                kv.first.first, kv.first.second, kv.second.side, kv.second.overlap);
        first = false;
    }
    // One entry per contributing shape pair: which two shapes, what kind, and the slab between them
    fprintf(f, "],\n\"pairs\":[\n");
    for (size_t i = 0; i < pairs.size(); ++i) {
        const PairRec& r = pairs[i];
        bool side = r.raw.side > 0;  // classify from the raw value: a fully blocked pair has final C = 0
        fprintf(f,
                "%s{\"a\":%d,\"b\":%d,\"type\":\"%s\",\"fF\":%.9g,\"raw_fF\":%.9g,\"gap\":%.6f,\"extent\":%.6f,"
                "\"clear\":%.6f,\"axis\":\"%c\",\"box\":[%.6f,%.6f,%.6f,%.6f,%.6f,%.6f],\"blockers\":[",
                i ? ",\n" : "", r.a, r.b, side ? "side" : "overlap", r.c.total(), r.raw.total(), r.g.gap,
                r.g.extent, r.g.clearRun, r.g.axis, r.g.x1, r.g.y1, r.g.z1, r.g.x2, r.g.y2, r.g.z2);
        for (size_t k = 0; k < r.blockers.size(); ++k) fprintf(f, "%s%d", k ? "," : "", r.blockers[k]);
        fprintf(f, "]}");
    }
    fprintf(f, "\n]\n}\n");
    fclose(f);
    return true;
}

// ---------------------------------------------------------------- main
// usage: pex [design.def] [tech.cfg] [out.sampex] [--model=analytic|table]
//   --model overrides COUPLING_MODEL from the tech file, handy for comparing the two on the same design.
int main(int argc, char** argv) {
    vector<string> pos;
    string modelOverride;
    for (int i = 1; i < argc; ++i) {
        string a = argv[i];
        if (a.rfind("--model=", 0) == 0) modelOverride = a.substr(8);
        else if (a.rfind("--", 0) == 0) {
            fprintf(stderr, "error: unknown option '%s'\nusage: pex [design.def] [tech.cfg] [out.sampex] [--model=analytic|table]\n",
                    a.c_str());
            return 1;
        } else pos.push_back(a);
    }
    string defPath = pos.size() > 0 ? pos[0] : "test.def";
    string techPath = pos.size() > 1 ? pos[1] : "tech.cfg";
    string outPath = pos.size() > 2 ? pos[2] : "out.sampex";

    Tech tech;
    if (!loadTech(techPath, tech)) return 1;
    if (!modelOverride.empty()) {
        if (modelOverride == "table") tech.useTable = true;
        else if (modelOverride == "analytic") tech.useTable = false;
        else {
            fprintf(stderr, "error: --model must be analytic or table, got '%s'\n", modelOverride.c_str());
            return 1;
        }
        if (!checkCouplingModel(tech)) return 1;
    }
    printf("Coupling model: %s", tech.useTable ? "TABLE" : "ANALYTIC (parallel-plate formula)");
    if (tech.useTable) printf("  (%s)", tech.captableFile.c_str());
    printf("\n");

    Design design;
    DefParser parser(tech, design);
    if (!parser.parseFile(defPath)) return 1;
    if (design.segs.empty()) {
        fprintf(stderr, "error: no routing shapes found on tech layers\n");
        return 1;
    }

    // Tree bounds: die area united with every shape (shapes may stick out of the die)
    Rect bounds = design.die;
    for (const Segment& s : design.segs) {
        bounds.x1 = min(bounds.x1, s.box.x1);
        bounds.y1 = min(bounds.y1, s.box.y1);
        bounds.x2 = max(bounds.x2, s.box.x2);
        bounds.y2 = max(bounds.y2, s.box.y2);
    }
    bounds = bounds.expanded(0.01);

    printf("Design %s: %zu nets, %zu shapes, %zu layers, dbu=%g\n", design.name.c_str(), design.netNames.size(),
           design.segs.size(), tech.layers.size(), design.dbu);

    vector<QuadTree> trees;
    for (size_t L = 0; L < tech.layers.size(); ++L) trees.emplace_back(bounds, &design.segs, 8, 10);
    for (const Segment& s : design.segs) trees[s.layer].insert(s.id);

    printf("\nQuad tree stats (capacity 8, max depth 10):\n");
    for (size_t L = 0; L < trees.size(); ++L) {
        auto st = trees[L].stats();
        printf("  %-6s nodes=%d leaves=%d maxDepth=%d avg/leaf=%.2f max/leaf=%d dupFactor=%.2f\n",
               tech.layers[L].name.c_str(), st.nodes, st.leaves, st.maxDepth, st.avgPerLeaf, st.maxPerLeaf,
               st.dupFactor);
    }

    auto t0 = chrono::steady_clock::now();
    vector<PairRec> pairRecs;
    CouplingMap viaTree = extractWithTrees(tech, design.segs, trees, &pairRecs);
    auto t1 = chrono::steady_clock::now();
    CouplingMap brute = extractBrute(tech, design.segs);
    auto t2 = chrono::steady_clock::now();

    bool ok = sameResult(viaTree, brute);
    printf("\nValidation vs brute force: %s\n", ok ? "PASS (identical)" : "FAIL (mismatch!)");
    printf("  candidate pairs evaluated: tree=%ld  brute=%ld\n", gTreeCandidates, gBruteCandidates);
    printf("  time: tree=%.3f ms  brute=%.3f ms\n", chrono::duration<double, milli>(t1 - t0).count(),
           chrono::duration<double, milli>(t2 - t1).count());

    // Table, largest first
    vector<pair<NetPair, Coupling>> rows(viaTree.begin(), viaTree.end());
    sort(rows.begin(), rows.end(), [](auto& a, auto& b) { return a.second.total() > b.second.total(); });
    printf("\nCoupling capacitance per net pair (fF)   [* = involves a supply net]\n");
    printf("  %-8s %-8s %12s %12s %12s\n", "netA", "netB", "sidewall", "overlap", "total");
    for (auto& r : rows) {
        bool sup = design.netIsSupply[r.first.first] || design.netIsSupply[r.first.second];
        printf("  %-8s %-8s %12.6f %12.6f %12.6f %s\n", design.netNames[r.first.first].c_str(),
               design.netNames[r.first.second].c_str(), r.second.side, r.second.overlap, r.second.total(),
               sup ? "*" : "");
    }

    // What slab blocking changed: raw = every facing pair counted (the old behaviour)
    map<NetPair, double> rawSum;
    int fullyBlocked = 0, partlyBlocked = 0;
    for (const PairRec& r : pairRecs) {
        rawSum[NetPair(min(design.segs[r.a].net, design.segs[r.b].net),
                       max(design.segs[r.a].net, design.segs[r.b].net))] += r.raw.total();
        if (r.raw.total() > 0 && r.c.total() == 0) ++fullyBlocked;
        else if (r.c.total() < r.raw.total() * (1 - 1e-12)) ++partlyBlocked;
    }
    printf("\nSlab blocking (raw = every facing pair counted; final = blocked runs removed):\n");
    printf("  %-8s %-8s %12s %12s %9s\n", "netA", "netB", "raw", "final", "removed");
    for (auto& r : rows) {
        double raw = rawSum[r.first];
        printf("  %-8s %-8s %12.6f %12.6f %8.1f%%\n", design.netNames[r.first.first].c_str(),
               design.netNames[r.first.second].c_str(), raw, r.second.total(),
               raw > 0 ? 100.0 * (raw - r.second.total()) / raw : 0.0);
    }
    printf("  pairs fully blocked: %d, partly blocked: %d, total facing pairs: %zu\n", fullyBlocked, partlyBlocked,
           pairRecs.size());

    printf("\nContributing shape pairs recorded: %zu\n", pairRecs.size());
    if (writeSampex(outPath, design, tech, trees, viaTree, pairRecs)) printf("\nWrote %s\n", outPath.c_str());
    return ok ? 0 : 2;
}