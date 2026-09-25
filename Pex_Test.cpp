// Author: Stavan Mehta, Affaan Fakih
// Version: Pilot Initial (V0)
//
// PexTest: standalone driver for stage-1 PEX (ground-only capacitance, no coupling).
//
// Pipeline:
//   1. Parse a routed .rect file into rectangles (same grammar as the router's .rect,
//      but parsed independently here -- no dependency on Corner_Stitch.cpp / Modular_Route.cpp).
//   2. Build one SegmentInterval per rectangle.
//   3. Run an x-sweep: INSERT at x_lo, DELETE at x_hi, using IntervalTree as the active set.
//   4. At each event, snapshot the active set (for the visualizer).
//   5. Compute flat per-segment resistance and ground capacitance (no tree query needed
//      for stage 1 -- ground is a uniform plane, so this is just per-segment geometry math).
//   6. Export everything to a .sampex file for the Python visualizer.
//
// This file is standalone: it does not #include any of the router's files.

#include <bits/stdc++.h>
#include "Interval_Tree.cpp"
using namespace std;

// ---------------------------------------------------------------------------
// Physical constants (placeholders -- swap in real process numbers later)
// ---------------------------------------------------------------------------
static const double RHO_PER_LAYER_DEFAULT = 0.05;   // ohms per square (sheet resistance placeholder)
static const double THICKNESS_DEFAULT     = 1.0;    // placeholder metal thickness (same units as x/y)
static const double C_GROUND_PER_AREA     = 0.0001; // placeholder ground-cap per unit area (fF/unit^2 style)

// Per-layer sheet resistance placeholder table. Keyed by the same LayerType
// values used in SegmentInterval::layer (see IntervalTree.cpp's local LayerType copy).
double sheetResistance(unsigned int layer) {
    switch (layer) {
        case L_POLY: return 0.08;
        case L_LI:   return 0.06;
        case L_M1:   return 0.05;
        case L_M2:   return 0.04;
        default:     return RHO_PER_LAYER_DEFAULT;
    }
}

string layerName(unsigned int layer) {
    switch (layer) {
        case L_NONE:      return "none";
        case L_NDIFF:     return "ndiff";
        case L_PDIFF:     return "pdiff";
        case L_NTRANS:    return "ntransistor";
        case L_PTRANS:    return "ptransistor";
        case L_POLY:      return "polysilicon";
        case L_LI:        return "li";
        case L_M1:        return "m1";
        case L_M2:        return "m2";
        case L_POLYCON:   return "polycon";
        case L_LIBRDRCON: return "librdrcon";
        case L_MCON:      return "mcon";
        case L_NDC:       return "ndc";
        case L_PDC:       return "pdc";
        default:          return "none";
    }
}

unsigned int layerFromName(const string& name) {
    if (name == "ndiff" || name == "ndiffusion")       return L_NDIFF;
    if (name == "pdiff" || name == "pdiffusion")       return L_PDIFF;
    if (name == "ntransistor")                          return L_NTRANS;
    if (name == "ptransistor")                          return L_PTRANS;
    if (name == "polysilicon")                          return L_POLY;
    if (name == "li")                                   return L_LI;
    if (name == "m1")                                   return L_M1;
    if (name == "m2")                                   return L_M2;
    if (name == "polycon")                              return L_POLYCON;
    if (name == "librdrcon")                            return L_LIBRDRCON;
    if (name == "mcon")                                 return L_MCON;
    return L_NONE;
}

// ---------------------------------------------------------------------------
// .rect parsing (standalone copy -- grammar matches Routing_Helpers.cpp's
// parseRectLine, but re-implemented here with no #include of that file).
// Grammar per line: <rect|inrect|outrect> <net> <layer> <x1> <y1> <x2> <y2>
// Lines starting with '#' or the leading "bbox ..." header line are skipped.
// ---------------------------------------------------------------------------
struct RectRaw {
    string inout;
    string net;
    string layer;
    Coord x1, y1, x2, y2;
};

bool parseRectLine(const string& line, RectRaw& out) {
    if (line.empty() || line[0] == '#') return false;

    stringstream ss(line);
    string kind;
    ss >> kind;

    if (kind == "bbox") return false; // header line from exportRect, not a shape
    if (kind != "rect" && kind != "inrect" && kind != "outrect") return false;

    out.inout = kind;
    ss >> out.net >> out.layer >> out.x1 >> out.y1 >> out.x2 >> out.y2;
    if (!ss) return false;
    if (out.x2 <= out.x1 || out.y2 <= out.y1) return false;

    return true;
}

vector<RectRaw> loadRectFile(const string& filename) {
    vector<RectRaw> rects;
    ifstream fin(filename);
    if (!fin) {
        cerr << "Error: cannot open " << filename << "\n";
        return rects;
    }
    string line;
    while (getline(fin, line)) {
        RectRaw r;
        if (parseRectLine(line, r)) rects.push_back(r);
    }
    return rects;
}

// Local net-name -> id mapping (standalone copy of the router's getNetId pattern)
unordered_map<string, unsigned int> netMap;
vector<string> netList = {"#"};
unsigned int nextNetId = 1;

unsigned int getNetId(const string& net) {
    if (net == "#") return 0;
    auto it = netMap.find(net);
    if (it != netMap.end()) return it->second;
    netMap[net] = nextNetId;
    netList.push_back(net);
    return nextNetId++;
}

// ---------------------------------------------------------------------------
// Sweep event structure
// ---------------------------------------------------------------------------
enum class EventType { INSERT, DELETE };

struct SweepEvent {
    Coord x;
    EventType type;
    uint32_t seg_id;
};

// Per-segment computed results, filled in after the sweep
struct SegmentResult {
    uint32_t id;
    unsigned int net_id;
    unsigned int layer;
    Coord x_lo, x_hi, y_lo, y_hi;
    double resistance;
    double cap_ground;
};

// Per-net aggregated results, computed from all segments sharing a net_id.
//
// cap_ground_total is exact for stage 1: ground capacitors from independent
// segments simply add in parallel, regardless of how the segments connect.
//
// resistance_total is NOT a true net resistance -- it is the flat sum of every
// segment's resistance on this net, as if they were all in series. We have no
// connectivity/adjacency information in this file (no corner-stitch, no via
// graph), so a topologically-correct series/parallel reduction isn't possible
// here. Treat resistance_total as a rough upper-bound/sanity number only.
struct NetResult {
    unsigned int net_id;
    string net_name;
    int segment_count;
    double resistance_total;   // naive sum -- see note above, NOT series-correct
    double cap_ground_total;   // exact -- ground caps add in parallel regardless of topology
};

// One recorded step of the sweep, for the visualizer
struct SweepStep {
    Coord x;
    EventType type;
    uint32_t seg_id;
    vector<uint32_t> active_ids;
};

// ---------------------------------------------------------------------------
// Flat per-segment R/C math (stage 1: no tree query needed, ground is a
// uniform plane so this is pure per-segment geometry -- see design discussion).
// ---------------------------------------------------------------------------
double computeResistance(const SegmentInterval& seg) {
    double length = static_cast<double>(seg.x_hi - seg.x_lo);
    double width  = static_cast<double>(seg.y_hi - seg.y_lo);
    if (width <= 0.0) return 0.0;
    double rho = sheetResistance(seg.layer);
    return rho * length / width;
}

double computeGroundCap(const SegmentInterval& seg) {
    double length = static_cast<double>(seg.x_hi - seg.x_lo);
    double width  = static_cast<double>(seg.y_hi - seg.y_lo);
    double area = length * width;
    return area * C_GROUND_PER_AREA;
}

// ---------------------------------------------------------------------------
// Per-net aggregation (see NetResult comment above for the resistance caveat)
// ---------------------------------------------------------------------------
vector<NetResult> aggregateByNet(const vector<SegmentResult>& results, const vector<string>& netList) {
    map<unsigned int, NetResult> byNet; // map (not unordered_map) so output is net_id-sorted

    for (const auto& r : results) {
        auto it = byNet.find(r.net_id);
        if (it == byNet.end()) {
            NetResult n;
            n.net_id = r.net_id;
            n.net_name = (r.net_id < netList.size()) ? netList[r.net_id] : "?";
            n.segment_count = 0;
            n.resistance_total = 0.0;
            n.cap_ground_total = 0.0;
            it = byNet.emplace(r.net_id, n).first;
        }
        it->second.segment_count += 1;
        it->second.resistance_total += r.resistance;
        it->second.cap_ground_total += r.cap_ground;
    }

    vector<NetResult> out;
    out.reserve(byNet.size());
    for (auto& [id, n] : byNet) out.push_back(n);
    return out;
}

void printNetSummary(const vector<NetResult>& nets) {
    cout << "\n--- Per-net PEX summary (stage 1: ground cap only, no coupling) ---\n";
    cout << "NOTE: resistance_total is a naive sum of all segments on the net,\n";
    cout << "      not a topologically-correct series/parallel value (no\n";
    cout << "      connectivity info available at this stage).\n\n";
    cout << left << setw(10) << "net"
         << right << setw(8) << "segs"
         << setw(16) << "R_total (ohm)"
         << setw(18) << "C_ground_total"
         << "\n";
    for (const auto& n : nets) {
        cout << left << setw(10) << n.net_name
             << right << setw(8) << n.segment_count
             << setw(16) << n.resistance_total
             << setw(18) << n.cap_ground_total
             << "\n";
    }
    cout << "\n";
}

// ---------------------------------------------------------------------------
// .sampex export
// ---------------------------------------------------------------------------
void exportSampex(
    const string& filename,
    const vector<SegmentResult>& results,
    const vector<SweepStep>& sweep,
    const vector<NetResult>& nets
) {
    ofstream fout(filename);
    if (!fout) {
        cerr << "Error: cannot open " << filename << " for writing\n";
        return;
    }

    fout << "{\n";

    // ---- segments ----
    fout << "  \"segments\": [\n";
    for (size_t i = 0; i < results.size(); i++) {
        const auto& r = results[i];
        fout << "    {"
             << "\"id\":" << r.id << ","
             << "\"net\":" << r.net_id << ","
             << "\"layer\":\"" << layerName(r.layer) << "\","
             << "\"x_lo\":" << r.x_lo << ","
             << "\"x_hi\":" << r.x_hi << ","
             << "\"y_lo\":" << r.y_lo << ","
             << "\"y_hi\":" << r.y_hi << ","
             << "\"resistance\":" << r.resistance << ","
             << "\"cap_ground\":" << r.cap_ground
             << "}";
        if (i + 1 < results.size()) fout << ",";
        fout << "\n";
    }
    fout << "  ],\n";

    // ---- nets ----
    fout << "  \"nets\": [\n";
    for (size_t i = 0; i < nets.size(); i++) {
        const auto& n = nets[i];
        fout << "    {"
             << "\"net_id\":" << n.net_id << ","
             << "\"net_name\":\"" << n.net_name << "\","
             << "\"segment_count\":" << n.segment_count << ","
             << "\"resistance_total\":" << n.resistance_total << ","
             << "\"cap_ground_total\":" << n.cap_ground_total
             << "}";
        if (i + 1 < nets.size()) fout << ",";
        fout << "\n";
    }
    fout << "  ],\n";

    // ---- sweep ----
    fout << "  \"sweep\": [\n";
    for (size_t i = 0; i < sweep.size(); i++) {
        const auto& s = sweep[i];
        fout << "    {"
             << "\"x\":" << s.x << ","
             << "\"event\":\"" << (s.type == EventType::INSERT ? "insert" : "delete") << "\","
             << "\"seg_id\":" << s.seg_id << ","
             << "\"active\":[";
        for (size_t j = 0; j < s.active_ids.size(); j++) {
            fout << s.active_ids[j];
            if (j + 1 < s.active_ids.size()) fout << ",";
        }
        fout << "]}";
        if (i + 1 < sweep.size()) fout << ",";
        fout << "\n";
    }
    fout << "  ]\n";

    fout << "}\n";
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    if (argc < 2) {
        cerr << "Usage: ./PexTest <file.rect> [output.sampex]\n";
        return 1;
    }

    string inputFile = argv[1];
    string outputFile = (argc >= 3) ? argv[2] : "output.sampex";

    vector<RectRaw> rawRects = loadRectFile(inputFile);
    if (rawRects.empty()) {
        cerr << "Warning: no valid rectangles parsed from " << inputFile << "\n";
    }

    // Build SegmentIntervals, one per rectangle, with a unique segment_id
    vector<SegmentInterval> segments;
    segments.reserve(rawRects.size());
    for (size_t i = 0; i < rawRects.size(); i++) {
        const auto& r = rawRects[i];
        SegmentInterval seg;
        seg.x_lo = r.x1;
        seg.x_hi = r.x2;
        seg.y_lo = r.y1;
        seg.y_hi = r.y2;
        seg.layer = layerFromName(r.layer);
        seg.net_id = getNetId(r.net);
        seg.segment_id = static_cast<uint32_t>(i);
        segments.push_back(seg);
    }

    // Build sweep events: INSERT at x_lo, DELETE at x_hi
    vector<SweepEvent> events;
    events.reserve(segments.size() * 2);
    for (const auto& seg : segments) {
        events.push_back({seg.x_lo, EventType::INSERT, seg.segment_id});
        events.push_back({seg.x_hi, EventType::DELETE, seg.segment_id});
    }

    // Sort by x; at equal x, INSERT before DELETE so a segment ending exactly
    // where another begins doesn't cause a false momentary gap in the active set.
    sort(events.begin(), events.end(), [](const SweepEvent& a, const SweepEvent& b) {
        if (a.x != b.x) return a.x < b.x;
        if (a.type != b.type) return a.type == EventType::INSERT;
        return a.seg_id < b.seg_id;
    });

    // Run the sweep
    IntervalTree tree;
    unordered_map<uint32_t, IntervalTree::Handle> handles;
    vector<SweepStep> sweepLog;
    sweepLog.reserve(events.size());

    for (const auto& ev : events) {
        if (ev.type == EventType::INSERT) {
            const SegmentInterval& seg = segments[ev.seg_id];
            handles[ev.seg_id] = tree.insert(seg);
        } else {
            auto it = handles.find(ev.seg_id);
            if (it != handles.end()) {
                tree.erase(it->second);
                handles.erase(it);
            }
        }

        SweepStep step;
        step.x = ev.x;
        step.type = ev.type;
        step.seg_id = ev.seg_id;
        for (const auto& active : tree.activeSet()) {
            step.active_ids.push_back(active.segment_id);
        }
        sweepLog.push_back(step);
    }

    // Compute flat per-segment R and ground-C (stage 1 -- no tree query needed)
    vector<SegmentResult> results;
    results.reserve(segments.size());
    for (const auto& seg : segments) {
        SegmentResult r;
        r.id = seg.segment_id;
        r.net_id = seg.net_id;
        r.layer = seg.layer;
        r.x_lo = seg.x_lo;
        r.x_hi = seg.x_hi;
        r.y_lo = seg.y_lo;
        r.y_hi = seg.y_hi;
        r.resistance = computeResistance(seg);
        r.cap_ground = computeGroundCap(seg);
        results.push_back(r);
    }

    // Aggregate per-net R/C (see NetResult comment for the resistance caveat)
    vector<NetResult> netResults = aggregateByNet(results, netList);

    exportSampex(outputFile, results, sweepLog, netResults);

    cout << "Parsed " << segments.size() << " segments from " << inputFile << "\n";
    cout << "Ran " << events.size() << " sweep events\n";
    cout << "Wrote " << outputFile << "\n";

    printNetSummary(netResults);

    return 0;
}

// ToDo: Add VIA Resistance cost as well