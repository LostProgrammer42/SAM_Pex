// Helpers.cpp -- geometry types, tech config, DEF parser.
// Included directly by Pex_Test.cpp (unity build). Do NOT compile this file on its own.
#pragma once

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

// ---------------------------------------------------------------- constants
// Vacuum permittivity in fF/um  (8.854e-12 F/m = 8.854e-18 F/um = 8.854e-3 fF/um)
const double EPS0_FF_PER_UM = 8.854e-3;
const double GEOM_EPS = 1e-9;  // um, tolerance for geometric comparisons

// ---------------------------------------------------------------- geometry
struct Rect {
    double x1 = 0, y1 = 0, x2 = 0, y2 = 0;  // x1<=x2, y1<=y2, microns
    double w() const { return x2 - x1; }
    double h() const { return y2 - y1; }
    Rect expanded(double d) const { return Rect{x1 - d, y1 - d, x2 + d, y2 + d}; }
};

inline Rect normRect(double a, double b, double c, double d) {
    return Rect{min(a, c), min(b, d), max(a, c), max(b, d)};
}

// Closed-interval overlap (touching counts). Used by the quad tree.
inline bool overlapsClosed(const Rect& a, const Rect& b) {
    return a.x1 <= b.x2 && b.x1 <= a.x2 && a.y1 <= b.y2 && b.y1 <= a.y2;
}

struct Segment {
    int id = -1;
    int net = -1;
    int layer = -1;      // index into Tech::layers
    bool isVia = false;  // true if this shape came from a via definition
    Rect box;            // metal rectangle, microns
};

// ---------------------------------------------------------------- technology
struct Layer {
    string name;
    double zBottom = 0, thickness = 0, minSpacing = 0, defWidth = 0;  // um
};

// Same-layer sidewall coupling per um of parallel run, as a function of edge spacing, for ONE layer.
// A real captable comes from a field solver. Rows are (spacing, fF/um), spacing strictly ascending.
// The width axis of a full captable is not modelled yet (the file format reserves it, see loadCapTables).
//
// Lookup rules (at()):
//   spacing inside [first, last] row : interpolate linearly in log-log space. Coupling falls roughly as
//                                      1/spacing, which is a straight line in log-log, so this is exact
//                                      for a pure power law and a good fit for real curves.
//   spacing above the last row       : 0 (no coupling; the last row acts as a cutoff, like HALO)
//   spacing below the first row      : clamp to the first row's value, with one warning
//   a row value of 0                 : that segment falls back to linear interpolation (log of 0 is undefined)
struct CapCurve {
    string layerName;
    vector<double> s;  // spacings (um), strictly ascending
    vector<double> c;  // coupling per um of run (fF/um), >= 0
    mutable bool warnedLow = false;

    bool empty() const { return s.empty(); }

    double at(double spacing) const {
        if (spacing > s.back() * (1 + 1e-9)) return 0.0;
        if (spacing <= s.front()) {
            if (spacing < s.front() * (1 - 1e-9) && !warnedLow) {
                warnedLow = true;
                fprintf(stderr,
                        "warning: layer %s: spacing %.4f um is below the table's first row (%.4f um); "
                        "clamping to that row (shown once)\n",
                        layerName.c_str(), spacing, s.front());
            }
            return c.front();
        }
        size_t hi = (size_t)(upper_bound(s.begin(), s.end(), spacing) - s.begin());  // first row with s > spacing
        if (hi >= s.size()) return c.back();                                        // spacing == last row
        size_t lo = hi - 1;
        if (c[lo] > 0 && c[hi] > 0) {
            double u = (log(spacing) - log(s[lo])) / (log(s[hi]) - log(s[lo]));
            return exp(log(c[lo]) + u * (log(c[hi]) - log(c[lo])));
        }
        double u = (spacing - s[lo]) / (s[hi] - s[lo]);
        return c[lo] + u * (c[hi] - c[lo]);
    }
};

struct Tech {
    vector<Layer> layers;  // sorted bottom -> top by zBottom
    double epsR = 3.9;
    double fringe = 1.0;
    double halo = 1.0;
    bool useTable = false;    // COUPLING_MODEL TABLE: same-layer coupling comes from `curves`, not the formula
    string captableFile;      // CAPTABLE_FILE (path as written in tech.cfg)
    vector<CapCurve> curves;  // one per layer index; an empty curve means "no table for this layer"

    int layerIndex(const string& n) const {
        for (size_t i = 0; i < layers.size(); ++i)
            if (layers[i].name == n) return (int)i;
        return -1;
    }
    double eps() const { return EPS0_FF_PER_UM * epsR; }
    // Vertical dielectric gap between layer lo and lo+1
    double gapAbove(int lo) const {
        return layers[lo + 1].zBottom - (layers[lo].zBottom + layers[lo].thickness);
    }
};

// captable.cfg format (whitespace separated, '#' starts a comment):
//
//   CAPTABLE <layer> WIDTH ANY
//     <spacing_um>  <coupling_fF_per_um>
//     ...                                  (>= 2 rows, spacing strictly ascending)
//   END
//
// "WIDTH ANY" means the table does not depend on wire width. A numeric width (a width-indexed table) and a
// third column are reserved for later and are rejected for now, so old files cannot be misread.
inline bool loadCapTables(const string& path, Tech& t) {
    ifstream f(path);
    if (!f) {
        fprintf(stderr, "error: cannot open captable file '%s'\n", path.c_str());
        return false;
    }
    t.curves.assign(t.layers.size(), CapCurve());
    string line;
    int lineNo = 0;
    CapCurve* cur = nullptr;
    set<string> seen;
    auto fail = [&](const string& msg) {
        fprintf(stderr, "error: %s:%d: %s\n", path.c_str(), lineNo, msg.c_str());
        return false;
    };
    while (getline(f, line)) {
        ++lineNo;
        size_t h = line.find('#');
        if (h != string::npos) line.erase(h);
        istringstream ss(line);
        string kw;
        if (!(ss >> kw)) continue;
        if (kw == "CAPTABLE") {
            if (cur) return fail("CAPTABLE found before END of the previous table");
            string lname, wkw, wval;
            if (!(ss >> lname >> wkw >> wval) || wkw != "WIDTH") return fail("expected: CAPTABLE <layer> WIDTH ANY");
            if (wval != "ANY") return fail("width-indexed tables are not supported yet; use WIDTH ANY");
            int li = t.layerIndex(lname);
            if (li < 0) return fail("unknown layer '" + lname + "' (not in the tech file)");
            if (!seen.insert(lname).second) return fail("second CAPTABLE for layer '" + lname + "'");
            cur = &t.curves[li];
            cur->layerName = lname;
        } else if (kw == "END") {
            if (!cur) return fail("END without CAPTABLE");
            if (cur->s.size() < 2) return fail("table for '" + cur->layerName + "' needs at least 2 rows");
            cur = nullptr;
        } else {
            if (!cur) return fail("data row outside a CAPTABLE block");
            double sp, cv;
            istringstream rs(line);
            if (!(rs >> sp >> cv)) return fail("expected a row: <spacing_um> <coupling_fF_per_um>");
            string extra;
            if (rs >> extra) return fail("unexpected third column (width-indexed rows are not supported yet)");
            if (!(sp > 0)) return fail("spacing must be > 0");
            if (!(cv >= 0)) return fail("coupling must be >= 0");
            if (!cur->s.empty() && !(sp > cur->s.back()))
                return fail("spacing must be strictly increasing within a table");
            cur->s.push_back(sp);
            cur->c.push_back(cv);
        }
    }
    if (cur) return fail("missing END for table '" + cur->layerName + "'");
    return true;
}

// COUPLING_MODEL TABLE needs a table on every layer. Called after the tech file is read, and again if
// the model is switched on the command line.
inline bool checkCouplingModel(const Tech& t) {
    if (!t.useTable) return true;
    if (t.captableFile.empty()) {
        fprintf(stderr, "error: COUPLING_MODEL TABLE needs a CAPTABLE_FILE line in the tech file\n");
        return false;
    }
    for (size_t i = 0; i < t.layers.size(); ++i) {
        if (i >= t.curves.size() || t.curves[i].empty()) {
            fprintf(stderr, "error: COUPLING_MODEL TABLE but layer '%s' has no CAPTABLE in '%s'\n",
                    t.layers[i].name.c_str(), t.captableFile.c_str());
            return false;
        }
        if (t.curves[i].s.back() < t.halo)
            fprintf(stderr,
                    "note: layer %s table ends at %.3f um, below HALO %.3f um: pairs in between get no coupling\n",
                    t.layers[i].name.c_str(), t.curves[i].s.back(), t.halo);
    }
    return true;
}

inline bool loadTech(const string& path, Tech& t) {
    ifstream f(path);
    if (!f) {
        fprintf(stderr, "error: cannot open tech file '%s'\n", path.c_str());
        return false;
    }
    string line;
    while (getline(f, line)) {
        size_t h = line.find('#');
        if (h != string::npos) line.erase(h);
        istringstream ss(line);
        string kw;
        if (!(ss >> kw)) continue;
        if (kw == "LAYER") {
            Layer L;
            if (!(ss >> L.name >> L.zBottom >> L.thickness >> L.minSpacing >> L.defWidth)) {
                fprintf(stderr, "error: bad LAYER line: %s\n", line.c_str());
                return false;
            }
            t.layers.push_back(L);
        } else if (kw == "EPS_R") {
            ss >> t.epsR;
        } else if (kw == "FRINGE_FACTOR") {
            ss >> t.fringe;
        } else if (kw == "HALO") {
            ss >> t.halo;
        } else if (kw == "COUPLING_MODEL") {
            string m;
            ss >> m;
            if (m == "TABLE") t.useTable = true;
            else if (m == "ANALYTIC") t.useTable = false;
            else {
                fprintf(stderr, "error: COUPLING_MODEL must be ANALYTIC or TABLE, got '%s'\n", m.c_str());
                return false;
            }
        } else if (kw == "CAPTABLE_FILE") {
            ss >> t.captableFile;
        } else {
            fprintf(stderr, "warning: unknown tech keyword '%s'\n", kw.c_str());
        }
    }
    sort(t.layers.begin(), t.layers.end(),
         [](const Layer& a, const Layer& b) { return a.zBottom < b.zBottom; });
    if (t.layers.empty()) {
        fprintf(stderr, "error: no layers in tech file\n");
        return false;
    }
    t.curves.assign(t.layers.size(), CapCurve());
    if (!t.captableFile.empty()) {
        // a relative captable path is relative to the folder of the tech file
        string tablePath = t.captableFile;
        size_t slash = path.find_last_of("/\\");
        if (tablePath[0] != '/' && slash != string::npos) tablePath = path.substr(0, slash + 1) + tablePath;
        if (!loadCapTables(tablePath, t)) return false;
    }
    return checkCouplingModel(t);
}

// ---------------------------------------------------------------- design
struct Design {
    string name;
    double dbu = 1000;  // DEF database units per micron
    Rect die;           // microns
    vector<string> netNames;
    vector<char> netIsSupply;
    vector<Segment> segs;
    map<string, int> netIndex;

    int getNet(const string& n) {
        auto it = netIndex.find(n);
        if (it != netIndex.end()) return it->second;
        int id = (int)netNames.size();
        netNames.push_back(n);
        string low;
        for (char c : n) low += (char)tolower((unsigned char)c);
        netIsSupply.push_back(low == "vdd" || low == "gnd" || low == "vss" || low == "vcc");
        netIndex[n] = id;
        return id;
    }
};

// ---------------------------------------------------------------- DEF parser
// Handles the subset we need: UNITS, DIEAREA, VIAS (RECT shapes), and
// ROUTED/NEW paths in SPECIALNETS (layer width pts) and NETS (layer pts, default width).
// Segments are drawn as rectangles of exactly the stated width, flush at the ends
// (no half-width extension), which matches the abutting fragments in the sample DEF.
class DefParser {
public:
    DefParser(const Tech& t, Design& d) : tech(t), design(d) {}

    bool parseFile(const string& path) {
        ifstream f(path);
        if (!f) {
            fprintf(stderr, "error: cannot open DEF '%s'\n", path.c_str());
            return false;
        }
        stringstream buf;
        buf << f.rdbuf();
        tk = tokenize(buf.str());
        p = 0;
        try {
            run();
        } catch (const exception& e) {
            fprintf(stderr, "error: DEF parse failed near token %zu ('%s'): %s\n", p,
                    p < tk.size() ? tk[p].c_str() : "<eof>", e.what());
            return false;
        }
        return true;
    }

private:
    const Tech& tech;
    Design& design;
    vector<string> tk;
    size_t p = 0;

    struct ViaShape {
        string layer;
        Rect r;  // DBU, relative to via origin
    };
    map<string, vector<ViaShape>> vias;
    set<string> warned;

    static vector<string> tokenize(const string& s) {
        vector<string> t;
        string cur;
        auto flush = [&]() {
            if (!cur.empty()) {
                t.push_back(cur);
                cur.clear();
            }
        };
        for (size_t i = 0; i < s.size(); ++i) {
            char c = s[i];
            if (c == '#') {
                flush();
                while (i < s.size() && s[i] != '\n') ++i;
            } else if (isspace((unsigned char)c)) {
                flush();
            } else if (c == '(' || c == ')' || c == ';') {
                flush();
                t.push_back(string(1, c));
            } else if (c == '"') {
                flush();
                size_t j = i + 1;
                string q;
                while (j < s.size() && s[j] != '"') q += s[j++];
                t.push_back(q);
                i = j;
            } else {
                cur += c;
            }
        }
        flush();
        return t;
    }

    void need(size_t n) {
        if (p + n > tk.size()) throw runtime_error("unexpected end of file");
    }

    void warnOnce(const string& key, const string& msg) {
        if (warned.insert(key).second) fprintf(stderr, "warning: %s\n", msg.c_str());
    }

    // reads "( a b )" with plain numbers (no '*'), used for RECT/DIEAREA
    void readPair(double& a, double& b) {
        need(4);
        if (tk[p] != "(") throw runtime_error("expected '('");
        a = stod(tk[p + 1]);
        b = stod(tk[p + 2]);
        p += 3;
        while (p < tk.size() && tk[p] != ")") ++p;
        ++p;
    }

    void run() {
        while (p < tk.size()) {
            const string& t = tk[p];
            if (t == "END" && p + 1 < tk.size() && tk[p + 1] == "DESIGN") {
                break;  // END DESIGN
            } else if (t == "DESIGN") {
                need(2);
                design.name = tk[p + 1];
                p += 2;
            } else if (t == "UNITS") {
                // UNITS DISTANCE MICRONS n ;
                need(4);
                design.dbu = stod(tk[p + 3]);
                p += 4;
            } else if (t == "DIEAREA") {
                ++p;
                double a, b, c, d;
                readPair(a, b);
                readPair(c, d);
                design.die = normRect(a / design.dbu, b / design.dbu, c / design.dbu, d / design.dbu);
            } else if (t == "VIAS") {
                parseVias();
            } else if (t == "SPECIALNETS") {
                parseNets(true);
            } else if (t == "NETS") {
                parseNets(false);
            } else {
                ++p;
            }
        }
    }

    void parseVias() {
        p += 3;  // VIAS n ;
        while (p < tk.size() && tk[p] != "END") {
            if (tk[p] == "-") {
                need(2);
                string name = tk[p + 1];
                p += 2;
                vector<ViaShape> shapes;
                while (p < tk.size() && tk[p] != ";") {
                    if (tk[p] == "RECT") {
                        need(2);
                        string layer = tk[p + 1];
                        p += 2;
                        double a, b, c, d;
                        readPair(a, b);
                        readPair(c, d);
                        shapes.push_back({layer, normRect(a, b, c, d)});
                    } else {
                        ++p;
                    }
                }
                ++p;  // ';'
                vias[name] = shapes;
            } else {
                ++p;
            }
        }
        p += 2;  // END VIAS
    }

    void parseNets(bool special) {
        p += 3;  // SECTION n ;
        while (p < tk.size() && tk[p] != "END") {
            if (tk[p] == "-") {
                need(2);
                int net = design.getNet(tk[p + 1]);
                p += 2;
                while (p < tk.size() && tk[p] != ";") {
                    const string& t = tk[p];
                    if (t == "ROUTED" || t == "NEW" || t == "FIXED" || t == "COVER" || t == "SHIELD") {
                        ++p;
                        parsePath(net, special);
                    } else {
                        ++p;
                    }
                }
                ++p;  // ';'
            } else {
                ++p;
            }
        }
        p += 2;  // END <SECTION>
    }

    void parsePath(int net, bool special) {
        need(1);
        string layer = tk[p++];
        double widthDbu = -1;
        if (special) {
            need(1);
            widthDbu = stod(tk[p++]);
        }
        int L = tech.layerIndex(layer);
        bool havePrev = false;
        double px = 0, py = 0;
        while (p < tk.size()) {
            const string& t = tk[p];
            if (t == ";" || t == "NEW" || t == "+") break;
            if (t == "(") {
                need(4);
                double x = (tk[p + 1] == "*") ? px : stod(tk[p + 1]);
                double y = (tk[p + 2] == "*") ? py : stod(tk[p + 2]);
                p += 3;
                while (p < tk.size() && tk[p] != ")") ++p;  // skip optional extension value
                ++p;
                if (havePrev) {
                    if (L >= 0) addWire(net, L, widthDbu, px, py, x, y);
                    else warnOnce("layer:" + layer, "layer '" + layer + "' not in tech config; its wires are skipped");
                }
                px = x;
                py = y;
                havePrev = true;
            } else {
                // via name after a point
                string v = t;
                ++p;
                placeVia(net, v, px, py);
                // if the path continues, subsequent wires are on the via's other routing layer
                auto it = vias.find(v);
                if (it != vias.end()) {
                    for (auto& s : it->second) {
                        int li = tech.layerIndex(s.layer);
                        if (li >= 0 && li != L) {
                            L = li;
                            layer = s.layer;
                            break;
                        }
                    }
                }
            }
        }
    }

    void addWire(int net, int L, double widthDbu, double x1, double y1, double x2, double y2) {
        if (widthDbu < 0) widthDbu = tech.layers[L].defWidth * design.dbu;
        double half = widthDbu / 2.0;
        Rect r;
        if (fabs(y1 - y2) < 0.5 && fabs(x1 - x2) >= 0.5) {  // horizontal
            r = Rect{min(x1, x2), y1 - half, max(x1, x2), y1 + half};
        } else if (fabs(x1 - x2) < 0.5 && fabs(y1 - y2) >= 0.5) {  // vertical
            r = Rect{x1 - half, min(y1, y2), x1 + half, max(y1, y2)};
        } else if (fabs(x1 - x2) < 0.5 && fabs(y1 - y2) < 0.5) {
            return;  // zero-length wire, nothing to add
        } else {
            warnOnce("nonmanhattan", "non-Manhattan wire skipped");
            return;
        }
        r = Rect{r.x1 / design.dbu, r.y1 / design.dbu, r.x2 / design.dbu, r.y2 / design.dbu};
        pushSeg(net, L, false, r);
    }

    void placeVia(int net, const string& name, double x, double y) {
        auto it = vias.find(name);
        if (it == vias.end()) {
            warnOnce("via:" + name, "via '" + name + "' not defined in DEF VIAS; skipped");
            return;
        }
        for (auto& s : it->second) {
            int L = tech.layerIndex(s.layer);
            if (L < 0) continue;  // e.g. the 'mcon' cut layer: not a conductor for coupling
            Rect r{(x + s.r.x1) / design.dbu, (y + s.r.y1) / design.dbu,
                   (x + s.r.x2) / design.dbu, (y + s.r.y2) / design.dbu};
            pushSeg(net, L, true, r);
        }
    }

    void pushSeg(int net, int L, bool isVia, const Rect& r) {
        Segment s;
        s.id = (int)design.segs.size();
        s.net = net;
        s.layer = L;
        s.isVia = isVia;
        s.box = r;
        design.segs.push_back(s);
    }
};