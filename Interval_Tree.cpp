// Author: Stavan Mehta, Affaan Fakih
// Version: Pilot Initial (V0)
//
// IntervalTree: sweep-line active-segment container for PEX (parasitic extraction).
//
// Stage 1 scope (ground-only capacitance, no coupling):
//   - insert(seg)   -> Handle   : add a segment when its x_lo event fires
//   - erase(handle)             : remove a segment when its x_hi event fires
//   - activeSet()   -> vector   : dump everything currently active, sorted by y_lo
//
// structure in the test file does not need to change.

#include <bits/stdc++.h>
using namespace std;

using Coord = long;   // matches CornerStitch's llx/lly type

enum LayerType {
    L_NONE = 0,
    L_NDIFF,
    L_PDIFF,
    L_NTRANS,
    L_PTRANS,
    L_POLY,
    L_LI,
    L_M1,
    L_M2,
    L_POLYCON,
    L_LIBRDRCON,
    L_MCON,
    L_NDC,
    L_PDC
};


struct SegmentInterval {
    Coord y_lo = 0, y_hi = 0;      // the interval this segment occupies in y (used as the tree key)
    Coord x_lo = 0, x_hi = 0;      // segment's x-extent (drives when it's active during the sweep)
    unsigned int layer = L_NONE;   // reuse LayerType attr values from Corner_Stitch.cpp
    unsigned int net_id = 0;       // matches CornerStitch::getNet() / getNetId() convention
    uint32_t segment_id = 0;       // unique id assigned by the driver, for exporter/visualizer/result attribution

    Coord width() const { return y_hi - y_lo; }
};

class IntervalTree {
public:
    // multimap keyed on y_lo lets multiple segments share the same y_lo,
    // gives O(log n) insert/erase, and an in-order walk sorted by y for free.
    using Container = multimap<Coord, SegmentInterval>;
    using Handle = Container::iterator;

    // Insert a segment into the active set. O(log n).
    // Returns a handle the caller must keep to erase this exact segment later.
    Handle insert(const SegmentInterval& seg) {
        return tree_.insert({seg.y_lo, seg});
    }

    // Remove a segment from the active set using its handle. O(1) amortized.
    // Caller is responsible for not reusing the handle afterward.
    void erase(Handle h) {
        tree_.erase(h);
    }

    // Dump everything currently active, in ascending y_lo order.
    // Used for: (a) ground-cap area/perimeter accumulation at a sweep step,
    //           (b) feeding the visualizer a snapshot of "what's active here".
    vector<SegmentInterval> activeSet() const {
        vector<SegmentInterval> result;
        result.reserve(tree_.size());
        for (const auto& [y, seg] : tree_) {
            result.push_back(seg);
        }
        return result;
    }

    size_t size() const { return tree_.size(); }
    bool empty() const { return tree_.empty(); }
    void clear() { tree_.clear(); }

private:
    Container tree_;
};