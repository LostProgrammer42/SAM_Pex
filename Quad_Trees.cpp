// Quad_Trees.cpp -- region quadtree over segment bounding boxes.
// Included directly by Pex_Test.cpp (unity build). Do NOT compile this file on its own.
#pragma once

#include "Helpers.cpp"

// One tree indexes the segments of ONE layer. The tree stores only segment IDs; geometry
// is looked up in the shared segment vector. A segment is stored in every leaf its box
// overlaps (so long wires are duplicated); queries de-duplicate with a visit stamp.
class QuadTree {
public:
    struct Node {
        Rect b;
        int depth = 0;
        int child[4] = {-1, -1, -1, -1};  // 0=SW 1=SE 2=NW 3=NE
        vector<int> ids;
        bool leaf() const { return child[0] < 0; }
    };

    struct Stats {
        int nodes = 0, leaves = 0, maxDepth = 0, maxPerLeaf = 0;
        long entries = 0;       // total ids stored across leaves
        double avgPerLeaf = 0;  // entries / leaves
        double dupFactor = 0;   // entries / distinct segments inserted
    };

    QuadTree(const Rect& bounds, const vector<Segment>* segs, int capacity = 8, int maxDepth = 10)
        : segs(segs), cap(capacity), maxDepth(maxDepth) {
        Node root;
        root.b = bounds;
        nodes.push_back(root);
        stamp.assign(segs->size(), 0);
    }

    void insert(int id) {
        ++inserted;
        insertAt(0, id);
    }

    // Calls cb(id) once for every stored segment whose box overlaps r (closed overlap).
    template <class F>
    void query(const Rect& r, F&& cb) {
        ++curStamp;
        queryAt(0, r, cb);
    }

    Stats stats() const {
        Stats s;
        s.nodes = (int)nodes.size();
        for (const Node& n : nodes) {
            s.maxDepth = max(s.maxDepth, n.depth);
            if (n.leaf()) {
                ++s.leaves;
                s.entries += (long)n.ids.size();
                s.maxPerLeaf = max(s.maxPerLeaf, (int)n.ids.size());
            }
        }
        s.avgPerLeaf = s.leaves ? (double)s.entries / s.leaves : 0;
        s.dupFactor = inserted ? (double)s.entries / inserted : 0;
        return s;
    }

    // Writes a JSON array of nodes: [{"llx":..,"lly":..,"wx":..,"wy":..,"depth":..,"leaf":..,"n":..}, ...]
    void writeNodesJson(FILE* f) const {
        fprintf(f, "[");
        for (size_t i = 0; i < nodes.size(); ++i) {
            const Node& n = nodes[i];
            fprintf(f, "%s{\"llx\":%.6f,\"lly\":%.6f,\"wx\":%.6f,\"wy\":%.6f,\"depth\":%d,\"leaf\":%s,\"n\":%zu}",
                    i ? "," : "", n.b.x1, n.b.y1, n.b.w(), n.b.h(), n.depth, n.leaf() ? "true" : "false",
                    n.ids.size());
        }
        fprintf(f, "]");
    }

private:
    const vector<Segment>* segs;
    int cap, maxDepth;
    vector<Node> nodes;
    vector<unsigned> stamp;
    unsigned curStamp = 0;
    long inserted = 0;

    // NOTE: nodes may reallocate during insertion, so never hold a Node& across a recursive call.
    void insertAt(int n, int id) {
        const Rect& box = (*segs)[id].box;
        if (nodes[n].leaf()) {
            nodes[n].ids.push_back(id);
            if ((int)nodes[n].ids.size() > cap && nodes[n].depth < maxDepth) split(n);
            return;
        }
        for (int k = 0; k < 4; ++k) {
            int c = nodes[n].child[k];
            if (overlapsClosed(nodes[c].b, box)) insertAt(c, id);
        }
    }

    void split(int n) {
        Rect b = nodes[n].b;
        double mx = (b.x1 + b.x2) / 2, my = (b.y1 + b.y2) / 2;
        int d = nodes[n].depth + 1;
        Rect q[4] = {Rect{b.x1, b.y1, mx, my}, Rect{mx, b.y1, b.x2, my},
                     Rect{b.x1, my, mx, b.y2}, Rect{mx, my, b.x2, b.y2}};
        vector<int> ids;
        ids.swap(nodes[n].ids);
        for (int k = 0; k < 4; ++k) {
            Node c;
            c.b = q[k];
            c.depth = d;
            nodes[n].child[k] = (int)nodes.size();
            nodes.push_back(c);
        }
        for (int id : ids) {
            for (int k = 0; k < 4; ++k) {
                int c = nodes[n].child[k];
                if (overlapsClosed(nodes[c].b, (*segs)[id].box)) insertAt(c, id);
            }
        }
    }

    template <class F>
    void queryAt(int n, const Rect& r, F& cb) {
        const Node& nd = nodes[n];  // safe: no insertion happens during a query
        if (!overlapsClosed(nd.b, r)) return;
        if (nd.leaf()) {
            for (int id : nd.ids) {
                if (stamp[id] == curStamp) continue;
                stamp[id] = curStamp;
                if (overlapsClosed((*segs)[id].box, r)) cb(id);
            }
            return;
        }
        for (int k = 0; k < 4; ++k) queryAt(nd.child[k], r, cb);
    }
};