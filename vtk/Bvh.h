#pragma once
#include "ObjLoader.h"
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>

// ─────────────────────────────────────────
//  AABB
// ─────────────────────────────────────────
struct AABB {
    Vec3 mn{ 1e30f,  1e30f,  1e30f };
    Vec3 mx{ -1e30f, -1e30f, -1e30f };

    void expand(const Vec3& p) {
        for (int i = 0; i < 3; ++i) { mn[i] = std::min(mn[i], p[i]); mx[i] = std::max(mx[i], p[i]); }
    }
    void expand(const AABB& b) { expand(b.mn); expand(b.mx); }

    // Minimum squared distance from point p to this AABB
    float minDistSq(const Vec3& p) const {
        float d = 0.f;
        for (int i = 0; i < 3; ++i) {
            float v = p[i];
            if (v < mn[i]) d += (mn[i] - v) * (mn[i] - v);
            else if (v > mx[i]) d += (v - mx[i]) * (v - mx[i]);
        }
        return d;
    }

    bool intersectRay(const Vec3& o, const Vec3& invDir, float& tmin, float& tmax) const {
        tmin = -1e30f; tmax = 1e30f;
        for (int i = 0; i < 3; ++i) {
            if (std::isinf(invDir[i])) {
                if (o[i] < mn[i] || o[i] > mx[i]) return false;
            }
            else {
                float t1 = (mn[i] - o[i]) * invDir[i], t2 = (mx[i] - o[i]) * invDir[i];
                if (t1 > t2) std::swap(t1, t2);
                tmin = std::max(tmin, t1); tmax = std::min(tmax, t2);
                if (tmin > tmax) return false;
            }
        }
        return tmax >= 0.f;
    }
};

inline AABB triAABB(const Triangle& t) {
    AABB b; b.expand(t.v0); b.expand(t.v1); b.expand(t.v2); return b;
}
inline Vec3 triCentroid(const Triangle& t) { return (t.v0 + t.v1 + t.v2) * (1.f / 3.f); }

// ─────────────────────────────────────────
//  Closest point on triangle to point p
// ─────────────────────────────────────────
inline Vec3 closestPointOnTriangle(const Vec3& p, const Triangle& tri) {
    Vec3 ab = tri.v1 - tri.v0;
    Vec3 ac = tri.v2 - tri.v0;
    Vec3 ap = p - tri.v0;

    float d1 = ab.dot(ap), d2 = ac.dot(ap);
    if (d1 <= 0.f && d2 <= 0.f) return tri.v0;

    Vec3  bp = p - tri.v1;
    float d3 = ab.dot(bp), d4 = ac.dot(bp);
    if (d3 >= 0.f && d4 <= d3) return tri.v1;

    Vec3  cp = p - tri.v2;
    float d5 = ab.dot(cp), d6 = ac.dot(cp);
    if (d6 >= 0.f && d5 <= d6) return tri.v2;

    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f) {
        float v = d1 / (d1 - d3);
        return tri.v0 + ab * v;
    }
    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f) {
        float w = d2 / (d2 - d6);
        return tri.v0 + ac * w;
    }
    float va = d3 * d6 - d5 * d4;
    if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f) {
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return tri.v1 + (tri.v2 - tri.v1) * w;
    }
    float denom = 1.f / (va + vb + vc);
    float v = vb * denom, w = vc * denom;
    return tri.v0 + ab * v + ac * w;
}

inline float distSqToTriangle(const Vec3& p, const Triangle& tri) {
    Vec3 cp = closestPointOnTriangle(p, tri);
    Vec3 d = p - cp;
    return d.dot(d);
}

// ─────────────────────────────────────────
//  BVH Node
// ─────────────────────────────────────────
struct BVHNode {
    AABB bounds;
    int  left = -1, right = -1;
    int  triStart = 0, triCount = 0;
};

// ─────────────────────────────────────────
//  BVH
// ─────────────────────────────────────────
class BVH {
public:
    std::vector<Triangle> tris;
    std::vector<BVHNode>  nodes;
    static constexpr int LEAF = 4;

    void build(const std::vector<Triangle>& triangles) {
        tris = triangles;
        nodes.clear();
        nodes.reserve(2 * (int)tris.size());
        buildNode(0, (int)tris.size());
    }

    // ── Nearest unsigned distance² to mesh ──────────────────
    float nearestDistSq(const Vec3& p) const {
        float best = 1e30f;
        queryNearest(0, p, best);
        return best;
    }

    // ── Parity ray test: count intersections in +X direction ─
    void collectHits(const Vec3& orig, const Vec3& dir,
        float tMin, std::vector<float>& hits) const {
        Vec3 invDir = {
            dir.x == 0.f ? 1e30f : 1.f / dir.x,
            dir.y == 0.f ? 1e30f : 1.f / dir.y,
            dir.z == 0.f ? 1e30f : 1.f / dir.z
        };
        hits.clear();
        collectNode(0, orig, dir, invDir, tMin, hits);
    }

private:
    int buildNode(int start, int end) {
        BVHNode node;
        node.triStart = start; node.triCount = end - start;
        for (int i = start; i < end; ++i) {
            node.bounds.expand(tris[i].v0);
            node.bounds.expand(tris[i].v1);
            node.bounds.expand(tris[i].v2);
        }
        int idx = (int)nodes.size();
        nodes.push_back(node);
        if (end - start <= LEAF) return idx;

        Vec3 ext = node.bounds.mx - node.bounds.mn;
        int axis = 0;
        if (ext[1] > ext[0])axis = 1;
        if (ext[2] > ext[axis])axis = 2;
        int mid = (start + end) / 2;
        std::nth_element(tris.begin() + start, tris.begin() + mid, tris.begin() + end,
            [axis](const Triangle& a, const Triangle& b) {
                return triCentroid(a)[axis] < triCentroid(b)[axis];
            });
        int l = buildNode(start, mid);
        int r = buildNode(mid, end);
        nodes[idx].left = l; nodes[idx].right = r; nodes[idx].triCount = 0;
        return idx;
    }

    void queryNearest(int idx, const Vec3& p, float& bestSq) const {
        const BVHNode& n = nodes[idx];
        if (n.bounds.minDistSq(p) >= bestSq) return;  // prune

        if (n.left == -1) {
            for (int i = n.triStart; i < n.triStart + n.triCount; ++i) {
                float d = distSqToTriangle(p, tris[i]);
                if (d < bestSq) bestSq = d;
            }
            return;
        }
        // Visit closer child first
        float dl = nodes[n.left].bounds.minDistSq(p);
        float dr = nodes[n.right].bounds.minDistSq(p);
        if (dl < dr) { queryNearest(n.left, p, bestSq); queryNearest(n.right, p, bestSq); }
        else { queryNearest(n.right, p, bestSq); queryNearest(n.left, p, bestSq); }
    }

    void collectNode(int idx, const Vec3& o, const Vec3& d, const Vec3& inv,
        float tMin, std::vector<float>& hits) const {
        const BVHNode& n = nodes[idx];
        float t0, t1;
        if (!n.bounds.intersectRay(o, inv, t0, t1)) return;
        if (n.left == -1) {
            for (int i = n.triStart; i < n.triStart + n.triCount; ++i) {
                float t; if (moeller(o, d, tris[i], t) && t > tMin) hits.push_back(t);
            }
            return;
        }
        collectNode(n.left, o, d, inv, tMin, hits);
        collectNode(n.right, o, d, inv, tMin, hits);
    }

    static bool moeller(const Vec3& o, const Vec3& d, const Triangle& tri, float& t) {
        constexpr float EPS = 1e-7f;
        Vec3 e1 = tri.v1 - tri.v0, e2 = tri.v2 - tri.v0, h = d.cross(e2);
        float a = e1.dot(h);
        if (std::abs(a) < EPS) return false;
        float f = 1.f / a;
        Vec3 s = o - tri.v0;
        float u = f * s.dot(h);
        if (u < 0.f || u>1.f) return false;
        Vec3 q = s.cross(e1);
        float v = f * d.dot(q);
        if (v < 0.f || u + v>1.f) return false;
        t = f * e2.dot(q);
        return true;
    }
};