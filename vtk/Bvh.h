#pragma once
#include "ObjLoader.h"
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <limits>
#include <cstdint>

// ─────────────────────────────────────────
//  AABB
// ─────────────────────────────────────────
struct AABB {
    Vec3 mn{ 1e30f, 1e30f, 1e30f };
    Vec3 mx{ -1e30f,-1e30f,-1e30f };

    void expand(const Vec3& p) {
        for (int i = 0; i < 3; ++i) { mn[i] = std::min(mn[i], p[i]); mx[i] = std::max(mx[i], p[i]); }
    }
    void expand(const AABB& b) { expand(b.mn); expand(b.mx); }

    float minDistSq(const Vec3& p) const {
        float d = 0.f;
        for (int i = 0; i < 3; ++i) {
            float v = p[i];
            if (v < mn[i]) d += (mn[i] - v) * (mn[i] - v);
            else if (v > mx[i]) d += (v - mx[i]) * (v - mx[i]);
        }
        return d;
    }
    bool intersectRay(const Vec3& o, const Vec3& inv,
        float& tmin, float& tmax) const {
        tmin = -1e30f; tmax = 1e30f;
        for (int i = 0; i < 3; ++i) {
            if (std::isinf(inv[i])) {
                if (o[i]<mn[i] || o[i]>mx[i]) return false;
            }
            else {
                float t1 = (mn[i] - o[i]) * inv[i], t2 = (mx[i] - o[i]) * inv[i];
                if (t1 > t2) std::swap(t1, t2);
                tmin = std::max(tmin, t1); tmax = std::min(tmax, t2);
                if (tmin > tmax) return false;
            }
        }
        return tmax >= 0.f;
    }
};

inline Vec3 triCentroid(const Triangle& t) {
    return (t.v0 + t.v1 + t.v2) * (1.f / 3.f);
}
inline Vec3 triNormal(const Triangle& t) {
    return (t.v1 - t.v0).cross(t.v2 - t.v0);
}

// ─────────────────────────────────────────
//  7-케이스 최근접점 (Ericson 알고리즘)
// ─────────────────────────────────────────
inline Vec3 closestPointOnTriangle(const Vec3& p, const Triangle& tri) {
    Vec3 ab = tri.v1 - tri.v0, ac = tri.v2 - tri.v0, ap = p - tri.v0;
    float d1 = ab.dot(ap), d2 = ac.dot(ap);
    if (d1 <= 0.f && d2 <= 0.f) return tri.v0;               // 꼭짓점 v0

    Vec3  bp = p - tri.v1;
    float d3 = ab.dot(bp), d4 = ac.dot(bp);
    if (d3 >= 0.f && d4 <= d3) return tri.v1;                 // 꼭짓점 v1

    Vec3  cp = p - tri.v2;
    float d5 = ab.dot(cp), d6 = ac.dot(cp);
    if (d6 >= 0.f && d5 <= d6) return tri.v2;                 // 꼭짓점 v2

    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f)                      // 엣지 v0-v1
        return tri.v0 + ab * (d1 / (d1 - d3));

    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f)                      // 엣지 v0-v2
        return tri.v0 + ac * (d2 / (d2 - d6));

    float va = d3 * d6 - d5 * d4;
    if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f)            // 엣지 v1-v2
        return tri.v1 + (tri.v2 - tri.v1) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));

    float inv = 1.f / (va + vb + vc);                            // 삼각형 내부
    return tri.v0 + ab * (vb * inv) + ac * (vc * inv);
}

inline float distSqToTriangle(const Vec3& p, const Triangle& tri) {
    Vec3 d = p - closestPointOnTriangle(p, tri); return d.dot(d);
}

// 최근접점이 어느 Feature에 있는지 구분
enum class Feature { FACE, EDGE_01, EDGE_02, EDGE_12, VERT_0, VERT_1, VERT_2 };

inline Feature closestFeature(const Vec3& p, const Triangle& tri) {
    Vec3 ab = tri.v1 - tri.v0, ac = tri.v2 - tri.v0, ap = p - tri.v0;
    float d1 = ab.dot(ap), d2 = ac.dot(ap);
    if (d1 <= 0.f && d2 <= 0.f) return Feature::VERT_0;

    Vec3  bp = p - tri.v1;
    float d3 = ab.dot(bp), d4 = ac.dot(bp);
    if (d3 >= 0.f && d4 <= d3) return Feature::VERT_1;

    Vec3  cp = p - tri.v2;
    float d5 = ab.dot(cp), d6 = ac.dot(cp);
    if (d6 >= 0.f && d5 <= d6) return Feature::VERT_2;

    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f) return Feature::EDGE_01;

    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f) return Feature::EDGE_02;

    float va = d3 * d6 - d5 * d4;
    if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f) return Feature::EDGE_12;

    return Feature::FACE;
}

// ─────────────────────────────────────────
//  BVH Node / BVH
// ─────────────────────────────────────────
struct BVHNode {
    AABB bounds;
    int  left = -1, right = -1;
    int  triStart = 0, triCount = 0;
};

// 엣지 키 (정점 인덱스 쌍, 순서 무관)
inline uint64_t edgeKey(int a, int b) {
    if (a > b) std::swap(a, b);
    return ((uint64_t)a << 32) | (uint64_t)b;
}

class BVH {
public:
    std::vector<Triangle> tris;
    std::vector<BVHNode>  nodes;

    // Pseudonormal 테이블
    std::unordered_map<int, Vec3>              vertNormal; // vertex → angle-weighted normal
    std::unordered_map<uint64_t, std::pair<Vec3, Vec3>> edgeNormal; // edge → (n0+n1), second face normal

    static constexpr int LEAF = 4;

    void build(const std::vector<Triangle>& triangles) {
        tris = triangles;
        nodes.clear();
        nodes.reserve(2 * (int)tris.size());
        buildNode(0, (int)tris.size());
        buildPseudoNormals();
    }

    float nearestDistSq(const Vec3& p, int& closestIdx) const {
        float best = 1e30f; closestIdx = -1;
        queryNearest(0, p, best, closestIdx);
        return best;
    }

    void collectHits(const Vec3& orig, const Vec3& dir,
        float tMin, std::vector<float>& hits) const {
        Vec3 inv = { dir.x == 0 ? 1e30f : 1.f / dir.x,
                  dir.y == 0 ? 1e30f : 1.f / dir.y,
                  dir.z == 0 ? 1e30f : 1.f / dir.z };
        hits.clear();
        collectNode(0, orig, dir, inv, tMin, hits);
    }

    // ── Angle-weighted pseudonormal 기반 부호 결정 ──────────
    //    Feature에 따라 적절한 법선을 골라 dot 부호 반환
    float signAt(const Vec3& p, int triIdx) const {
        const Triangle& tri = tris[triIdx];
        Feature feat = closestFeature(p, tri);
        Vec3 cp = closestPointOnTriangle(p, tri);
        Vec3 toP = p - cp;

        Vec3 N = { 0,0,0 };

        switch (feat) {
        case Feature::FACE:
            N = triNormal(tri);
            break;

        case Feature::VERT_0: {
            auto it = vertNormal.find(tri.i0);
            if (it != vertNormal.end()) N = it->second;
            else N = triNormal(tri);
            break;
        }
        case Feature::VERT_1: {
            auto it = vertNormal.find(tri.i1);
            if (it != vertNormal.end()) N = it->second;
            else N = triNormal(tri);
            break;
        }
        case Feature::VERT_2: {
            auto it = vertNormal.find(tri.i2);
            if (it != vertNormal.end()) N = it->second;
            else N = triNormal(tri);
            break;
        }
        case Feature::EDGE_01: {
            auto it = edgeNormal.find(edgeKey(tri.i0, tri.i1));
            if (it != edgeNormal.end()) N = it->second.first;
            else N = triNormal(tri);
            break;
        }
        case Feature::EDGE_02: {
            auto it = edgeNormal.find(edgeKey(tri.i0, tri.i2));
            if (it != edgeNormal.end()) N = it->second.first;
            else N = triNormal(tri);
            break;
        }
        case Feature::EDGE_12: {
            auto it = edgeNormal.find(edgeKey(tri.i1, tri.i2));
            if (it != edgeNormal.end()) N = it->second.first;
            else N = triNormal(tri);
            break;
        }
        }
        return toP.dot(N); // 양수=바깥, 음수=안쪽
    }

private:
    // ── angle-weighted pseudonormal 사전 빌드 ───────────────
    void buildPseudoNormals() {
        vertNormal.clear();
        edgeNormal.clear();

        for (const auto& tri : tris) {
            Vec3 n = triNormal(tri).normalized();
            if (n.lengthSq() < 1e-20f) continue;

            // 각 꼭짓점에서 대응 각도 계산 (angle-weighted)
            auto angleAt = [](const Vec3& a, const Vec3& b, const Vec3& c) {
                Vec3 ab = (b - a).normalized(), ac = (c - a).normalized();
                float d = ab.dot(ac);
                d = std::max(-1.f, std::min(1.f, d));
                return std::acos(d);
                };

            float a0 = angleAt(tri.v0, tri.v1, tri.v2);
            float a1 = angleAt(tri.v1, tri.v0, tri.v2);
            float a2 = angleAt(tri.v2, tri.v0, tri.v1);

            vertNormal[tri.i0] = vertNormal[tri.i0] + n * a0;
            vertNormal[tri.i1] = vertNormal[tri.i1] + n * a1;
            vertNormal[tri.i2] = vertNormal[tri.i2] + n * a2;

            // 엣지 법선: 공유 엣지의 두 삼각형 법선 합산
            auto addEdge = [&](int a, int b) {
                uint64_t k = edgeKey(a, b);
                auto& e = edgeNormal[k];
                e.first = e.first + n;  // 누적
                };
            addEdge(tri.i0, tri.i1);
            addEdge(tri.i0, tri.i2);
            addEdge(tri.i1, tri.i2);
        }
    }

    int buildNode(int start, int end) {
        BVHNode node; node.triStart = start; node.triCount = end - start;
        for (int i = start; i < end; ++i) {
            node.bounds.expand(tris[i].v0);
            node.bounds.expand(tris[i].v1);
            node.bounds.expand(tris[i].v2);
        }
        int idx = (int)nodes.size(); nodes.push_back(node);
        if (end - start <= LEAF) return idx;

        Vec3 ext = node.bounds.mx - node.bounds.mn;
        int axis = 0; if (ext[1] > ext[0])axis = 1; if (ext[2] > ext[axis])axis = 2;
        int mid = (start + end) / 2;
        std::nth_element(tris.begin() + start, tris.begin() + mid, tris.begin() + end,
            [axis](const Triangle& a, const Triangle& b) {
                return triCentroid(a)[axis] < triCentroid(b)[axis];
            });
        int l = buildNode(start, mid), r = buildNode(mid, end);
        nodes[idx].left = l; nodes[idx].right = r; nodes[idx].triCount = 0;
        return idx;
    }

    void queryNearest(int idx, const Vec3& p, float& best, int& bestTri) const {
        const BVHNode& n = nodes[idx];
        if (n.bounds.minDistSq(p) >= best) return;
        if (n.left == -1) {
            for (int i = n.triStart; i < n.triStart + n.triCount; ++i) {
                float d = distSqToTriangle(p, tris[i]);
                if (d < best) { best = d; bestTri = i; }
            }
            return;
        }
        float dl = nodes[n.left].bounds.minDistSq(p);
        float dr = nodes[n.right].bounds.minDistSq(p);
        if (dl < dr) { queryNearest(n.left, p, best, bestTri); queryNearest(n.right, p, best, bestTri); }
        else { queryNearest(n.right, p, best, bestTri); queryNearest(n.left, p, best, bestTri); }
    }

    void collectNode(int idx, const Vec3& o, const Vec3& d, const Vec3& inv,
        float tMin, std::vector<float>& hits) const {
        const BVHNode& n = nodes[idx];
        float t0, t1; if (!n.bounds.intersectRay(o, inv, t0, t1)) return;
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
        float a = e1.dot(h); if (std::abs(a) < EPS) return false;
        float f = 1.f / a; Vec3 s = o - tri.v0;
        float u = f * s.dot(h); if (u < 0.f || u>1.f) return false;
        Vec3 q = s.cross(e1); float v = f * d.dot(q);
        if (v < 0.f || u + v>1.f) return false;
        t = f * e2.dot(q); return true;
    }
};