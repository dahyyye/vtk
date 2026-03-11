#pragma once
#include "Bvh.h"
#include <vector>
#include <algorithm>
#include <cstdint>
#include <functional>
#include <cmath>

// 式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式
//  SDF Voxelizer
//
//  For each voxel:
//   1. BVH nearest-point query  ⊥ unsigned distance to surface
//   2. Parity ray test (+X ray) ⊥ sign  (negative = inside)
//
//  Output: Float32 signed distance field  (same as Torus_128.vti)
// 式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式

// Returns true if point is inside mesh (odd parity along +X)
static bool isInside(const Vec3& p, const BVH& bvh) {
    Vec3 dir = { 1.f, 0.f, 0.f };
    Vec3 orig = { p.x - 1e4f, p.y, p.z };  // start far left
    std::vector<float> hits;
    bvh.collectHits(orig, dir, 0.f, hits);

    // Deduplicate near-coincident hits
    std::sort(hits.begin(), hits.end());
    std::vector<float> deduped;
    for (float h : hits)
        if (deduped.empty() || h - deduped.back() > 1e-4f)
            deduped.push_back(h);

    // Count hits that fall before our point
    float dist = p.x - orig.x;
    int cnt = (int)(std::lower_bound(deduped.begin(), deduped.end(), dist) - deduped.begin());
    return (cnt & 1) != 0;
}

inline std::vector<float> voxelize(
    const Mesh& mesh,
    const BVH& bvh,
    int         res,
    const std::function<void(int, int)>& progress = nullptr)
{
    std::vector<float> vol(static_cast<size_t>(res) * res * res, 0.f);

    // Padded bounds
    Vec3  bmin = mesh.bmin, bmax = mesh.bmax;
    float maxExt = 0.f;
    for (int i = 0; i < 3; ++i) maxExt = std::max(maxExt, bmax[i] - bmin[i]);
    float pad = maxExt * 0.02f;
    for (int i = 0; i < 3; ++i) { bmin[i] -= pad; bmax[i] += pad; }

    float dx = (bmax.x - bmin.x) / res;
    float dy = (bmax.y - bmin.y) / res;
    float dz = (bmax.z - bmin.z) / res;

    for (int iz = 0; iz < res; ++iz) {
        if (progress) progress(iz, res);
        float wz = bmin.z + (iz + 0.5f) * dz;

        for (int iy = 0; iy < res; ++iy) {
            float wy = bmin.y + (iy + 0.5f) * dy;

            for (int ix = 0; ix < res; ++ix) {
                float wx = bmin.x + (ix + 0.5f) * dx;
                Vec3 p = { wx, wy, wz };

                // Unsigned distance to nearest surface point
                float distSq = bvh.nearestDistSq(p);
                float dist = std::sqrt(distSq);

                // Sign: negative inside, positive outside
                if (isInside(p, bvh)) dist = -dist;

                size_t idx = (size_t)ix + (size_t)iy * res + (size_t)iz * res * res;
                vol[idx] = dist;
            }
        }
    }
    return vol;
}